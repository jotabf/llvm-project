#include "NelderMead.h"

#include "kmp.h"
#include "kmp_debug.h"
#include "kmp_os.h"

#include <cmath> // pow, sqrt, fmod
#include <cstdint> // uint64_t, uintptr_t
#include <cstdlib> // getenv, strtoull
#include <cstring> // memcpy
#include <ctime> // time
#include <limits> // std::numeric_limits
#include <sstream> // std::stringstream
#include <string> // std::string

// Constants for algorithm parameters
#ifndef NM_ALFA
#define NM_ALFA 1 ///< Default alfa value to reflection operation.
#endif
#ifndef NM_GAMA
#define NM_GAMA 2 ///< Default gama value to expansion operation.
#endif
#ifndef NM_RHO
#define NM_RHO 0.5 ///< Default rho value to contraction operation.
#endif
#ifndef NM_SIGMA
#define NM_SIGMA 0.5 ///< Default sigma value to shrink operation.
#endif

const double NelderMead::m_alpha = NM_ALFA; // Used in reflection
const double NelderMead::m_gamma = NM_GAMA; // Used in expansion
const double NelderMead::m_rho = NM_RHO; // Used in contraction
const double NelderMead::m_sigma = NM_SIGMA; // Used in reduction

template <typename T> inline double todouble(T x) {
  return static_cast<double>(x);
}

template <typename T> inline int64_t toint64(T x) {
  return static_cast<int64_t>(x);
}

inline int64_t NelderMead::circ_mod(int64_t x) const {
  const int64_t span = m_max - m_min;
  // setLimits() é público e não tem pré-condição: __kmp_start_autotuning
  // recalcula min/max a cada execução do loop, e basta o trip count encolher
  // ou o número de threads crescer para span virar 0 (ou negativo). Sem esta
  // guarda o "% span" abaixo é SIGFPE.
  if (span <= 0)
    return m_min;
  if (x < m_min)
    return (x - m_min) % span + m_max;
  if (x > m_max)
    return (x - m_max) % span + m_min;
  return x;
}

//===----------------------------------------------------------------------===//
// PRNG
//===----------------------------------------------------------------------===//
//
// splitmix64: 8 bytes de estado, sem alocação, sem estado global e sem
// dependência da libc. Cabe direto no objeto, que é criado por __kmp_allocate
// (memória crua, sem construtor), então tem de ser POD.

static inline uint64_t nm_splitmix64(uint64_t &state) {
  uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

/// Semente base do processo e contador de otimizadores. Cada NelderMead recebe
/// um fluxo distinto, de modo que dois loops "auto" nunca partem do mesmo
/// simplex. Defina KMP_AUTOTUNING_SEED para tornar o experimento reprodutível.
static uint64_t nm_seed_base = 0;
static uint64_t nm_seed_count = 0;

static uint64_t nm_next_seed() {
  if (nm_seed_base == 0) {
    const char *env = getenv("KMP_AUTOTUNING_SEED");
    if (env && *env)
      nm_seed_base = (uint64_t)strtoull(env, NULL, 0);
    if (nm_seed_base == 0)
      nm_seed_base = (uint64_t)time(NULL) ^
                     ((uint64_t)(uintptr_t)&nm_seed_base << 16);
  }
  // Create() roda sob info->start_lock no caminho da libomp, então o
  // incremento não precisa ser atômico aqui.
  return nm_seed_base + (++nm_seed_count) * 0x9E3779B97F4A7C15ULL;
}

inline int64_t NelderMead::rand_gen() {
  // span >= 1 mesmo quando m_max == m_min, então esta função não divide por
  // zero nem quando os limites degeneram (ao contrário de circ_mod).
  const uint64_t span = (uint64_t)(m_max - m_min) + 1ULL;
  return m_min + (int64_t)(nm_splitmix64(m_rng) % span);
}

void NelderMead::setSeed(uint64_t seed) {
  m_rng = seed;
  for (unsigned i = 0; i < m_nPoints; ++i)
    for (unsigned j = 0; j < m_dim; ++j)
      p_points[i][j] = rand_gen();
}

NelderMead *NelderMead::Create(int64_t min, int64_t max, unsigned dim,
                               int64_t error) {

  size_t size = sizeof(NelderMead);
  NelderMead *nm = static_cast<NelderMead *>(__kmp_allocate(size));

  nm->m_dim = dim;
  nm->m_nPoints = (dim > 2) ? (dim + 1) : 3;
  nm->m_error = error;
  nm->m_worstID = nm->m_nPoints - 1;
  nm->m_secondID = nm->m_nPoints - 2;
  nm->m_step = NelderMead::steps::init;
  nm->m_iPoint = 0;
  nm->m_iCost = NelderMead::NO_SAVE;
  nm->m_costReflection = 0.0; // Cost of reflected point
  nm->m_costExpansion = 0.0; // Cost of expanded point
  nm->m_costContraction = 0.0; // Cost of contracted point
  nm->m_min = min;
  nm->m_max = max;

  nm->p_costs =
      static_cast<double *>(__kmp_allocate(nm->m_nPoints * sizeof(double)));
  nm->p_points = static_cast<int64_t **>(
      __kmp_allocate(nm->m_nPoints * sizeof(int64_t *)));
  for (size_t i = 0; i < nm->m_nPoints; i++) {
    nm->p_points[i] =
        static_cast<int64_t *>(__kmp_allocate(nm->m_dim * sizeof(int64_t)));
  }
  nm->p_centroid =
      static_cast<int64_t *>(__kmp_allocate(nm->m_dim * sizeof(int64_t)));
  nm->p_pointReflection =
      static_cast<int64_t *>(__kmp_allocate(nm->m_dim * sizeof(int64_t)));
  nm->p_pointExpansion =
      static_cast<int64_t *>(__kmp_allocate(nm->m_dim * sizeof(int64_t)));
  nm->p_pointContraction =
      static_cast<int64_t *>(__kmp_allocate(nm->m_dim * sizeof(int64_t)));

  // Simplex inicial. Cada otimizador tem seu próprio fluxo de números
  // aleatórios; nada de srand(), que destruía a semente global da aplicação.
  nm->m_rng = nm_next_seed();
  for (unsigned i = 0; i < nm->m_nPoints; i++) {
    for (unsigned j = 0; j < nm->m_dim; j++) {
      nm->p_points[i][j] = nm->rand_gen();
    }
  }

  for (unsigned i = 0; i < nm->m_nPoints; i++) {
    nm->p_costs[i] = std::numeric_limits<double>::max();
  }

  return nm;
}

void NelderMead::Destroy(NelderMead *optimizer) {
  __kmp_free(optimizer->p_pointContraction);
  __kmp_free(optimizer->p_pointExpansion);
  __kmp_free(optimizer->p_pointReflection);
  __kmp_free(optimizer->p_centroid);
  for (size_t i = 0; i < optimizer->m_nPoints; i++) {
    __kmp_free(optimizer->p_points[i]);
  }
  __kmp_free(optimizer->p_points);
  __kmp_free(optimizer->p_costs);
  __kmp_free(optimizer);
}

int64_t *NelderMead::run(double _cost) {
  do {
    switch (m_step) {
    case steps::init: // Initialize Points
      if (m_iCost != NO_SAVE) {
        p_costs[m_iCost] = _cost;
      }
      if (m_iPoint < m_nPoints) {
        auto point = p_points[m_iPoint];
        m_iCost = m_iPoint;
        m_iPoint++;
        return point;
      }
      m_step = steps::reflection;
      continue;

    case steps::reflection: // Reflection - Use m_alpha
      sort_points();
      calculate_centroid();
      // Xr = Xo + (-alpha) * (Xn+1 - Xo)
      calculate_point(p_pointReflection, -m_alpha, p_points[m_worstID],
                      p_centroid);
      m_step = steps::decision;
      return p_pointReflection;

    case steps::decision:
      m_costReflection = _cost;
      switch (make_decision(_cost)) {
      case steps::decision_reflection:
        std::swap(p_pointReflection, p_points[m_worstID]);
        p_costs[m_worstID] = m_costReflection;
        m_step = steps::reflection;
        break;
      case steps::decision_expansion:
        // Xe = Xo + gamma * (Xr - Xo)
        calculate_point(p_pointExpansion, m_gamma, p_pointReflection,
                        p_centroid);
        m_step = steps::expansion;
        return p_pointExpansion;
      case steps::decision_out_contraction:
        // Xc = Xo + rho * (Xr - Xo)
        calculate_point(p_pointContraction, m_rho, p_pointReflection,
                        p_centroid);
        m_step = steps::out_contraction;
        return p_pointContraction;
      case steps::decision_in_contraction:
        // Xc = Xo + rho * (Xn+1 - Xo)
        calculate_point(p_pointContraction, m_rho, p_points[m_worstID],
                        p_centroid);
        m_step = steps::in_contraction;
        return p_pointContraction;
      }
      break;
    case steps::expansion:
      m_costExpansion = _cost;
      if (m_costExpansion < m_costReflection) {
        std::swap(p_pointExpansion, p_points[m_worstID]);
        p_costs[m_worstID] = m_costExpansion;
      } else {
        std::swap(p_pointReflection, p_points[m_worstID]);
        p_costs[m_worstID] = m_costReflection;
      }
      m_step = steps::reflection;
      break;

    case steps::out_contraction:
      m_costContraction = _cost;
      if (m_costContraction <= m_costReflection) {
        std::swap(p_pointContraction, p_points[m_worstID]);
        p_costs[m_worstID] = m_costContraction;
        m_step = steps::reflection;
      } else {
        m_step = steps::reduction;
      }
      break;

    case steps::in_contraction:
      m_costContraction = _cost;
      if (m_costContraction < p_costs[m_worstID]) {
        std::swap(p_pointContraction, p_points[m_worstID]);
        p_costs[m_worstID] = m_costContraction;
        m_step = steps::reflection;
      } else {
        m_step = steps::reduction;
      }
      break;
    default:
      KMP_ASSERT2(false, "Error in Nelder Mead algorithm");
      KMP_BUILTIN_UNREACHABLE;
    }

    if (m_step == steps::reduction) {
      sort_points();
      // Replace all p_points, except the p_bestPoint
      for (unsigned i = 0; i < m_nPoints; i++) {
        if (i != m_bestID) {
          // Xi = Xb + sigma * (Xi - Xb)
          calculate_point(p_points[i], m_sigma, p_points[i],
                          p_points[m_bestID]);
        }
      }
      m_step = steps::init;
      m_iPoint = m_bestID + 1;
      m_iCost = NO_SAVE;
    }

  } while (simplex_size() > m_error);

  // Ordena antes de congelar. Entre o último sort_points() (feito no início do
  // passo de reflexão) e este ponto, os passos de reflexão/expansão/contração
  // já trocaram p_points[m_worstID] por um ponto possivelmente MELHOR que
  // p_points[m_bestID], sem reordenar. Sem este sort, o chunk congelado -- e
  // tudo que getMinPoint() devolve daqui pra frente -- não é necessariamente o
  // melhor ponto medido.
  sort_points();

  m_step = steps::finalization;

  return p_points[m_bestID];
}

int NelderMead::make_decision(double _cost) const {
  if (p_costs[m_bestID] <= _cost && _cost < p_costs[m_secondID])
    return steps::decision_reflection;
  if (_cost < p_costs[m_bestID])
    return steps::decision_expansion;
  if (p_costs[m_secondID] <= _cost && _cost < p_costs[m_worstID])
    return steps::decision_out_contraction;
  if (_cost >= p_costs[m_worstID])
    return steps::decision_in_contraction;

  KMP_ASSERT2(false, "Error in make_decision");
  KMP_BUILTIN_UNREACHABLE;
  return steps::finalization;
}

void NelderMead::sort_points() {
  // Put in order solutions, ie, p_points[0][x] is the best, ...,
  // p_points[n+1][x] is the worst Bubble sort
  for (unsigned i = 0; i < m_nPoints - 1; ++i) {
    for (unsigned j = i + 1; j < m_nPoints; ++j) {
      if (p_costs[i] > p_costs[j]) {
        std::swap(p_points[i], p_points[j]);
        std::swap(p_costs[i], p_costs[j]);
      }
    }
  }
}

void NelderMead::calculate_centroid() {
  // Calculate p_centroid
  for (unsigned j = 0; j < m_dim; ++j) {
    double count = 0;
    p_centroid[j] = 0; // Centroid = {0}
    for (unsigned i = 0; i < m_nPoints; ++i) {
      // The worst solution is not necessary
      if (i != m_worstID) {
        p_centroid[j] += p_points[i][j];
        count += 1.0;
      }
    }
    p_centroid[j] = toint64(round(todouble(p_centroid[j]) / count));
  }
}

inline void NelderMead::calculate_point(int64_t *&p_out, double _const,
                                        int64_t *p_in1, int64_t *p_in2) {
  for (unsigned j = 0; j < m_dim; ++j) {
    int64_t point =
        p_in2[j] + toint64(round(_const * todouble(p_in1[j] - p_in2[j])));
    p_out[j] = circ_mod(point);
  }
}

void NelderMead::reset(unsigned level) {
  // level é unsigned, então "level >= 0" era sempre verdadeiro (-Wtype-limits).
  KMP_ASSERT2((level <= 1),
              "Invalid Nelder Mead reset level value, set 0 <= level <= 1.");

  m_costReflection = 0.0; // Cost of reflected point
  m_costExpansion = 0.0; // Cost of expanded point
  m_costContraction = 0.0; // Cost of contracted point

  m_iPoint = m_bestID + 1;
  m_iCost = NO_SAVE;
  m_step = steps::init;

  sort_points();
  // Sem srand(): rand_gen() continua de onde m_rng parou, o que dá pontos
  // novos sem tocar no estado global nem re-semear com time(NULL) (que dava
  // o mesmo simplex a dois resets no mesmo segundo).
  // Reset point but keep the best solution
  for (unsigned i = 0; i < m_nPoints; ++i) {
    if (i != m_bestID) {
      for (unsigned j = 0; j < m_dim; ++j) {
        p_points[i][j] = rand_gen();
      }
    }
  }

  if (level < 1)
    return;

  m_iPoint = m_bestID;
  for (unsigned j = 0; j < m_dim; ++j) {
    p_points[m_bestID][j] = rand_gen();
  }
}

double NelderMead::simplex_size() const {
  // Calcula o centroide LOCALMENTE, em vez de usar p_centroid.
  //
  // p_centroid só é escrito em calculate_centroid(), chamado apenas no passo
  // de reflexão, e de propósito EXCLUI o pior ponto. Usá-lo aqui media a
  // dispersão em torno de uma referência (a) de antes da última substituição
  // de ponto e (b) que nem é o centroide do conjunto sendo medido. Pior: a
  // primeira avaliação, alcançada pelo "continue" no fim da fase de init,
  // rodava com p_centroid ainda todo zero, medindo distância até a ORIGEM.
  //
  // Como o centroide verdadeiro é o ponto que minimiza a distância RMS,
  // qualquer outra referência SUPERESTIMA o resultado -- o algoritmo rodava
  // mais do que o critério pedia. O caso caro era o passo de redução, que
  // encolhe o simplex e logo em seguida testa a convergência com o centroide
  // de antes do encolhimento, perdendo a parada por um ciclo inteiro de init
  // (3 avaliações, isto é, 3 execuções do loop do usuário).
  //
  // Duas passadas por dimensão para não precisar alocar um vetor temporário
  // (m_dim é dinâmico).
  double total = 0.0;
  for (unsigned j = 0; j < m_dim; ++j) {
    double mean = 0.0;
    for (unsigned i = 0; i < m_nPoints; ++i)
      mean += todouble(p_points[i][j]);
    mean /= todouble(m_nPoints);

    for (unsigned i = 0; i < m_nPoints; ++i) {
      const double d = todouble(p_points[i][j]) - mean;
      total += d * d; // sem o sqrt()+pow(,2) de ida e volta do original
    }
  }
  return sqrt(total / todouble(m_nPoints));
}