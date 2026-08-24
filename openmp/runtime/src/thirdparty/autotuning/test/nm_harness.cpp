//===-- nm_harness.cpp - Banco de testes standalone do Nelder-Mead --------===//
//
// Exercita openmp/runtime/src/thirdparty/autotuning/NelderMead.cpp fora da
// libomp, com um modelo sintético de custo de chunk size.  Não precisa do
// clang/libomp patchados: compila com qualquer C++17.
//
//   make && ./nm_harness
//
// Cada cenário roda num processo filho, de modo que um SIGFPE/SIGSEGV/loop
// infinito num cenário não esconde os demais -- o pai relata o sinal.
//
// Modelo de custo (unimodal, ótimo conhecido):
//
//     cost(x) = overhead / x  +  imbalance * x
//
// O primeiro termo modela o custo de dispatch (N/x operações de despacho), o
// segundo modela o desbalanceamento de carga na cauda (proporcional ao chunk).
// O mínimo real fica em x* = sqrt(overhead / imbalance).
//
//===----------------------------------------------------------------------===//

#include "NelderMead.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <limits>

#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

// Definido aqui porque o shim kmp_debug.h só o declara.
int nm_shim_assert_count = 0;

//===----------------------------------------------------------------------===//
// Modelo de custo
//===----------------------------------------------------------------------===//

struct Model {
  double overhead;  ///< custo de despacho agregado
  double imbalance; ///< custo de desbalanceamento por unidade de chunk
  double noise;     ///< amplitude relativa do ruído (0.0 = determinístico)
};

static double objective(const Model &m, int64_t x) {
  // Um chunk <= 0 é inválido. Na libomp real ele é silenciosamente clampado
  // (kmp_dispatch.cpp:302 / kmp_sched.cpp:374); aqui damos custo proibitivo
  // para que o otimizador tenha incentivo a sair dali, e contamos à parte.
  if (x <= 0)
    return 1e300;
  double c = m.overhead / (double)x + m.imbalance * (double)x;
  if (m.noise > 0.0) {
    double u = (double)rand() / (double)RAND_MAX * 2.0 - 1.0; // [-1, 1]
    c *= (1.0 + m.noise * u);
  }
  return c;
}

static double true_optimum(const Model &m) {
  return std::sqrt(m.overhead / m.imbalance);
}

/// Custo sem ruído, para avaliar um ponto de forma determinística.
static double clean_cost(const Model &m, int64_t x) {
  if (x <= 0)
    return 1e300;
  return m.overhead / (double)x + m.imbalance * (double)x;
}

/// Melhor custo alcançável por um chunk INTEIRO dentro de [min, max].
/// O ótimo analítico é real; o algoritmo só pode devolver inteiros.
static double best_integer_cost(const Model &m, int64_t lo, int64_t hi) {
  const double opt = true_optimum(m);
  int64_t cands[2] = {(int64_t)std::floor(opt), (int64_t)std::ceil(opt)};
  double best = 1e300;
  for (int i = 0; i < 2; ++i) {
    int64_t x = cands[i];
    if (x < lo)
      x = lo;
    if (x > hi)
      x = hi;
    const double c = clean_cost(m, x);
    if (c < best)
      best = c;
  }
  return best;
}

//===----------------------------------------------------------------------===//
// Resultado de um cenário (POD, vive em memória compartilhada)
//===----------------------------------------------------------------------===//

struct Result {
  int completed;         ///< filho terminou o cenário sem morrer
  int converged;         ///< isEnd() ficou true dentro do limite de iterações
  int iters;             ///< iterações consumidas
  int asserts;           ///< KMP_ASSERT2 disparados dentro do NelderMead
  long long out_of_range; ///< pontos devolvidos fora de [min, max]
  long long nonpositive;  ///< pontos devolvidos <= 0 (chunk inválido)
  long long min_seen;
  long long max_seen;
  long long final_point;
  long long best_x_seen;
  double best_cost_seen;
  double final_cost;
};

static void result_init(Result *r) {
  std::memset(r, 0, sizeof(*r));
  r->min_seen = std::numeric_limits<long long>::max();
  r->max_seen = std::numeric_limits<long long>::min();
  r->best_cost_seen = std::numeric_limits<double>::max();
}

//===----------------------------------------------------------------------===//
// Cenário de busca
//===----------------------------------------------------------------------===//

struct Scenario {
  const char *name;
  int64_t min;
  int64_t max;
  Model model;
  int64_t error; ///< critério de parada (m_error), 1 é o que a libomp usa
  const char *note;
  bool expect_converge; ///< false = não convergir é o resultado esperado
};

static const int MAX_ITERS = 400;

static void run_search(const Scenario &s, Result *r) {
  result_init(r);

  // Mesma construção que kmp_autotuning.cpp: DIM = 1, MIN_ERROR = 1.
  NelderMead *nm = NelderMead::Create(s.min, s.max, /*dim=*/1, s.error);

  double cost = 0.0; // a 1a chamada descarta o custo (m_iCost == NO_SAVE)
  int it = 0;
  for (; it < MAX_ITERS && !nm->isEnd(); ++it) {
    int64_t *p = nm->run(cost);
    int64_t x = p[0];

    r->min_seen = std::min<long long>(r->min_seen, x);
    r->max_seen = std::max<long long>(r->max_seen, x);
    if (x < s.min || x > s.max)
      ++r->out_of_range;
    if (x <= 0)
      ++r->nonpositive;

    cost = objective(s.model, x);
    if (x > 0 && cost < r->best_cost_seen) {
      r->best_cost_seen = cost;
      r->best_x_seen = x;
    }
    r->final_point = x;
    r->final_cost = cost;
  }

  r->iters = it;
  r->converged = nm->isEnd() ? 1 : 0;
  if (r->converged)
    r->final_point = nm->getMinPoint()[0];
  r->asserts = nm_shim_assert_count;
  r->completed = 1;
}

//===----------------------------------------------------------------------===//
// Cenário: setLimits com min >= max
//===----------------------------------------------------------------------===//
//
// __kmp_start_autotuning (kmp_autotuning.h:191-196) recalcula min/max a cada
// execução do loop e chama setLimits() SEM o guarda "min >= max" que existe em
// __kmp_init_autotuning.  Basta o número de threads crescer ou o trip count
// encolher para que m_max == m_min, e então circ_mod()/rand_gen() fazem
// "% (m_max - m_min)" == "% 0".

static void run_degenerate(const Scenario &s, Result *r) {
  result_init(r);

  NelderMead *nm = NelderMead::Create(s.min, s.max, /*dim=*/1, s.error);

  // Passa da fase de init (3 pontos) para chegar à reflexão, que é onde
  // calculate_point() -> circ_mod() é chamado.
  double cost = 0.0;
  for (int i = 0; i < 4; ++i) {
    int64_t *p = nm->run(cost);
    cost = objective(s.model, p[0]);
  }

  // Agora o intervalo degenera: min == max.
  nm->setLimits(s.min, s.min);

  for (int i = 0; i < 8 && !nm->isEnd(); ++i) {
    int64_t *p = nm->run(cost);
    int64_t x = p[0];
    if (x < s.min || x > s.max)
      ++r->out_of_range;
    if (x <= 0)
      ++r->nonpositive;
    cost = objective(s.model, x);
    r->final_point = x;
  }

  r->asserts = nm_shim_assert_count;
  r->completed = 1;
}

//===----------------------------------------------------------------------===//
// Cenário: dois otimizadores criados em sequência
//===----------------------------------------------------------------------===//
//
// NelderMead::Create chama srand(time(NULL)) (NelderMead.cpp:91).  Dois loops
// "auto" inicializados no mesmo segundo recebem a MESMA semente e portanto o
// MESMO simplex inicial -- e a semente global da aplicação é destruída.

static void run_duplicate_seed(Result *r) {
  result_init(r);

  int64_t a[3], b[3];
  NelderMead *n1 = NelderMead::Create(1, 4096, 1, 1);
  double c = 0.0;
  for (int i = 0; i < 3; ++i) {
    a[i] = n1->run(c)[0];
    c = 1.0;
  }
  NelderMead *n2 = NelderMead::Create(1, 4096, 1, 1);
  c = 0.0;
  for (int i = 0; i < 3; ++i) {
    b[i] = n2->run(c)[0];
    c = 1.0;
  }

  std::printf("        simplex #1: [%lld, %lld, %lld]\n", (long long)a[0],
              (long long)a[1], (long long)a[2]);
  std::printf("        simplex #2: [%lld, %lld, %lld]\n", (long long)b[0],
              (long long)b[1], (long long)b[2]);

  bool identical = (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
  r->out_of_range = identical ? 1 : 0; // reaproveitado como flag
  r->completed = 1;
}

//===----------------------------------------------------------------------===//
// Execução isolada em processo filho
//===----------------------------------------------------------------------===//

static const char *signame(int sig) {
  switch (sig) {
  case SIGFPE:  return "SIGFPE (divisão por zero)";
  case SIGSEGV: return "SIGSEGV";
  case SIGABRT: return "SIGABRT";
  case SIGALRM: return "SIGALRM (travou / loop infinito)";
  case SIGBUS:  return "SIGBUS";
  default:      return "sinal desconhecido";
  }
}

/// Roda \p body num processo filho. Devolve 0 se saiu normalmente, senão o
/// número do sinal que o matou.
template <typename F> static int isolated(Result *shared, F body) {
  result_init(shared);
  pid_t pid = fork();
  if (pid < 0) {
    std::perror("fork");
    std::exit(2);
  }
  if (pid == 0) {
    alarm(15); // watchdog contra loop infinito dentro de run()
    body(shared);
    std::_Exit(0);
  }
  int status = 0;
  waitpid(pid, &status, 0);
  if (WIFSIGNALED(status))
    return WTERMSIG(status);
  return 0;
}

//===----------------------------------------------------------------------===//
// main
//===----------------------------------------------------------------------===//

static const Scenario kScenarios[] = {
    // nome            min   max     overhead imbal noise  err  nota
    {"typical",          1, 6250, {62500.0, 1.0, 0.00},   1,
     "N=100k, 8 threads; ótimo ~250", true},
    {"small-range",      1,   64, {  256.0, 1.0, 0.00},   1,
     "intervalo curto; ótimo ~16", true},
    {"tiny-range",       1,    4, {    4.0, 1.0, 0.00},   1,
     "intervalo menor que o simplex; ótimo 2", true},
    {"offset-min",     101,  200, {22500.0, 1.0, 0.00},   1,
     "min != 1 (caso do inner loop de distribute); ótimo 150", true},
    {"noisy",            1, 6250, {62500.0, 1.0, 0.10},   1,
     "ruído de +-10% na medição; ótimo ~250", true},
    // Calibração do critério de parada. simplex_size() é o desvio padrão
    // populacional dos pontos; para 3 pontos inteiros os valores possíveis
    // perto da convergência são:
    //   (x,x,x)     -> 0.000     (x,x,x+1)   -> 0.471
    //   (x,x+1,x+2) -> 0.816     (x,x,x+2)   -> 0.943
    //   (x,x+1,x+3) -> 1.247     (x,x,x+3)   -> 1.414
    // Logo m_error=1 para quando os 3 pontos cabem num intervalo de 2.
    {"err0",             1, 6250, {62500.0, 1.0, 0.00},   0,
     "m_error=0: exige os 3 pontos IDÊNTICOS", false},
    {"err1",             1, 6250, {62500.0, 1.0, 0.00},   1,
     "m_error=1: o valor atual", true},
    {"err2",             1, 6250, {62500.0, 1.0, 0.00},   2,
     "m_error=2: para com os pontos mais espalhados", true},
};

static int report(const Scenario &s, const Result &r, int sig) {
  std::printf("  %-12s  %s\n", s.name, s.note);
  std::printf("        intervalo válido: [%lld, %lld]   ótimo real: %.1f\n",
              (long long)s.min, (long long)s.max, true_optimum(s.model));

  if (sig) {
    std::printf("        \033[31mCRASH: %s\033[0m\n\n", signame(sig));
    return 1;
  }
  if (!r.completed) {
    std::printf("        \033[31mfilho não completou\033[0m\n\n");
    return 1;
  }

  std::printf("        pontos devolvidos: [%lld, %lld]   iterações: %d   "
              "convergiu: %s\n",
              r.min_seen, r.max_seen, r.iters, r.converged ? "sim" : "NÃO");
  std::printf("        ponto final: %lld   melhor visto: %lld (custo %.4g)\n",
              r.final_point, r.best_x_seen, r.best_cost_seen);

  int fails = 0;
  if (r.nonpositive > 0) {
    std::printf("        \033[31mFALHA: %lld pontos <= 0 (chunk inválido)"
                "\033[0m\n",
                r.nonpositive);
    ++fails;
  }
  if (r.out_of_range > 0) {
    std::printf("        \033[31mFALHA: %lld pontos fora de [%lld, %lld]"
                "\033[0m\n",
                r.out_of_range, (long long)s.min, (long long)s.max);
    ++fails;
  }
  if (!r.converged) {
    if (s.expect_converge) {
      std::printf("        \033[31mFALHA: não convergiu em %d iterações"
                  "\033[0m\n",
                  MAX_ITERS);
      ++fails;
    } else {
      std::printf("        não convergiu em %d iterações -- ESPERADO\n",
                  MAX_ITERS);
    }
  } else if (!s.expect_converge) {
    std::printf("        \033[31mFALHA: convergiu, mas não deveria\033[0m\n");
    ++fails;
  } else {
    // O critério é o EXCESSO DE CUSTO, não a distância até o ótimo. Num
    // intervalo pequeno, errar por um inteiro é uma distância relativa enorme
    // (em [1,4], 3 em vez de 2 são "50%") mas um custo quase igual (8%). Para
    // um autoajustador de chunk, o que importa é quanto se perde de
    // desempenho, não onde o ponto caiu.
    const double cost_final = clean_cost(s.model, r.final_point);
    const double cost_best = best_integer_cost(s.model, s.min, s.max);
    const double excess = cost_final / cost_best - 1.0;
    const double opt = true_optimum(s.model);
    if (excess > 0.15) {
      std::printf("        \033[31mFALHA: custo %.4g contra %.4g do melhor "
                  "inteiro (+%.1f%%); ponto %lld vs ótimo %.1f\033[0m\n",
                  cost_final, cost_best, excess * 100.0, r.final_point, opt);
      ++fails;
    } else {
      std::printf("        excesso de custo: +%.1f%%  (ponto %lld, ótimo real "
                  "%.1f)\n",
                  excess * 100.0, r.final_point, opt);
    }
  }
  if (r.asserts > 0) {
    std::printf("        \033[31mFALHA: %d KMP_ASSERT2 disparados\033[0m\n",
                r.asserts);
    ++fails;
  }

  std::printf("        %s\n\n",
              fails ? "\033[31m=> FALHOU\033[0m" : "\033[32m=> OK\033[0m");
  return fails ? 1 : 0;
}

int main(int argc, char **argv) {
  bool only_degenerate = (argc > 1 && std::strcmp(argv[1], "degenerate") == 0);

  Result *shared = (Result *)mmap(nullptr, sizeof(Result),
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (shared == MAP_FAILED) {
    std::perror("mmap");
    return 2;
  }

  int failures = 0;

  if (!only_degenerate) {
    std::printf("\n=== Busca (modelo de custo unimodal) ===\n\n");
    for (const Scenario &s : kScenarios) {
      int sig = isolated(shared, [&s](Result *r) { run_search(s, r); });
      failures += report(s, *shared, sig);
    }

    std::printf("=== Semente do simplex inicial ===\n\n");
    std::printf("  duplicate-seed  dois otimizadores criados em sequência\n");
    int sig = isolated(shared, [](Result *r) { run_duplicate_seed(r); });
    if (sig) {
      std::printf("        \033[31mCRASH: %s\033[0m\n\n", signame(sig));
      ++failures;
    } else if (shared->out_of_range) {
      std::printf("        \033[31mFALHA: simplex inicial idêntico -- "
                  "srand(time(NULL)) em Create()\033[0m\n");
      std::printf("        \033[31m=> FALHOU\033[0m\n\n");
      ++failures;
    } else {
      std::printf("        simplexes diferentes\n        \033[32m=> OK"
                  "\033[0m\n\n");
    }
  }

  std::printf("=== Intervalo degenerado (setLimits(min, min)) ===\n\n");
  std::printf("  degenerate    __kmp_start_autotuning chama setLimits() sem "
              "checar min < max\n");
  {
    Scenario s = kScenarios[0];
    int sig = isolated(shared, [&s](Result *r) { run_degenerate(s, r); });
    if (sig) {
      std::printf("        \033[31mCRASH: %s\033[0m\n", signame(sig));
      std::printf("        \033[31m=> FALHOU\033[0m\n\n");
      ++failures;
    } else {
      std::printf("        sobreviveu; ponto final %lld\n", shared->final_point);
      std::printf("        \033[32m=> OK\033[0m\n\n");
    }
  }

  std::printf("%s: %d cenário(s) com falha\n\n",
              failures ? "\033[31mRESULTADO\033[0m" : "\033[32mRESULTADO\033[0m",
              failures);
  return failures ? 1 : 0;
}
