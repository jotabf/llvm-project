#include "NelderMead.h"

#include "kmp.h"
#include "kmp_debug.h"
#include "kmp_os.h"

#include <cmath>   // pow, sqrt, fmod
#include <cstring> // memcpy
#include <ctime>   // time
#include <limits>  // std::numeric_limits
#include <sstream> // std::stringstream
#include <string>  // std::string

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

const double NelderMead::m_alpha = NM_ALFA;  // Used in reflection
const double NelderMead::m_gamma = NM_GAMA;  // Used in expansion
const double NelderMead::m_rho = NM_RHO;     // Used in contraction
const double NelderMead::m_sigma = NM_SIGMA; // Used in reduction

inline double circ_modf(double x) {
  if (x > 1.0)
    return fmod(x + 1., 2.) - 1.;
  if (x < 1.0)
    return fmod(x - 1., 2.) + 1.;
  return x;
}

inline double rand_fgen(drand48_data *buffer) {
  double result;
  drand48_r(buffer, &result);
  return (2.0 * result - 1.0);
}

NelderMead *NelderMead::Create(unsigned dim, double error) {

  size_t size = sizeof(NelderMead);
  NelderMead *nm = static_cast<NelderMead *>(__kmp_allocate(size));

  nm->m_dim = dim;
  nm->m_nPoints = (dim > 2) ? (dim + 1) : 3;
  nm->m_error = error;
  nm->m_bestID = 0;
  nm->m_worstID = nm->m_nPoints - 1;
  nm->m_secondID = nm->m_nPoints - 2;
  nm->m_step = NelderMead::steps::init;
  nm->m_iPoint = 0;
#ifdef AT_DEBUG
  nm->m_iIteration = 0;
  nm->m_iEvaluation = 0;
#endif
  nm->m_iCost = NelderMead::NO_SAVE;
  nm->m_costReflection = 0.0;  // Cost of reflected point
  nm->m_costExpansion = 0.0;   // Cost of expanded point
  nm->m_costContraction = 0.0; // Cost of contracted point

  nm->p_points =
      static_cast<double **>(__kmp_allocate(nm->m_nPoints * sizeof(double *)));
  for (size_t i = 0; i < nm->m_nPoints; i++) {
    nm->p_points[i] =
        static_cast<double *>(__kmp_allocate(nm->m_dim * sizeof(double)));
  }
  nm->p_costs =
      static_cast<double *>(__kmp_allocate(nm->m_nPoints * sizeof(double)));
  nm->p_centroid =
      static_cast<double *>(__kmp_allocate(nm->m_dim * sizeof(double)));
  nm->p_pointReflection =
      static_cast<double *>(__kmp_allocate(nm->m_dim * sizeof(double)));
  nm->p_pointExpansion =
      static_cast<double *>(__kmp_allocate(nm->m_dim * sizeof(double)));
  nm->p_pointContraction =
      static_cast<double *>(__kmp_allocate(nm->m_dim * sizeof(double)));

  // Generate inital p_points
  srand48_r(time(NULL), &nm->buffer); // random seed
  for (unsigned i = 0; i < nm->m_nPoints; i++) {
    for (unsigned j = 0; j < nm->m_dim; j++) {
      nm->p_points[i][j] = rand_fgen(&nm->buffer); // numbers between -1 and 1.
    }
  }

  for (unsigned i = 0; i < nm->m_nPoints; i++) {
    nm->p_costs[i] = std::numeric_limits<double>::max();
  }

  return nm;
}

double *NelderMead::run(double _cost) {
#ifdef AT_DEBUG
  ++m_iEvaluation;
#endif
  do {
    switch (m_step) {
    case steps::init: // Initialize Points
      if (m_iCost != NO_SAVE) {
        p_costs[m_iCost] = _cost;
      }
      if (m_iPoint < m_nPoints) {
        auto &point = p_points[m_iPoint];
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
        // swap(p_pointReflection, p_points[m_worstID]);
        std::swap(p_pointReflection, p_points[m_worstID]);
        p_costs[m_worstID] = m_costReflection;
        m_step = steps::reflection;
        break;
      case steps::decision_expansion:
        // Xe = Xo + gamma * (Xr - Xo)
        calculate_point(p_pointReflection, m_gamma, p_pointReflection,
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
        // swap(p_pointExpansion, p_points[m_worstID]);
        std::swap(p_pointExpansion, p_points[m_worstID]);
        p_costs[m_worstID] = m_costExpansion;
      } else {
        // swap(p_pointReflection, p_points[m_worstID]);
        std::swap(p_pointReflection, p_points[m_worstID]);
        p_costs[m_worstID] = m_costReflection;
      }
      m_step = steps::reflection;
      break;

    case steps::out_contraction:
      m_costContraction = _cost;
      if (m_costContraction <= m_costReflection) {
        // swap(p_pointContraction, p_points[m_worstID]);
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
        // swap(p_pointContraction, p_points[m_worstID]);
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

#ifdef AT_DEBUG
    ++m_iIteration;
#endif

  } while (volume() > m_error);

  m_step = steps::finalization;
  // std::memcpy(p_bestPoint, p_points[m_bestID], m_dim * sizeof(double));
  // std::copy(p_points[m_bestID].begin(), p_points[m_bestID].end(),
  // p_bestPoint.begin());

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
        // swap(p_points[i], p_points[j]);
        // swap(p_costs[i], p_costs[j]);
        std::swap(p_points[i], p_points[j]);
        std::swap(p_costs[i], p_costs[j]);
      }
    }
  }
}

void NelderMead::calculate_centroid() {
  // Calculate p_centroid
  for (unsigned j = 0; j < m_dim; ++j) {
    double result = 0.0;
    p_centroid[j] = 0.0; // Centroid = {0}
    for (unsigned i = 0; i < m_nPoints; ++i) {
      // The worst solution is not necessary
      if (i != m_worstID) {
        p_centroid[j] += p_points[i][j];
        result += 1.0;
      }
    }
    p_centroid[j] /= (result);
  }
}

inline void NelderMead::calculate_point(double *&p_out, double _const,
                                        double *p_in1, double *p_in2) {
  for (unsigned j = 0; j < m_dim; ++j) {
    p_out[j] = circ_modf(p_in2[j] + _const * (p_in1[j] - p_in2[j]));
  }
}

void NelderMead::reset(unsigned level) {
  KMP_ASSERT2((level >= 0 && level <= 2),
              "Invalid Nelder Mead reset level value, set 0 <= level <= 2.");

  m_costReflection = 0.0;  // Cost of reflected point
  m_costExpansion = 0.0;   // Cost of expanded point
  m_costContraction = 0.0; // Cost of contracted point

  m_iPoint = m_bestID + 1;
  m_iCost = NO_SAVE;
  m_step = steps::init;

  sort_points();
  // Reset point but keep the best solution
  for (unsigned i = 0; i < m_nPoints; ++i) {
    if (i != m_bestID) {
      for (unsigned j = 0; j < m_dim; ++j) {
        p_points[i][j] = rand_fgen(&buffer); // numbers between -1 and 1.
      }
    }
  }

  if (level < 1)
    return;

  m_iPoint = m_bestID;
  for (unsigned j = 0; j < m_dim; ++j) {
    p_points[m_bestID][j] = rand_fgen(&buffer); // numbers between -1 and 1.
  }
}

double NelderMead::volume() {
  double total = 0.0;
  for (unsigned i = 0; i < m_nPoints; ++i) {
    double value = 0.0;
    for (unsigned j = 0; j < m_dim; ++j) {
      value += pow(p_points[i][j] - p_centroid[j], 2.0);
    }
    value = sqrt(value);     // With this calculation, 'value' is the norm.
    value = pow(value, 2.0); // value is equal to norm²
    total += value;
  }
  return sqrt(total / m_nPoints);
}

// void NelderMead::swap(double *&p1, double *&p2) {
//   double *temp = p1;
//   p1 = p2;
//   p2 = temp;
// }

// void NelderMead::swap(double &p1, double &p2) {
//   double temp = p1;
//   p1 = p2;
//   p2 = temp;
// }