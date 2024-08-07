#include "Autotuning.hpp"

#include <omp.h>

#include <iostream>  // cout, endl

Autotuning::Autotuning(double min, double max, int ignore, int dim, int num_opt, int max_iter)
    : Autotuning(min, max, ignore, new CSA(dim, num_opt, max_iter)) {}

Autotuning::Autotuning(double min, double max, int ignore, NumericalOptimizer *optimizer)
    : m_min(min),
      m_max(max),
      m_iter(0),
      m_ignore(ignore + 1),
      m_cost(0),
      m_t0(0),
      m_runtime(0),
      p_optimizer(optimizer) {

  if (ignore < 0) {
    throw std::invalid_argument("Ignore Value Invalid! Set _ignore >= 0.");
  }

#ifdef VERBOSE
  print();
#endif
}

Autotuning::~Autotuning() {
  delete p_optimizer;
}

void Autotuning::reset(int level) {
// #ifdef _OPENMP
// #pragma omp single
// #endif
//   {
    m_iter = 0;
    p_optimizer->reset(level);
  // }
}

void Autotuning::end() {
  if (!p_optimizer->isEnd()) {
// #ifdef _OPENMP
// #pragma omp single
// #endif
//     { 
      const auto m_t1 = std::chrono::high_resolution_clock::now();
      m_runtime = std::chrono::duration<double>(m_t1 - m_t0).count();
    // }
  }
}

void Autotuning::print() const {
  std::cout << "------------------- Autotuning Parameters -------------------" << std::endl;
  std::cout << "NIgn: " << m_ignore << "\t";
  std::cout << "Min: " << m_min << "\tMax: " << m_max << std::endl;
  p_optimizer->print();
  std::cout << "-------------------------------------------------------------" << std::endl;
}
