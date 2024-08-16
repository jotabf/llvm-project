#include "Autotuning.h"

#include <cmath>    // round
#include <cstring>  // memcpy
#include <iostream> // cout, endl

double error_converter(unsigned error, int min, int max) {
  return (2. * static_cast<double>(error)) / static_cast<double>(max - min);
}

Autotuning::Autotuning(double min, double max, unsigned ignore,
                       NumericalOptimizer *optimizer)
    : Autotuning(&min, &max, ignore, optimizer) {}

Autotuning::Autotuning(double *min, double *max, unsigned ignore,
                       NumericalOptimizer *optimizer)
    : m_iter(0), m_ignore(ignore + 1), m_runtime(0), p_optimizer(optimizer) {

  p_max = new double[p_optimizer->getDimension()];
  p_min = new double[p_optimizer->getDimension()];
  p_point = new double[p_optimizer->getDimension()];

  std::memcpy(p_max, max, sizeof(double) * p_optimizer->getDimension());
  std::memcpy(p_min, min, sizeof(double) * p_optimizer->getDimension());

#ifdef VERBOSE
  print();
#endif
}

Autotuning::~Autotuning() {
  delete[] p_max;
  delete[] p_min;
  delete[] p_point;
  delete p_optimizer;
}

void Autotuning::start() {
  // End execution
  if (!p_optimizer->isEnd()) {
    if ((m_iter % m_ignore) == 0) {
      p_point = p_optimizer->run(m_runtime);
    }
    m_iter++;

    m_t0 = std::chrono::high_resolution_clock::now();
  }
}

void Autotuning::reset(unsigned level) {
  m_iter = 0;
  p_optimizer->reset(level);
}

void Autotuning::end() {
  if (!p_optimizer->isEnd()) {
    const auto m_t1 = std::chrono::high_resolution_clock::now();
    m_runtime = std::chrono::duration<double>(m_t1 - m_t0).count();
  }
}

void Autotuning::print() const {
  std::cout << "------------------- Autotuning Parameters -------------------"
            << std::endl;
  std::cout << "NIgn: " << m_ignore << "\t";
  std::cout << "Maxs: [ ";
  for (size_t i = 0; i < p_optimizer->getDimension(); i++) {
    std::cout << p_max[i] << " ";
  }
  std::cout << "]\n";
  std::cout << "Mins: [ ";
  for (size_t i = 0; i < p_optimizer->getDimension(); i++) {
    std::cout << p_min[i] << " ";
  }
  std::cout << "]\n";
  p_optimizer->print();
  std::cout << "-------------------------------------------------------------"
            << std::endl;
}
