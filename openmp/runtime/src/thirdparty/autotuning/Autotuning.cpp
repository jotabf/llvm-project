#include "Autotuning.h"

#include <cmath> // round
#include <cstring> // memcpy
#include <sstream>
#include <type_traits> // std::is_integral, std::is_floating_point

double error_converter(unsigned error, int min, int max) {
  return (2. * static_cast<double>(error)) / static_cast<double>(max - min);
}

Autotuning::Autotuning(double min, double max, unsigned ignore,
                       NumericalOptimizer *optimizer)
    : Autotuning(&min, &max, ignore, optimizer) {}

Autotuning::Autotuning(double *min, double *max, unsigned ignore,
                       NumericalOptimizer *optimizer)
    : m_iter(0), m_ignore(ignore + 1), m_runtime(0), p_optimizer(optimizer) {

  // p_max = new double[p_optimizer->getDimension()];
  // p_min = new double[p_optimizer->getDimension()];
  // p_point = new double[p_optimizer->getDimension()];
  p_max.resize(p_optimizer->getDimension());
  p_min.resize(p_optimizer->getDimension());
  p_point.resize(p_optimizer->getDimension());

  // std::memcpy(p_max, max, sizeof(double) * p_optimizer->getDimension());
  // std::memcpy(p_min, min, sizeof(double) * p_optimizer->getDimension());
  p_max.assign(max, max + p_optimizer->getDimension());
  p_min.assign(min, min + p_optimizer->getDimension());

#ifdef VERBOSE
  print();
#endif
}

Autotuning::~Autotuning() {
  // delete[] p_max;
  // delete[] p_min;
  // delete[] p_point;
  delete p_optimizer;
}

void Autotuning::start() {
  // End execution
  if (!p_optimizer->isEnd()) {
    if ((m_iter % m_ignore) == 0) {
      p_point = p_optimizer->run(m_runtime);
    }
    ++m_iter;

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

std::string array_to_string(std::vector<double> array) {
  std::stringstream ss;
  for (auto &elem : array)
    ss << elem << " ";
  return ss.str();
}

std::string Autotuning::getInfo() const {
  std::stringstream ss;
  ss << "------------------- Autotuning Parameters -------------------\n";
  ss << "NIgn: " + std::to_string(m_ignore) + "\t";
  ss << "Maxs: [ " << array_to_string(p_max) << "]\n";
  ss << "Mins: [ " << array_to_string(p_min) << "]\n";
  ss << p_optimizer->getInfo();
  ss << "-------------------------------------------------------------\n";
  return ss.str();
}