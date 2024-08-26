/**
 * @file Autotuning.hpp
 * @brief Header file for the Autotuning class
 */

#pragma once

#include <cmath> // round
#include <chrono> // std::chrono::high_resolution_clock
#include <string> // std::string
#include <vector> // std::vector

#include "NumericalOptimizer.h"
#include "NelderMead.h"

/**
 * @brief Class for Autotuning
 */
class Autotuning {

  unsigned m_iter; ///< Iteration number
  const unsigned m_ignore; ///< Number of iterations to ignore

  // double *p_point;
  // double *p_min; ///< Minimum value of the search interval
  // double *p_max; ///< Maximum value of the search interval

  std::vector<double> p_point;
  std::vector<double> p_min; ///< Minimum value of the search interval
  std::vector<double> p_max; ///< Maximum value of the search interval

  std::chrono::high_resolution_clock::time_point m_t0; ///< Starting time
  double m_runtime; ///< Total time of a task

  NumericalOptimizer *const p_optimizer; ///< Numerical optimizer instance

  /**
   * @brief Rescale point from Type [-1,1] to Floating [min,max]
   * @tparam T Type of the output point
   * @param out Output point
   */
  template <typename T> void rescale(const T *&out) const {
    for (unsigned i = 0; i < p_optimizer->getDimension(); i++) {
      out[i] = rescale<T>(i);
    }
  }

  template <typename T> T rescale(unsigned i) const {
    static_assert(std::is_integral<T>::value ||
                      std::is_floating_point<T>::value,
                  "T must be either an integer or a floating point type");

    if constexpr (std::is_floating_point<T>::value) {
      return static_cast<T>((p_point[i] + 1.0) / 2.0 * (p_max[i] - p_min[i]) +
                            p_min[i]);
    } else {
      return static_cast<T>(
          round(((p_point[i] + 1.0) / 2.0) * (p_max[i] - p_min[i]) + p_min[i]));
    }
  }

public:
  // Deleted constructors and assignment operators to prevent copying and moving
  auto operator=(Autotuning &&) -> Autotuning & = delete;
  auto operator=(Autotuning) -> Autotuning = delete;
  Autotuning(const Autotuning &) = delete;
  Autotuning(Autotuning &&) = delete;
  Autotuning() = delete;

  /**
   * @brief Parameterized constructor with CSA as the default optimizer
   * @param dim Cost Function Dimension
   * @param min Minimum value of the search interval
   * @param max Maximum value of the search interval
   * @param ignore Number of iterations to ignore
   * @param num_opt Number of optimizers
   * @param max_iter Maximum number of iterations
   */
  Autotuning(double min, double max, unsigned ignore = 0,
             NumericalOptimizer *optimizer = new NelderMead(1, 0.0222222));

  /**
   * @brief Parameterized constructor with a custom optimizer
   * @param min Minimum value of the search interval
   * @param max Maximum value of the search interval
   * @param ignore Number of iterations to ignore
   * @param optimizer Numerical optimizer instance
   */
  Autotuning(double *min, double *max, unsigned ignore,
             NumericalOptimizer *optimizer);

  /**
   * @brief Destructor
   */
  ~Autotuning();

  /**
   * @brief Get the point in the search space
   * @tparam T Type of the output point
   * @param i Index of the point
   * @return The point in the search space
   */
  template <typename T> T getPoint(unsigned i) const { return rescale<T>(i); }

  /**
   * @brief Get the point in the search space
   * @tparam T Type of the output point
   * @param out Output point
   */
  // template <typename T> void getPoint(T *out) const { rescale<T>(out); }

  /**
   * @brief Start a new iteration of the autotuning algorithm
   * @param point Input/output array of tuning parameters
   */
  void start();

  /**
   * @brief End the current iteration of the autotuning algorithm
   */
  void end();

  bool isEnd() const { return p_optimizer->isEnd(); }

  /**
   * @brief
   */
  std::string getInfo() const;

  /**
   * @brief Reset the autotuning and numerical optimizer
   * @param level Reset level:
   *   - level 2: Reset the number of iterations
   *   - level 1: Reset the points and the temperatures (plus the previous ones)
   *   - level 0: Remove the best solution (plus the previous ones)
   */
  void reset(unsigned level);
};