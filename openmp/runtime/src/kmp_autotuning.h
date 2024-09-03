//===-- kmp_autotuning.h - Scheduling autotuning mode management -*- C++-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the declaration of autotuning functions that are used in
/// scheduling runtime to auto adjust the chunk size when selected the auto
/// mode.
///
//===----------------------------------------------------------------------===//

#ifndef KMP_AUTOTUNING_H
#define KMP_AUTOTUNING_H

#include "kmp.h"
#include "thirdparty/autotuning/NelderMead.h"
#include "thirdparty/autotuning/NumericalOptimizer.h"

#include <atomic>
#include <chrono> // std::chrono::high_resolution_clock
#include <cmath>  // round
#include <ctime>  // time_t
#include <map>

class Autotuning;

struct kmp_autotuning_info {
  KMP_ALIGN_CACHE
  volatile bool initialized = FALSE;
  volatile bool release_start = FALSE;
  KMP_ALIGN_CACHE
  std::atomic<int> count = 0;
  kmp_lock_t lock;
  Autotuning *at = NULL;
};

extern const unsigned __KMP_NUM_AUTO_MODE;

void __kmp_init_autotuning(int gtid, unsigned id);

template <typename T>
T __kmp_start_autotuning(int gtid, unsigned id, T lb, T ub);

void __kmp_autotuning_global_initialize();

void __kmp_end_autotuning(int gtid, unsigned id);

kmp_autotuning_info *__kmp_find_autotuning_info(unsigned id);

///@brief Class for Autotuning
class Autotuning {

  long long m_min; ///< Minimum value of the search interval
  long long m_max; ///< Maximum value of the search interval

  double p_point;    ///< Point in the search space
  unsigned m_ignore; ///< Number of iterations to ignore
  unsigned m_iter;   ///< Iteration number

  clock_t m_t0;     ///< Starting time
  double m_runtime; ///< Total time of a task

public:
  NelderMead *p_optimizer; ///< Numerical optimizer instance

  template <typename T> T rescale(T min, T max) const {
    static_assert(std::is_integral<T>::value ||
                      std::is_floating_point<T>::value,
                  "T must be either an integer or a floating point type");
    const double opt_min = -1;
    const double opt_max = 1;
    if constexpr (std::is_floating_point<T>::value) {
      return static_cast<T>(
          (p_point - opt_min) / (opt_max - opt_min) * (max - min) + min);
    } else {
      return static_cast<T>(
          round((p_point - opt_min) / (opt_max - opt_min) * (max - min) + min));
    }
  }

public:
  // Deleted constructors and assignment operators to prevent copying and moving
  auto operator=(Autotuning &&) -> Autotuning & = delete;
  auto operator=(Autotuning) -> Autotuning = delete;
  Autotuning(const Autotuning &) = delete;
  Autotuning(Autotuning &&) = delete;
  Autotuning() = delete;
  ~Autotuning() = delete;

  ///@brief Parameterized constructor
  ///@param dim Cost Function Dimension
  ///@param min Minimum value of the search interval
  ///@param max Maximum value of the search interval
  ///@param ignore Number of iterations to ignore
  ///@param num_opt Number of optimizers
  ///@param max_iter Maximum number of iterations
  static Autotuning *Create(int min, int max, unsigned ignore = 0);

  ///@brief Destructor
  static void Destroy(Autotuning *at) { __kmp_free(at); }

  ///@brief Get the point in the search space
  ///@tparam T Type of the output point
  ///@param i Index of the point
  ///@return The point in the search space
  template <typename T> T getPoint(T min, T max) const {
    return rescale<T>(min, max);
  }

  ///@brief Start a new iteration of the autotuning algorithm
  ///@param point Input/output array of tuning parameters
  void start();

  ///@brief End the current iteration of the autotuning algorithm
  void end();

  ///@brief Check if the optimization has reached the end
  bool isEnd() const { return p_optimizer->isEnd(); }

  ///@brief Reset the autotuning and numerical optimizer
  ///@param level Reset level, depending on the Optimizer
  void reset(unsigned level);
};

// TO DO: TEST IF ALL THREADS ARE RETURNING THE SAME VALUE
template <typename T>
T __kmp_start_autotuning(int gtid, unsigned id, T lb, T ub) {
  printf("Starting autotuning %d\n", id);

  kmp_autotuning_info *info = __kmp_find_autotuning_info(id);

  KMP_ASSERT2(info != NULL, "Sched Autotuning info was not initialized");
  KMP_ASSERT2(info->at != NULL, "Autotuning was not initialized");

  const T min = lb + 1;
  const T max = (ub + 1) / static_cast<T>(__kmp_nth);

  if (info->at->isEnd())
    return info->at->getPoint<T>(min, max);

  if (TCR_4(info->release_start))
    return info->at->getPoint<T>(min, max);
  __kmp_acquire_bootstrap_lock(&info->lock);
  if (TCR_4(info->release_start)) {
    __kmp_release_bootstrap_lock(&info->lock);
    return info->at->getPoint<T>(min, max);
  }

  info->at->start();
  TCW_SYNC_4(info->release_start, TRUE);
  KMP_MB();

  __kmp_release_bootstrap_lock(&info->lock);

  return info->at->getPoint<T>(min, max);
}

#endif // KMP_AUTOTUNING_H