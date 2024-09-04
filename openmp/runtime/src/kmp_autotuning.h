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
#include <ctime> // time_t

class Autotuning;

extern __attribute__((weak)) const unsigned __KMP_NUM_AUTO_MODE;

struct kmp_autotuning_info {
  KMP_ALIGN_CACHE
  volatile bool initialized = FALSE;
  volatile bool release_start = FALSE;
  KMP_ALIGN_CACHE
  std::atomic<int> count = 0;
  kmp_lock_t lock;
  Autotuning *at = NULL;
};

template <typename T>
void __kmp_init_autotuning(int gtid, unsigned id, T lb, T ub);

template <typename T>
T __kmp_start_autotuning(int gtid, unsigned id, T lb, T ub);

void __kmp_autotuning_global_initialize();

void __kmp_end_autotuning(int gtid, unsigned id);

kmp_autotuning_info *__kmp_find_autotuning_info(unsigned id);

///@brief Class for Autotuning
class Autotuning {

  // long long m_min; ///< Minimum value of the search interval
  // long long m_max; ///< Maximum value of the search interval

  int64_t *p_point;  ///< Point in the search space
  unsigned m_ignore; ///< Number of iterations to ignore
  unsigned m_iter;   ///< Iteration number

  NelderMead *p_optimizer; ///< Numerical optimizer instance

  clock_t m_t0;     ///< Starting time
  double m_runtime; ///< Total time of a task

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
  static Autotuning *Create(int64_t min, int64_t max, unsigned ignore = 0);

  ///@brief Destructor
  static void Destroy(Autotuning *at) { __kmp_free(at); }

  ///@brief Get the point in the search space
  ///@tparam T Type of the output point
  ///@param i Index of the point
  ///@return The point in the search space
  int64_t getPoint(int i = 0) const {
    KMP_ASSERT(p_point != NULL);
    return p_point[0];
  }

  ///@brief Start a new iteration of the autotuning algorithm
  ///@param point Input/output array of tuning parameters
  void start();

  ///@brief End the current iteration of the autotuning algorithm
  void end();

  ///@brief Check if the optimization has reached the end
  bool isEnd() const { return p_optimizer->isEnd(); }

  ///@brief Set the limits of the search interval
  ///@param min Minimum value of the search interval
  ///@param max Maximum value of the search interval
  void setLimits(int64_t min, int64_t max) { p_optimizer->setLimits(min, max); }

  ///@brief Reset the autotuning and numerical optimizer
  ///@param level Reset level, depending on the Optimizer
  void reset(unsigned level);
};

template <typename T>
void __kmp_init_autotuning(int gtid, unsigned id, T lb, T ub) {

  __kmp_autotuning_global_initialize();

  auto info = __kmp_find_autotuning_info(id);

  KMP_ASSERT2(info != NULL, "Sched Autotuning info was not initialized");

  if (TCR_4(info->initialized))
    return;
  __kmp_acquire_bootstrap_lock(&__kmp_initz_lock);
  if (TCR_4(info->initialized)) {
    __kmp_release_bootstrap_lock(&__kmp_initz_lock);
    return;
  }

  int64_t min = static_cast<int64_t>(lb + 1);
  int64_t max = static_cast<int64_t>((ub + 1) / static_cast<T>(__kmp_nth));

  info->at = Autotuning::Create(min, max);

  KMP_MB(); // Flush initialized
  TCW_SYNC_4(info->initialized, TRUE);

  __kmp_release_bootstrap_lock(&__kmp_initz_lock);
}

// TO DO: TEST IF ALL THREADS ARE RETURNING THE SAME VALUE
template <typename T>
T __kmp_start_autotuning(int gtid, unsigned id, T lb, T ub) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(id);

  KMP_ASSERT2(info != NULL, "Sched Autotuning info was not initialized");
  KMP_ASSERT2(info->at != NULL, "Autotuning was not initialized");

  if (info->at->isEnd())
    return info->at->getPoint();

  if (TCR_4(info->release_start))
    return info->at->getPoint();
  __kmp_acquire_bootstrap_lock(&info->lock);
  if (TCR_4(info->release_start)) {
    __kmp_release_bootstrap_lock(&info->lock);
    return info->at->getPoint();
  }

  int64_t min = static_cast<int64_t>(lb + 1);
  int64_t max = static_cast<int64_t>((ub + 1) / static_cast<T>(__kmp_nth));

  info->at->setLimits(min, max);

  info->at->start();
  TCW_SYNC_4(info->release_start, TRUE);
  KMP_MB();

  __kmp_release_bootstrap_lock(&info->lock);

  return info->at->getPoint();
}

#endif // KMP_AUTOTUNING_H