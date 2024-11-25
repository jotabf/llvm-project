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

// extern __attribute__((weak)) const unsigned __KMP_NUM_AUTO_MODE;
extern int64_t __kmp_end_max[256];

struct kmp_autotuning_info {
  KMP_ALIGN_CACHE
  volatile int initialized = FALSE;
  volatile int started = FALSE;
  volatile kmp_uint32 ended = TRUE;
  KMP_ALIGN_CACHE
  kmp_lock_t start_lock;
  kmp_lock_t end_lock;
  std::atomic<int> count = 0;
  Autotuning *at = NULL;
};

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T>
T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub);

void __kmp_autotuning_global_initialize();

void __kmp_end_autotuning(int gtid, ident_t *loc);

kmp_autotuning_info *__kmp_find_autotuning_info(ident_t *loc, int64_t max);

kmp_autotuning_info *__kmp_create_autotuning_info(ident_t *loc, int64_t max);

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
  ///@param i Index of the point
  ///@return The point in the search space
  int64_t getPoint(int i = 0) const {
    KMP_ASSERT(p_point != NULL);
    return p_point[0];
  }

  unsigned getIter() const { return m_iter; }

  ///@brief Start a new iteration of the autotuning algorithm
  ///@param point Input/output array of tuning parameters
  void start();

  ///@brief End the current iteration of the autotuning algorithm
  void end();

  ///@brief Check if the optimization has reached the end
  bool isEnd() const { return p_optimizer->isEnd(); }

  ///@brief Set the point in the search space
  ///@param v Value of the point
  ///@param id Index of the point
  ///@param dim Dimension index of the point
  void setPoint(int64_t v, unsigned id, unsigned dim = 0) {
    p_optimizer->setPoint(v, id, dim);
  }

  ///@brief Set the limits of the search interval
  ///@param min Minimum value of the search interval
  ///@param max Maximum value of the search interval
  void setLimits(int64_t min, int64_t max) { p_optimizer->setLimits(min, max); }

  ///@brief Reset the autotuning and numerical optimizer
  ///@param level Reset level, depending on the Optimizer
  void reset(unsigned level);
};

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {

  __kmp_autotuning_global_initialize();

  kmp_autotuning_info *info =
      __kmp_find_autotuning_info(loc, static_cast<int64_t>(ub));
  if (info == NULL) {
    __kmp_barrier(bs_plain_barrier, gtid, FALSE, 0, NULL, NULL);
    if (__kmpc_single(loc, gtid)) {
      __kmp_create_autotuning_info(loc, static_cast<int64_t>(ub));
      __kmpc_end_single(loc, gtid);
    }
    __kmp_barrier(bs_plain_barrier, gtid, FALSE, 0, NULL, NULL);
    info = __kmp_find_autotuning_info(loc, static_cast<int64_t>(ub));
  }

  KMP_ASSERT2(info != NULL, "Sched Autotuning info was not initialized");

  if (TCR_4(info->initialized))
    return;
  __kmp_acquire_bootstrap_lock(&info->start_lock);
  if (TCR_4(info->initialized)) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return;
  }

  int nth = TCR_4(__kmp_nth);
  int64_t min = static_cast<int64_t>(lb);
  int64_t max = static_cast<int64_t>((ub) / static_cast<T>(nth * 2));

  if (min >= max) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return;
  }

  info->at = Autotuning::Create(min, max);

  int64_t ninter = static_cast<int64_t>(ub - lb);
  double factor = log2(static_cast<double>(ninter) / nth) * (1.0 / 1.618);
  int64_t point = static_cast<int64_t>(ninter / (pow(2.0, factor) * 2.0 * nth));
  if (point < min)
    point = min;
  if (point > max)
    point = max;

  info->at->setPoint(point, 0);

  printf("__kmp_init_autotuning: %s min=%li max=%li point=%li\n", loc->psource,
         min, max, point);

  TCW_SYNC_4(info->started, FALSE);

  TCW_SYNC_4(info->initialized, TRUE);
  KMP_MB(); // Flush initialized

  __kmp_release_bootstrap_lock(&info->start_lock);
}

// TO DO: TEST IF ALL THREADS ARE RETURNING THE SAME VALUE
template <typename T>
T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub) {

  __kmp_init_autotuning(gtid, loc, lb, ub);

  kmp_autotuning_info *info =
      __kmp_find_autotuning_info(loc, static_cast<int64_t>(ub));

  // KMP_ASSERT(id > 0);
  KMP_DEBUG_ASSERT2(info != NULL, "Sched Autotuning info was not initialized");

  if (!TCR_4(info->initialized))
    return 1;

  if (info->at->isEnd())
    return info->at->getPoint();

  KMP_ASSERT2(info->at != NULL, "Autotuning was not initialized");

  __kmp_barrier(bs_plain_barrier, gtid, FALSE, 0, NULL, NULL);

  if (TCR_4(info->started))
    return info->at->getPoint();
  __kmp_acquire_bootstrap_lock(&info->start_lock);
  if (TCR_4(info->started)) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return info->at->getPoint();
  }

  // KMP_WAIT(&info->ended, TRUE, KMP_EQ, NULL);

  __kmp_end_max[gtid] = static_cast<int64_t>(ub);

  int64_t min = static_cast<int64_t>(lb);
  int64_t max =
      static_cast<int64_t>((ub) / static_cast<T>(TCR_4(__kmp_nth) * 2));

  info->at->setLimits(min, max);
  info->at->start();

  TCW_SYNC_4(info->started, TRUE);
  TCW_SYNC_4(info->ended, FALSE);
  KMP_MB();

  __kmp_release_bootstrap_lock(&info->start_lock);

  if (info->at->isEnd())
    printf("__kmp_start_autotuning: %s final chunk %li min=%li max=%li\n",
           loc->psource, info->at->getPoint(), min, max);

  return info->at->getPoint();
}

#endif // KMP_AUTOTUNING_H