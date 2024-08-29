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
  volatile bool initialized = FALSE;
  volatile bool release_start = FALSE;
  ident_t *loc;
  std::atomic<int> count;
  kmp_lock_t lock;
  Autotuning *at = NULL;
  kmp_autotuning_info *next = NULL;
};

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T>
T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub);

void __kmp_end_autotuning(int gtid, ident_t *loc);

kmp_autotuning_info *__kmp_find_autotuning_info(ident_t *loc);

kmp_autotuning_info *__kmp_create_autotuning_info(ident_t *loc);

///@brief Class for Autotuning
class Autotuning {

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
    if constexpr (std::is_floating_point<T>::value) {
      return static_cast<T>((p_point + 1.0) / 2.0 * (max - min) + min);
    } else {
      return static_cast<T>(round(((p_point + 1.0) / 2.0) * (max - min) + min));
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
  static Autotuning *Create(double min, double max, unsigned ignore = 0);

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

  // double getMin() const { return p_min; }

  // double getMax() const { return p_max; }

  // void setMin(double min) { p_min = min; }

  // void setMax(double max) { p_max = max; }
};

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {

  printf("Initializing autotuning %d %d %p %s\n", lb, ub, loc, loc->psource);
  
  T max = (ub + 1) / static_cast<T>(__kmp_nth);

  auto it = __kmp_find_autotuning_info(loc);
  if (it != NULL && TCR_4(it->initialized))
    return;
  __kmp_acquire_bootstrap_lock(&__kmp_initz_lock);
  if (it != NULL && TCR_4(it->initialized)) {
    __kmp_release_bootstrap_lock(&__kmp_initz_lock);
    return;
  }

  auto *info = __kmp_create_autotuning_info(loc);
  TCW_SYNC_4(info->initialized, TRUE);

  KMP_DEBUG_ASSERT2(__kmp_find_autotuning_info(loc) != NULL,
                    "Error creating autotuning info\n");
  KMP_MB(); // Flush initialized

  __kmp_release_bootstrap_lock(&__kmp_initz_lock);
}

// TO DO: TEST IF ALL THREADS ARE RETURNING THE SAME VALUE
template <typename T>
T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc);

  KMP_ASSERT2(info != NULL, "Sched Autotuning info was not initialized");
  KMP_ASSERT2(info->at != NULL, "Autotuning was not initialized");

  const T min = lb + 1;
  const T max = (ub + 1) / static_cast<T>(__kmp_nth);

  if (info->at->isEnd())
    return info->at->getPoint<T>(min, max);

  if(TCR_4(info->release_start))
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

  // if (info->at->isEnd())
  //   return info->at->getPoint<T>(min, max);

  // __kmpc_barrier(loc, gtid);
  // if (__kmpc_single(loc, gtid)) {
  //   info->at->start();
  //   KMP_MB();
  //   TCW_SYNC_4(info->release_start, TRUE);
  //   __kmpc_end_single(loc, gtid);
  // }
  // __kmpc_barrier(loc, gtid);

  // // KMP_MB();
  // // while (!TCR_4(info->release_start))
  // //   KMP_YIELD(TRUE);

  // return info->at->getPoint<T>(min, max);

  // auto &kmp_at = __kmp_sched_autotunig_map[loc];
  // KMP_ASSERT2(kmp_at.at != nullptr, "Autotuning is not initialized");

  // // if (kmp_at.at->isEnd())
  // // return kmp_at.at->getPoint<typename traits_t<T>::signed_t>();
  // if (Autotuning_IsEnd(kmp_at.at))
  //   return Autotuning_GetPoint(kmp_at.at, 0);

  // if (__kmpc_single(loc, gtid)) {
  //   // kmp_at.at->start();
  //   Autotuning_Start(kmp_at.at);
  //   KMP_MFENCE();
  //   TCW_SYNC_4(kmp_at.release_start, TRUE);
  //   __kmpc_end_single(loc, gtid);
  // }

  // KMP_SFENCE();
  // while (!kmp_at.release_start)
  //   KMP_YIELD(TRUE);

  // // return kmp_at.at->getPoint<typename traits_t<T>::signed_t>();
  // return Autotuning_GetPoint(kmp_at.at, 0);
}

#endif // KMP_AUTOTUNING_H