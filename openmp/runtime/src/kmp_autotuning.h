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
#include "thirdparty/autotuning/Autotuning.h"
#include <map>
#include <atomic>

struct kmp_autotuning {
  bool initialized = false;
  bool release_start = false;
  std::atomic<int> count;
  kmp_lock_t lock;
  Autotuning *at = nullptr;
};

extern std::map<ident_t *, kmp_autotuning> __kmp_sched_autotunig_map;

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc);

void __kmp_end_autotuning(int gtid, ident_t *loc);

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {

  auto end_it = __kmp_sched_autotunig_map.end();
  auto it = __kmp_sched_autotunig_map.find(loc);
  if (it != end_it && it->second.initialized)
    return;

  if (__kmpc_single(loc, gtid)) {
    T max = ub / static_cast<T>(__kmp_nth);

    KD_TRACE(100, ("__kmp_init_autotuning: Tid %d, loc %s, lb %lld, ub %lld, "
                   "max %lld\n",
                   gtid, loc->psource, lb, ub, max));

    // Create new key and initialize autotuning
    auto &kmp_at = __kmp_sched_autotunig_map[loc];
    kmp_at.at = new Autotuning(1.0, static_cast<double>(max));
    KMP_SFENCE();

    kmp_at.initialized = true;
    __kmpc_end_single(loc, gtid);
  }

  // Wait new key be create
  it = __kmp_sched_autotunig_map.find(loc);
  while (it == end_it){
    KMP_YIELD(TRUE);
    it = __kmp_sched_autotunig_map.find(loc);
  }
    
  // Wait autotuning be initialized
  while (!it->second.initialized)
    KMP_YIELD(TRUE);
}


// TO DO: TEST IF ALL THREADS ARE RETURNING THE SAME VALUE
template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc) {
  auto &kmp_at = __kmp_sched_autotunig_map[loc];

  KMP_ASSERT2(kmp_at.at != nullptr, "Autotuning is not initialized");

  if (kmp_at.at->isEnd())
    return kmp_at.at->getPoint<typename traits_t<T>::signed_t>(0);
  
  if (__kmpc_single(loc, gtid)) {
    kmp_at.at->start();
    KMP_MFENCE();
    kmp_at.release_start = true;
    __kmpc_end_single(loc, gtid);
  }

  KMP_SFENCE();
  while (!kmp_at.release_start)
    KMP_YIELD(TRUE);

  return kmp_at.at->getPoint<typename traits_t<T>::signed_t>(0);
}

#endif // KMP_AUTOTUNING_H