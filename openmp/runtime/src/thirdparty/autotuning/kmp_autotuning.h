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

#include "../../kmp.h"
#include "Autotuning.h"
#include <map>

struct kmp_autotuning {
  bool initialized = false;
  Autotuning at;
};

extern std::map<ident_t *, kmp_autotuning> __kmp_sched_autotunig_map;

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc);

void __kmp_end_autotuning(int gtid, ident_t *loc);

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {
  auto it = __kmp_sched_autotunig_map.find(loc);
  if (it != __kmp_sched_autotunig_map.end() && it->second.initialized)
    return;

  auto &kmp_at = __kmp_sched_autotunig_map[loc];
  // FIXME: Non optimal approach using __kmpc_single
  if (__kmpc_single(loc, gtid)) {
    T max = ub / static_cast<T>(__kmp_nth);

    KD_TRACE(100, ("__kmp_init_autotuning: Tid %d, loc %s, lb %lld, ub %lld, "
                   "max %lld\n",
                   gtid, loc->psource, lb, ub, max));

    kmp_at.at = new Autotuning(1.0, static_cast<double>(max));
    kmp_at.initialized = true;
    __kmpc_end_single(loc, gtid);
  }
}

template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc) {
  auto &kmp_at = __kmp_sched_autotunig_map[loc];
  if (kmp_at.at->isEnd())
    return kmp_at.at->getPoint<typename traits_t<T>::signed_t>(0);

  __kmpc_barrier(loc, gtid); // FIXME: Try to remove this barrier
  if (__kmpc_single(loc, gtid)) {
    kmp_at.at->start();
    __kmpc_end_single(loc, gtid);
  }
  // __kmpc_barrier(loc, gtid); // FIXME: Try to remove this barrier

  return kmp_at.at->getPoint<typename traits_t<T>::signed_t>(0);
}

#endif // KMP_AUTOTUNING_H