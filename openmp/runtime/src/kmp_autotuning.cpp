#include "kmp_autotuning.h"

std::map<ident_t *, kmp_autotuning> __kmp_sched_autotunig_map;

void __kmp_end_autotuning(int gtid, ident_t *loc) {

  //TODO: Remove this find
  auto it = __kmp_sched_autotunig_map.find(loc);
  if (it == __kmp_sched_autotunig_map.end() || it->second.at->isEnd())
    return;

  auto &kmp_at = it->second;
  // Only the last thread will end the autotuning.
  KMP_ATOMIC_ADD(&kmp_at.count, 1);
  if (KMP_ATOMIC_LD_ACQ(&kmp_at.count) != __kmp_nth) 
    return;

  // if (__kmpc_single(loc, gtid)) {
  kmp_at.at->end();
  KMP_ATOMIC_ST_REL(&kmp_at.count, 0);
  kmp_at.release_start = false;
  // __kmpc_end_single(loc, gtid);
  // }
}