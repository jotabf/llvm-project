#include "kmp_autotuning.h"

std::map<ident_t *, kmp_autotuning> __kmp_sched_autotunig_map;

void __kmp_end_autotuning(int gtid, ident_t *loc) {
  auto it = __kmp_sched_autotunig_map.find(loc);
  if (it == __kmp_sched_autotunig_map.end() || it->second.at->isEnd())
    return;

  __kmpc_barrier(loc, gtid);
  if (__kmpc_single(loc, gtid)) {
    it->second.at->end();
    __kmpc_end_single(loc, gtid);
  }
}