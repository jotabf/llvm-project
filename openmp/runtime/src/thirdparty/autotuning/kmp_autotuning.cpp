#include "kmp_autotuning.h"

std::map<ident_t *, Autotuning *> __kmp_sched_autotunig_map;

void __kmp_end_autotuning(int gtid, ident_t *loc) {
  printf("Ending autotuning for %d and loc %s\n", gtid, loc->psource);

  __kmp_sched_autotunig_map[loc]->end();
}