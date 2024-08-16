#include "../../kmp.h"
#include "Autotuning.h"
#include <map>

extern std::map<ident_t *, Autotuning *> __kmp_sched_autotunig_map;

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc);

void __kmp_end_autotuning(int gtid, ident_t *loc);

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {
  auto it = __kmp_sched_autotunig_map.find(loc);
  if (it == __kmp_sched_autotunig_map.end()) {
    printf("Initializing autotuning for %d and loc %s\n", gtid, loc->psource);
    
    T max = ub / static_cast<T>(__kmp_nth);
    __kmp_sched_autotunig_map[loc] =
        new Autotuning(1.0, static_cast<double>(max));
  }
}

template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc) {
  printf("Starting autotuning for %d and loc %s\n", gtid, loc->psource);

  __kmp_sched_autotunig_map[loc]->start();

  return __kmp_sched_autotunig_map[loc]->getPoint<T>(0);
}