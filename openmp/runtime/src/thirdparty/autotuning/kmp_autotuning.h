#include "../../kmp.h"
#include "Autotuning.h"
#include <map>

typedef struct kmp_autotuning {
  bool initialized = false;
  Autotuning *at = nullptr;
} kmp_autotuning_t;

extern std::map<ident_t *, kmp_autotuning> __kmp_sched_autotunig_map;

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc);

void __kmp_end_autotuning(int gtid, ident_t *loc);

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {
  auto it = __kmp_sched_autotunig_map.find(loc);
  if (it == __kmp_sched_autotunig_map.end() || !it->second.initialized) {
    // FIXME: Non optimal approach using __kmpc_single
    if (__kmpc_single(loc, gtid)) {
      printf("Initializing autotuning for %d and loc %s\n", gtid, loc->psource);
      T max = ub / static_cast<T>(__kmp_nth);

      __kmp_sched_autotunig_map[loc].at =
          new Autotuning(1.0, static_cast<double>(max));
      __kmpc_end_single(loc, gtid);
    }
    __kmpc_barrier(loc, gtid); // FIXME: After the initialization, thread don't
                               // need to wait for the other threads
    __kmp_sched_autotunig_map[loc].initialized = true;
  }
}

template <typename T>
typename traits_t<T>::signed_t __kmp_start_autotuning(int gtid, ident_t *loc) {

  __kmpc_barrier(loc, gtid); // FIXME: Try to remove this barrier
  if (__kmpc_single(loc, gtid)) {
    __kmp_sched_autotunig_map[loc].at->start();
    __kmpc_end_single(loc, gtid);
    printf("Started autotuning for %d and loc %s pointer %p and chunk %llu\n",
           gtid, loc->psource, __kmp_sched_autotunig_map[loc].at,
           __kmp_sched_autotunig_map[loc].at->getPoint<T>(0));
  }
  __kmpc_barrier(loc, gtid); // FIXME: Try to remove this barrier

  return __kmp_sched_autotunig_map[loc].at->getPoint<T>(0);
}