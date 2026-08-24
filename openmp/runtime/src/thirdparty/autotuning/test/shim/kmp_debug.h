// Shim mínimo de "kmp_debug.h".
#ifndef KMP_SHIM_KMP_DEBUG_H
#define KMP_SHIM_KMP_DEBUG_H

#include <cstdio>
#include <cstdlib>

// Contador global de asserts disparados, para o harness poder relatar em vez
// de simplesmente abortar quando NM_SHIM_ASSERT_FATAL=0.
extern int nm_shim_assert_count;

#ifndef NM_SHIM_ASSERT_FATAL
#define NM_SHIM_ASSERT_FATAL 0
#endif

#define KMP_ASSERT2(cond, msg)                                                 \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ++nm_shim_assert_count;                                                  \
      std::fprintf(stderr, "  [KMP_ASSERT2] %s  (%s:%d)\n", (msg), __FILE__,   \
                   __LINE__);                                                  \
      if (NM_SHIM_ASSERT_FATAL)                                                \
        std::abort();                                                          \
    }                                                                          \
  } while (0)

#define KMP_ASSERT(cond) KMP_ASSERT2(cond, #cond)

#endif // KMP_SHIM_KMP_DEBUG_H
