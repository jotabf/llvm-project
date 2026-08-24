// Shim mínimo de "kmp.h" para compilar NelderMead.cpp fora da libomp.
// Só precisa fornecer __kmp_allocate / __kmp_free.
// IMPORTANTE: ___kmp_allocate_align da libomp zera o bloco alinhado
// (kmp_alloc.cpp: "Aligned block is filled with zeros"), por isso calloc
// e não malloc -- vários campos do NelderMead dependem disso.
#ifndef KMP_SHIM_KMP_H
#define KMP_SHIM_KMP_H

#include <cstdint>
#include <cstdio>
#include <cstdlib>

static inline void *__kmp_allocate(std::size_t size) {
  void *p = std::calloc(1, size);
  if (!p) {
    std::fprintf(stderr, "shim __kmp_allocate: out of memory (%zu bytes)\n",
                 size);
    std::abort();
  }
  return p;
}

static inline void __kmp_free(void *p) { std::free(p); }

#endif // KMP_SHIM_KMP_H
