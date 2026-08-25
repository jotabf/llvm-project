// RUN: %libomp-compile
// RUN: env KMP_AT_FORCE=1 KMP_AUTOTUNING_QUIET=1 %libomp-run | FileCheck %s
// RUN: env KMP_AUTOTUNING_QUIET=1 %libomp-run | FileCheck --check-prefix=OFF %s
//
// Testa KMP_AT_FORCE: autotunar um loop schedule(dynamic) que NAO foi anotado
// com "auto". E o caminho do flang e do MLIR, que passam pelo OMPIRBuilder e
// nao tem a clausula do clang.
//
// A diferenca em relacao a dispatch_auto_abi.c e deliberada e tripla:
//
//  1. O schedule vai SEM o bit kmp_sch_chunk_mode_auto -- exatamente como o
//     OMPIRBuilder o emite.
//  2. O binario NAO define __KMP_NUM_AUTO_MODE, porque o flang nao a emite.
//     Isso exercita a excecao no guard do simbolo fraco: sem ela o modo
//     forcado sairia de __kmp_start_autotuning sem fazer nada, e em silencio.
//  3. Roda o MESMO executavel duas vezes, com e sem a variavel. E a
//     propriedade que interessa para a medicao: baseline e autotuning saem do
//     mesmo binario, na mesma maquina, na mesma rodada.
//
// O sinal observavel e o chunk inferido: com forca ele muda entre repeticoes
// (o Nelder-Mead esta explorando); sem forca fica preso no que passamos ao
// dispatch_init.

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct ident {
  int reserved_1;
  int flags;
  int reserved_2;
  int reserved_3;
  char const *psource;
} ident_t;

extern int __kmpc_global_thread_num(ident_t *);
extern void __kmpc_dispatch_init_4(ident_t *loc, int gtid, unsigned atid,
                                   int schedule, int lb, int ub, int st,
                                   int chunk);
extern int __kmpc_dispatch_next_4(ident_t *loc, int gtid, int *p_last,
                                  int *p_lb, int *p_ub, int *p_st);
extern void __kmpc_dispatch_deinit(ident_t *loc, int gtid, unsigned atid,
                                   int schedule);

// De proposito NAO definimos __KMP_NUM_AUTO_MODE aqui. Ver item 2 acima.

#define KMP_SCH_DYNAMIC_CHUNKED 35
// Sem bit de modo: e o que o OMPIRBuilder emite. O ultimo argumento do
// dispatch_deinit tambem vai 0, como no codigo que o OMPIRBuilder gera.
#define SCHED_PLAIN KMP_SCH_DYNAMIC_CHUNKED

#define N 100000
#define REPS 40
#define CHUNK_IN 64 // chunk "do programador", que o modo forcado deve ignorar

static int visits[N];
static int owner[N];
static volatile double sink;

static ident_t loc_f = {0, 2, 0, 0, ";dispatch_force_abi.c;forced;1;1;;"};

static double work(int i) {
  double acc = 0.0;
  int n = 8 + (i >> 9);
  for (int k = 0; k < n; ++k)
    acc += (double)(k ^ i) * 1.000001;
  return acc;
}

/// Menor bloco contiguo de iteracoes da mesma thread == chunk do dynamic.
static long infer_chunk(const int *who, long n) {
  long best = -1, start = 0, runs = 0;
  for (long i = 1; i <= n; ++i) {
    if (i == n || who[i] != who[start]) {
      if (i != n) {
        long len = i - start;
        if (best < 0 || len < best)
          best = len;
        ++runs;
      }
      start = i;
    }
  }
  return runs > 0 ? best : -1;
}

static void run_once(int gtid) {
  int last, lb, ub, st;
  double acc = 0.0;
  __kmpc_dispatch_init_4(&loc_f, gtid, /*atid=*/0, SCHED_PLAIN, 0, N - 1, 1,
                         CHUNK_IN);
  while (__kmpc_dispatch_next_4(&loc_f, gtid, &last, &lb, &ub, &st)) {
    int tid = omp_get_thread_num();
    for (int i = lb; i <= ub; ++i) {
      visits[i] += 1;
      owner[i] = tid;
      acc += work(i);
    }
  }
  __kmpc_dispatch_deinit(&loc_f, gtid, /*atid=*/0, /*schedule=*/0);
  sink += acc;
}

int main(void) {
  long seen[REPS];

  printf("threads=%d N=%d reps=%d chunk_in=%d\n", omp_get_max_threads(), N,
         REPS, CHUNK_IN);

#pragma omp parallel
  {
    int gtid = __kmpc_global_thread_num(&loc_f);
    for (int rep = 0; rep < REPS; ++rep) {
#pragma omp barrier
      run_once(gtid);
#pragma omp barrier
#pragma omp master
      seen[rep] = infer_chunk(owner, N);
      // Fecha a janela: sem esta barreira as outras threads entrariam na
      // repeticao seguinte e reescreveriam owner[] enquanto o master ainda
      // esta lendo.
#pragma omp barrier
    }
  }

  // Corretude primeiro: chunks divergentes entre threads duplicam e perdem
  // iteracoes, e isso invalida qualquer medicao feita em cima.
  long bad = 0;
  for (int i = 0; i < N; ++i)
    if (visits[i] != REPS)
      ++bad;
  printf("iteracoes com contagem errada: %ld\n", bad);

  long distinct = 0;
  for (int r = 0; r < REPS; ++r) {
    int novo = 1;
    for (int q = 0; q < r; ++q)
      if (seen[q] == seen[r])
        novo = 0;
    distinct += novo;
  }
  printf("chunks distintos em %d repeticoes: %ld\n", REPS, distinct);
  printf("chunk primeiro=%ld ultimo=%ld\n", seen[0], seen[REPS - 1]);

  // Com KMP_AT_FORCE o chunk passado (64) nao e usado: o autotuning escolhe.
  // Sem ele, as 40 repeticoes rodam com 64 e ha um unico chunk distinto.
  printf("%s\n", distinct > 1 ? "AUTOTUNING ATIVO" : "AUTOTUNING INATIVO");
  return bad != 0;
}

// CHECK: iteracoes com contagem errada: 0
// CHECK: AUTOTUNING ATIVO

// OFF: iteracoes com contagem errada: 0
// OFF: chunks distintos em 40 repeticoes: 1
// OFF: AUTOTUNING INATIVO
