// RUN: %libomp-compile-and-run
//
// Teste end-to-end de schedule(dynamic, auto).  Precisa de um clang com o
// suporte a chunk "auto"; com um clang normal isto é erro de compilação.
//
// O teste faz duas coisas:
//
//  1. CORRETUDE.  Cada iteração incrementa visits[i].  Depois de R execuções
//     todo visits[i] tem de valer exatamente R.  Isso pega o caso em que
//     threads do mesmo time recebem chunks DIFERENTES: em dynamic o contador
//     de iterações é compartilhado mas o chunk é privado (pr->u.p.parm1), de
//     modo que chunks divergentes produzem iterações duplicadas e perdidas.
//
//  2. OBSERVABILIDADE.  Não há API para ler o chunk escolhido, mas em dynamic
//     as iterações são entregues em blocos contíguos de tamanho `chunk`.
//     Gravando qual thread executou cada iteração e medindo o menor bloco
//     contíguo, recuperamos o chunk efetivo -- e dá para ver o Nelder-Mead
//     convergindo ao longo das execuções.
//
// Carga propositalmente desbalanceada (custo cresce com i) para que o chunk
// realmente importe.

#include <omp.h>
#include <stdio.h>
#include <stdlib.h>

#define N 200000
#define REPS 60

static int owner[N];
static int visits[N];

/// Trabalho sintético cujo custo cresce linearmente com o índice.
static double work(int i) {
  double acc = 0.0;
  int n = 8 + (i >> 7);
  for (int k = 0; k < n; ++k)
    acc += (double)(k ^ i) * 1.000001;
  return acc;
}

/// Menor bloco contíguo de iterações executadas pela mesma thread.  Em
/// schedule(dynamic, C) isso recupera C (o último bloco pode ser parcial e é
/// descartado).  Devolve -1 se houver um único bloco (padrão de static).
static long infer_chunk(const int *who, long n) {
  long best = -1;
  long start = 0;
  long runs = 0;
  for (long i = 1; i <= n; ++i) {
    if (i == n || who[i] != who[start]) {
      if (i != n) { // descarta o último bloco, que pode ser parcial
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

int main(void) {
  double sink = 0.0;
  int failures = 0;

  for (int i = 0; i < N; ++i)
    visits[i] = 0;

  printf("threads=%d  N=%d  reps=%d\n\n", omp_get_max_threads(), N, REPS);
  printf("  rep      tempo(s)   chunk inferido\n");
  printf("  ------------------------------------\n");

  for (int rep = 1; rep <= REPS; ++rep) {
    double t0 = omp_get_wtime();

#pragma omp parallel for schedule(dynamic, auto)
    for (int i = 0; i < N; ++i) {
      owner[i] = omp_get_thread_num();
      visits[i] += 1;
      sink += work(i);
    }

    double dt = omp_get_wtime() - t0;
    long chunk = infer_chunk(owner, N);

    // Corretude: toda iteração executada exatamente uma vez nesta execução.
    long bad = 0;
    long first_bad = -1;
    for (int i = 0; i < N; ++i) {
      if (visits[i] != rep) {
        if (first_bad < 0)
          first_bad = i;
        ++bad;
      }
    }

    if (chunk < 0)
      printf("  %3d   %10.4f   (bloco único -- chunk ignorado?)\n", rep, dt);
    else
      printf("  %3d   %10.4f   %ld\n", rep, dt, chunk);

    if (bad) {
      printf("      ERRO: %ld iterações com contagem errada; "
             "visits[%ld] = %d, esperado %d\n",
             bad, first_bad, visits[first_bad], rep);
      ++failures;
      // Ressincroniza para não cascatear o erro nas próximas execuções.
      for (int i = 0; i < N; ++i)
        visits[i] = rep;
    }
  }

  printf("\n");
  if (failures) {
    printf("FALHOU: %d execução(ões) com iterações duplicadas ou perdidas\n",
           failures);
    return 1;
  }
  printf("OK: %d execuções, todas as iterações executadas exatamente uma vez\n",
         REPS);
  // Impede que o compilador elimine o trabalho.
  if (sink == 12345.6789)
    fputs("", stderr);
  return 0;
}
