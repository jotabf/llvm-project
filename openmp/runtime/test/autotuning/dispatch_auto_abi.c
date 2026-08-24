// RUN: %libomp-compile-and-run
//
// Testa schedule(dynamic, auto) SEM precisar de um clang patchado: chama as
// entradas __kmpc_dispatch_* diretamente, montando o schedule na mão com o bit
// kmp_sch_chunk_mode_auto (1<<28). É assim que os testes kmp_sch_* da libomp já
// exercitam o dispatch.
//
// Verifica três coisas:
//
//  1. CORRETUDE: cada iteração é executada exatamente uma vez por repetição.
//     Se as threads de um mesmo time receberem chunks diferentes, o contador de
//     iterações é compartilhado mas o chunk é privado (pr->u.p.parm1), então
//     iterações são duplicadas e perdidas -- e isso aparece aqui.
//
//  2. NOWAIT: a variante B não põe barreira entre as repetições, de modo que
//     uma thread pode entrar na repetição k+1 enquanto outra ainda está na k.
//     É o caso que a barreira dentro de __kmp_start_autotuning existe para
//     cobrir, e também o caso em que ela poderia travar.
//
//  3. CONVERGÊNCIA: imprime o chunk inferido por repetição (em dynamic as
//     iterações saem em blocos contíguos, então o menor bloco é o chunk).

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

// O runtime só liga o autotuning se este símbolo estiver definido no binário
// -- é a global fraca que o clang emite nas TUs com pelo menos um loop "auto".
// Só o endereço é usado; o valor não significa nada. Aqui a definimos à mão
// porque estamos montando o schedule sem o front-end.
const unsigned __KMP_NUM_AUTO_MODE = 1;

#define KMP_SCH_DYNAMIC_CHUNKED 35
#define KMP_SCH_CHUNK_MODE_AUTO (1 << 28)
#define SCHED_AUTO (KMP_SCH_DYNAMIC_CHUNKED | KMP_SCH_CHUNK_MODE_AUTO)

#define N 100000
#define REPS_DEFAULT 40

// Ajustavel para medir quantas repeticoes o Nelder-Mead precisa para convergir
// num dado intervalo: AT_REPS=400 ./dispatch_auto_abi
static int reps = REPS_DEFAULT;

// Dois loops distintos => dois ident_t distintos => duas entradas na tabela.
static ident_t loc_a = {0, 2, 0, 0, ";dispatch_auto_abi.c;loop_a;1;1;;"};
static ident_t loc_b = {0, 2, 0, 0, ";dispatch_auto_abi.c;loop_b;2;2;;"};

static int visits[N];
static int owner[N];
static volatile double sink;

static double work(int i) {
  double acc = 0.0;
  int n = 8 + (i >> 9);
  for (int k = 0; k < n; ++k)
    acc += (double)(k ^ i) * 1.000001;
  return acc;
}

/// Menor bloco contíguo de iterações da mesma thread == chunk do dynamic.
/// O último bloco pode ser parcial e é descartado.
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

/// Uma execução do loop dinâmico com chunk autoajustado.
static void run_once(ident_t *loc, int gtid) {
  int last, lb, ub, st;
  double acc = 0.0;
  __kmpc_dispatch_init_4(loc, gtid, /*atid=*/1, SCHED_AUTO, 0, N - 1, 1, 1);
  while (__kmpc_dispatch_next_4(loc, gtid, &last, &lb, &ub, &st)) {
    int tid = omp_get_thread_num();
    for (int i = lb; i <= ub; ++i) {
      visits[i] += 1;
      owner[i] = tid;
      acc += work(i);
    }
  }
  __kmpc_dispatch_deinit(loc, gtid, /*atid=*/1, SCHED_AUTO);
  sink += acc;
}

static int check(int rep, const char *tag) {
  long bad = 0, first = -1;
  for (int i = 0; i < N; ++i) {
    if (visits[i] != rep) {
      if (first < 0)
        first = i;
      ++bad;
    }
  }
  if (bad) {
    printf("  %s rep %d: ERRO -- %ld iteracoes com contagem errada "
           "(visits[%ld]=%d, esperado %d)\n",
           tag, rep, bad, first, visits[first], rep);
    for (int i = 0; i < N; ++i)
      visits[i] = rep; // ressincroniza para nao cascatear
    return 1;
  }
  return 0;
}

int main(void) {
  int failures = 0;
  int nth = omp_get_max_threads();

  const char *e = getenv("AT_REPS");
  if (e && *e) {
    int v = atoi(e);
    if (v > 0)
      reps = v;
  }

  printf("threads=%d N=%d reps=%d\n", nth, N, reps);

  // ---------------------------------------------------------- variante A
  // Com barreira entre as repeticoes: equivale a um "omp for" sem nowait.
  printf("\n[A] com barreira entre repeticoes (equivale a sem nowait)\n");
  for (int i = 0; i < N; ++i)
    visits[i] = 0;

  printf("  %-5s %-10s %-12s\n", "rep", "chunk", "tempo(s)");
  {
    double t0 = 0.0;
#pragma omp parallel
    {
      int gtid = __kmpc_global_thread_num(&loc_a);
      for (int rep = 1; rep <= reps; ++rep) {
        // As barreiras em volta fazem a janela medida aqui coincidir com a
        // que o runtime mede: do start() da primeira thread ao end() da
        // ultima. E o que permite conferir m_runtime contra omp_get_wtime().
#pragma omp barrier
#pragma omp master
        t0 = omp_get_wtime();
#pragma omp barrier

        run_once(&loc_a, gtid);

#pragma omp barrier
#pragma omp master
        {
          double dt = omp_get_wtime() - t0;
          long c = infer_chunk(owner, N);
          if (rep <= 6 || rep % (reps / 10 > 0 ? reps / 10 : 1) == 0 ||
              rep == reps)
            printf("  %-5d %-10ld %-12.6f\n", rep, c, dt);
        }
#pragma omp barrier
      }
    }
  }
  failures += check(reps, "[A]");

  // ---------------------------------------------------------- variante B
  // Sem barreira nenhuma entre as repeticoes: uma thread pode entrar na
  // repeticao k+1 enquanto outra ainda esta na k. Se a barreira interna de
  // __kmp_start_autotuning estiver mal colocada, isto trava.
  printf("\n[B] sem barreira entre repeticoes (equivale a nowait)\n");
  for (int i = 0; i < N; ++i)
    visits[i] = 0;

#pragma omp parallel
  {
    int gtid = __kmpc_global_thread_num(&loc_b);
    for (int rep = 1; rep <= reps; ++rep)
      run_once(&loc_b, gtid);
  }
  failures += check(reps, "[B]");
  printf("  chunk final inferido=%ld\n", infer_chunk(owner, N));

  printf("\n");
  if (failures) {
    printf("FALHOU: %d verificacao(oes)\n", failures);
    return 1;
  }
  printf("OK: iteracoes executadas exatamente uma vez em ambas as variantes\n");
  return 0;
}
