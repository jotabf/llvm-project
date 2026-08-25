#include "kmp_autotuning.h"

volatile int __kmp_global_auto_initialized = FALSE;

/// Tabela de loops "auto", indexada por ordem de chegada. A chave é o ponteiro
/// do ident_t. Slots são preenchidos e nunca liberados até o teardown, então um
/// slot já publicado nunca muda de loc -- é o que permite a leitura sem lock em
/// __kmp_find_autotuning_info.
static kmp_autotuning_info *__kmp_autotuning_table = NULL;
static unsigned __kmp_autotuning_slots = KMP_AUTOTUNING_DEFAULT_SLOTS;
/// Número de slots já publicados. Só cresce, e só sob __kmp_autotuning_lock.
static volatile unsigned __kmp_autotuning_used = 0;
static kmp_bootstrap_lock_t __kmp_autotuning_lock;
static int __kmp_autotuning_table_full = FALSE;

/// Indice hash de endereçamento aberto sobre o ponteiro do ident_t.
///
/// A varredura linear custava O(K) COM PASSO DE 192 BYTES -- uma linha de
/// cache por entrada, e ainda por cima a linha que contem `loc` e' a mesma que
/// contem `count`, que toda thread escreve no fim do loop. Com 200 loops
/// registrados isso media ~200 ns por busca. Aqui as chaves ficam num array
/// contiguo de ponteiros (8 por linha de cache) e a busca e' O(1).
///
/// Leitura sem lock pelo mesmo argumento da tabela: uma posicao so' vai de
/// NULL para um loc, nunca muda depois, e nunca e' reaproveitada.
static ident_t **__kmp_at_keys = NULL;
static unsigned *__kmp_at_slot = NULL;
static unsigned __kmp_at_hash_mask = 0;

/// Fibonacci hashing: espalha os bits altos do ponteiro, que e' onde a
/// variacao esta' (idents sao globais alinhadas, os bits baixos repetem).
static inline unsigned __kmp_at_hash(ident_t *loc) {
  kmp_uint64 h = (kmp_uint64)(uintptr_t)loc;
  h *= 0x9E3779B97F4A7C15ull;
  return (unsigned)(h >> 40) & __kmp_at_hash_mask;
}

/// Profiler do caminho quente. Ver kmp_autotuning.h.
int __kmp_at_profile = FALSE;
kmp_at_prof *__kmp_at_prof_tab = NULL;

/// Cache de uma entrada por thread. Ver kmp_autotuning.h.
kmp_at_cache_ent *__kmp_at_cache = NULL;

/// Ver KMP_AT_OPT em kmp_autotuning.h.
int __kmp_at_opt = KMP_AT_OPT_ALL;

/// Segundos por tick de KMP_NOW(). Em x86 KMP_NOW() e' o TSC e a conversao usa
/// __kmp_ticks_per_usec; nas demais arquiteturas KMP_NOW() ja' e' nanossegundo.
static inline double __kmp_at_tick_seconds(void) {
#if KMP_OS_UNIX && (KMP_ARCH_X86 || KMP_ARCH_X86_64)
  return __kmp_ticks_per_usec > 0 ? 1e-6 / (double)__kmp_ticks_per_usec : 1e-9;
#else
  return 1e-9;
#endif
}

Autotuning *Autotuning::Create(int64_t min, int64_t max, unsigned ignore) {
  const int DIM = 1;
  const int MIN_ERROR = 1;

  Autotuning *at =
      static_cast<Autotuning *>(__kmp_allocate(sizeof(Autotuning)));

  at->p_point = NULL;
  at->m_ignore = ignore + 1;
  at->m_iter = 0;
  at->m_t0 = 0.0;
  at->m_tick0 = 0;
  at->m_runtime = 0.0;
  at->p_optimizer = NelderMead::Create(min, max, DIM, MIN_ERROR);

  return at;
}

void Autotuning::start() {
  // End execution
  if (p_optimizer->isEnd())
    return;

  if ((m_iter % m_ignore) == 0) {
    p_point = p_optimizer->run(m_runtime);
  }

  if (p_optimizer->isEnd()) {
    p_point = p_optimizer->getMinPoint();
  }

  // Relógio de PAREDE. clock() devolvia tempo de CPU do processo inteiro,
  // somado entre todas as threads e incluindo trabalho concorrente alheio ao
  // loop -- o custo alimentado ao Nelder-Mead não media o que se queria medir.
  //
  // KMP_NOW() tem resolucao de nanossegundos; __kmp_elapsed() usa gettimeofday
  // (z_Linux_util.cpp), cujo passo e' 1 us -- num loop de poucos
  // microssegundos isso sozinho ja' domina o custo medido.
  if (__kmp_at_opt & KMP_AT_OPT_NSEC)
    m_tick0 = KMP_NOW();
  else
    __kmp_elapsed(&m_t0);
}

void Autotuning::reset(unsigned level) {
  m_iter = 0;
  p_optimizer->reset(level);
}

void Autotuning::end() {
  if (!p_optimizer->isEnd()) {
    if (__kmp_at_opt & KMP_AT_OPT_NSEC) {
      m_runtime = (double)(KMP_NOW() - m_tick0) * __kmp_at_tick_seconds();
    } else {
      double t1;
      __kmp_elapsed(&t1);
      m_runtime = t1 - m_t0; // segundos
    }
    ++m_iter;
  }
}

void __kmp_autotuning_global_initialize(int gtid) {

  if (TCR_4(__kmp_global_auto_initialized))
    return;
  __kmp_acquire_bootstrap_lock(&__kmp_initz_lock);
  if (TCR_4(__kmp_global_auto_initialized)) {
    __kmp_release_bootstrap_lock(&__kmp_initz_lock);
    return;
  }

  const char *env = getenv("KMP_AUTOTUNING_SLOTS");
  if (env && *env) {
    long v = strtol(env, NULL, 0);
    if (v > 0)
      __kmp_autotuning_slots = static_cast<unsigned>(v);
  }

  const char *oenv = getenv("KMP_AT_OPT");
  if (oenv && *oenv)
    __kmp_at_opt = (int)strtol(oenv, NULL, 0);

  const char *penv = getenv("KMP_AT_PROFILE");
  if (penv && *penv && strcmp(penv, "0") != 0) {
    __kmp_at_profile = TRUE;
    __kmp_at_prof_tab = static_cast<kmp_at_prof *>(
        __kmp_allocate(sizeof(kmp_at_prof) * KMP_AT_PROF_MAX_THREADS));
  }

  __kmp_at_cache = static_cast<kmp_at_cache_ent *>(
      __kmp_allocate(sizeof(kmp_at_cache_ent) * KMP_AT_CACHE_SLOTS));

  __kmp_init_bootstrap_lock(&__kmp_autotuning_lock);

  __kmp_autotuning_table = static_cast<kmp_autotuning_info *>(
      __kmp_allocate(sizeof(kmp_autotuning_info) * __kmp_autotuning_slots));

  // Fator de carga <= 1/4: com enderecamento aberto e sondagem linear, acima
  // de ~1/2 o numero de sondas cresce rapido.
  unsigned cap = 4;
  while (cap < __kmp_autotuning_slots * 4)
    cap <<= 1;
  __kmp_at_hash_mask = cap - 1;
  __kmp_at_keys = static_cast<ident_t **>(__kmp_allocate(sizeof(ident_t *) * cap));
  __kmp_at_slot = static_cast<unsigned *>(__kmp_allocate(sizeof(unsigned) * cap));

  for (unsigned i = 0; i < __kmp_autotuning_slots; ++i) {
    kmp_autotuning_info *info = &__kmp_autotuning_table[i];
    TCW_SYNC_4(info->initialized, FALSE);
    TCW_SYNC_4(info->started, FALSE);
    TCW_SYNC_4(info->ended, FALSE);
    TCW_SYNC_4(info->frozen, FALSE);
    info->chunk = 1;
    KMP_ATOMIC_ST_REL(&info->count, 0);
    __kmp_init_bootstrap_lock(&info->start_lock);
    info->loc = NULL;
    info->at = NULL;
  }
  __kmp_autotuning_used = 0;

  // A barreira vem ANTES do store da flag: TCW_SYNC_4 é store volátil puro,
  // sem fence, e sem isto uma thread em ARM pode ver a flag TRUE antes de ver
  // o ponteiro da tabela.
  KMP_MB();
  TCW_SYNC_4(__kmp_global_auto_initialized, TRUE);
  KMP_MB();

  KA_TRACE(10, ("__kmp_autotuning_global_initialize: T#%d table with %u "
                "slots.\n",
                gtid, __kmp_autotuning_slots));

  __kmp_release_bootstrap_lock(&__kmp_initz_lock);
}

kmp_autotuning_info *__kmp_find_autotuning_info(ident_t *loc) {
  if (loc == NULL || !TCR_4(__kmp_global_auto_initialized))
    return NULL;

  // O profiler usa gtid so' para achar seus contadores; aqui nao temos gtid,
  // entao pegamos o da thread corrente. E' barato (leitura de TLS) e so'
  // acontece com KMP_AT_PROFILE ligado.
  kmp_at_prof *prof =
      __kmp_at_profile ? __kmp_at_prof_get(__kmp_get_gtid()) : NULL;
  KMP_AT_PROF_SCOPE(prof, find);

  if ((__kmp_at_opt & KMP_AT_OPT_KEYS) && __kmp_at_keys != NULL) {
    unsigned h = __kmp_at_hash(loc);
    unsigned probes = 0;
    for (;;) {
      ident_t *k = __kmp_at_keys[h];
      ++probes;
      if (k == NULL) { // posicao vazia => a chave nao esta' na tabela
        if (prof)
          prof->find_steps += probes;
        return NULL;
      }
      if (k == loc) {
        if (prof)
          prof->find_steps += probes;
        return &__kmp_autotuning_table[__kmp_at_slot[h]];
      }
      h = (h + 1) & __kmp_at_hash_mask;
      if (probes > __kmp_at_hash_mask) // tabela cheia: nao girar para sempre
        break;
    }
    if (prof)
      prof->find_steps += probes;
    return NULL;
  }

  // Leitura sem lock: slots só são preenchidos, nunca reaproveitados, e
  // __kmp_autotuning_used só cresce. Ler um valor defasado de `used` no pior
  // caso faz perder um slot recém-criado, e o chamador tenta de novo.
  const unsigned used = TCR_4(__kmp_autotuning_used);
  for (unsigned i = 0; i < used; ++i) {
    if (__kmp_autotuning_table[i].loc == loc) {
      if (prof)
        prof->find_steps += i + 1;
      return &__kmp_autotuning_table[i];
    }
  }
  if (prof)
    prof->find_steps += used;
  return NULL;
}

kmp_autotuning_info *__kmp_get_autotuning_info(ident_t *loc) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc);
  if (info != NULL)
    return info;
  if (loc == NULL || !TCR_4(__kmp_global_auto_initialized))
    return NULL;

  __kmp_acquire_bootstrap_lock(&__kmp_autotuning_lock);

  // Outra thread pode ter criado enquanto esperávamos o lock.
  for (unsigned i = 0; i < __kmp_autotuning_used; ++i) {
    if (__kmp_autotuning_table[i].loc == loc) {
      __kmp_release_bootstrap_lock(&__kmp_autotuning_lock);
      return &__kmp_autotuning_table[i];
    }
  }

  if (__kmp_autotuning_used >= __kmp_autotuning_slots) {
    if (!__kmp_autotuning_table_full) {
      __kmp_autotuning_table_full = TRUE;
      fprintf(stderr,
              "OMP: Warning: autotuning table full (%u slots); further "
              "'auto' loops run with chunk 1. Raise KMP_AUTOTUNING_SLOTS.\n",
              __kmp_autotuning_slots);
    }
    __kmp_release_bootstrap_lock(&__kmp_autotuning_lock);
    return NULL;
  }

  const unsigned slot = __kmp_autotuning_used;
  __kmp_autotuning_table[slot].loc = loc;

  if (__kmp_at_keys != NULL) {
    unsigned h = __kmp_at_hash(loc);
    while (__kmp_at_keys[h] != NULL)
      h = (h + 1) & __kmp_at_hash_mask;
    __kmp_at_slot[h] = slot;
    // O indice do slot tem de estar visivel ANTES da chave: quem le ve a
    // chave e em seguida le o indice.
    KMP_MB();
    __kmp_at_keys[h] = loc;
  }

  // Publica o loc antes de tornar o slot visível pelo contador.
  KMP_MB();
  TCW_SYNC_4(__kmp_autotuning_used, slot + 1);
  KMP_MB();

  __kmp_release_bootstrap_lock(&__kmp_autotuning_lock);
  return &__kmp_autotuning_table[slot];
}

/// Ganchos de TESTE. Nao fazem parte da ABI do OpenMP: existem para os
/// benchmarks. O nome comeca em kmp_ (e nao __kmp_) porque so' esse padrao e'
/// exportado pelo version script em exports_so.txt.
/// benchmarks casarem exatamente o chunk do baseline com o que o autotuning
/// escolheu. Inferir o chunk pelo padrao de blocos nao serve -- o resto do
/// loop forma um bloco parcial menor que o chunk.
extern "C" int64_t kmp_autotuning_debug_chunk(ident_t *loc) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc);
  if (info == NULL || !TCR_4(info->initialized) || info->at == NULL)
    return -1;
  return info->at->getPoint();
}

extern "C" int kmp_autotuning_debug_converged(ident_t *loc) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc);
  if (info == NULL || !TCR_4(info->initialized) || info->at == NULL)
    return -1;
  return info->at->isEnd() ? 1 : 0;
}

extern "C" unsigned kmp_autotuning_debug_iters(ident_t *loc) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc);
  if (info == NULL || !TCR_4(info->initialized) || info->at == NULL)
    return 0;
  return info->at->getIter();
}

void __kmp_end_autotuning(int gtid, ident_t *loc) {
  // Mesma guarda de símbolo fraco de __kmp_start_autotuning.
  if (&__KMP_NUM_AUTO_MODE == nullptr || loc == NULL ||
      !TCR_4(__kmp_global_auto_initialized))
    return;

  kmp_at_prof *prof = __kmp_at_prof_get(gtid);
  KMP_AT_PROF_SCOPE(prof, end);

  // O start desta mesma execucao ja' publicou (loc, info) no cache desta
  // thread; aqui isso vira uma comparacao de ponteiro em vez de uma terceira
  // varredura linear da tabela.
  kmp_autotuning_info *info = __kmp_at_cache_get(gtid, loc);
  if (info == NULL)
    info = __kmp_find_autotuning_info(loc);

  // info->at == NULL: autotuning desabilitado para este loop. Tem de ser
  // testado ANTES de info->at->isEnd().
  // Mesmo caminho quente do start: um load resolve o caso convergido.
  if (info == NULL)
    return;
  if ((__kmp_at_opt & KMP_AT_OPT_FROZEN) && TCR_4(info->frozen))
    return;

  if (!TCR_4(info->initialized) || info->at == NULL || info->at->isEnd())
    return;

  // Tamanho do TIME que está executando este loop, não __kmp_nth (que é a
  // contagem de threads do processo inteiro e não bate com o time quando há
  // regiões aninhadas, num_threads menor, ou hot teams retidos).
  //
  // KMP_ATOMIC_ADD é fetch_add (kmp_os.h:1263): devolve o valor ANTIGO. Com um
  // time de T threads a última a chegar recebe T-1, por isso a comparação é
  // com nproc - 1 e não com nproc.
  const int nproc = __kmp_threads[gtid]->th.th_team_nproc;
  // Incremento RELAXADO: este contador so' elege quem fecha a medicao, nao
  // publica dado nenhum. O acquire/release de que a eleicao precisa ja' vem do
  // start_lock logo abaixo. Em x86 nao muda o codigo gerado; em ARM (A64FX) o
  // acq_rel do KMP_ATOMIC_ADD emitiria barreiras de memoria a cada saida de
  // loop de cada thread, durante toda a fase de exploracao.
  int count = KMP_ATOMIC_ADD_RLX(&info->count, 1);
  if (count == nproc - 1) {
    // Mesmo lock de __kmp_start_autotuning. Autotuning::end() lê m_t0 e
    // escreve m_runtime/m_iter, enquanto start() lê m_runtime e escreve m_t0 --
    // nenhum dos dois é atômico. A barreira em start() estreita a janela mas
    // não a fecha: o end() do time N pode cruzar com o start() do time N+1.
    //
    // Não há risco de deadlock: quem segura o start_lock (em
    // __kmp_start_autotuning) nunca chama __kmp_end_autotuning de dentro dele.
    __kmp_acquire_bootstrap_lock(&info->start_lock);

    info->at->end();

    KMP_ATOMIC_ST_REL(&info->count, 0);
    TCW_SYNC_4(info->started, FALSE);
    TCW_SYNC_4(info->ended, TRUE);
    KMP_MB();

    __kmp_release_bootstrap_lock(&info->start_lock);
  }
}

/// Converte ticks de KMP_NOW() em nanossegundos. Em x86 KMP_NOW() e' o TSC;
/// nas demais arquiteturas ja' e' nanossegundo.
static double __kmp_at_ticks_to_ns(void) {
#if KMP_OS_UNIX && (KMP_ARCH_X86 || KMP_ARCH_X86_64)
  if (__kmp_ticks_per_usec > 0)
    return 1000.0 / (double)__kmp_ticks_per_usec;
  return 1.0;
#else
  return 1.0;
#endif
}

/// Soma os contadores de todas as threads e imprime ns medios por chamada.
/// Os escopos sao ANINHADOS: start inclui bar e find; end inclui find. As
/// colunas "excl." descontam o que esta' dentro.
static void __kmp_at_prof_report(const char *tag, int reset) {
  if (!__kmp_at_profile || __kmp_at_prof_tab == NULL)
    return;

  kmp_at_prof tot;
  memset(&tot, 0, sizeof(tot));
  for (int i = 0; i < KMP_AT_PROF_MAX_THREADS; ++i) {
    kmp_at_prof *p = &__kmp_at_prof_tab[i];
    tot.start_cyc += p->start_cyc;   tot.start_n += p->start_n;
    tot.bar_cyc   += p->bar_cyc;     tot.bar_n   += p->bar_n;
    tot.end_cyc   += p->end_cyc;     tot.end_n   += p->end_n;
    tot.find_cyc  += p->find_cyc;    tot.find_n  += p->find_n;
    tot.find_steps += p->find_steps;
  }

  const double k = __kmp_at_ticks_to_ns();
  const double start_ns = (double)tot.start_cyc * k;
  const double bar_ns = (double)tot.bar_cyc * k;
  const double end_ns = (double)tot.end_cyc * k;
  const double find_ns = (double)tot.find_cyc * k;

#define AT_AVG(ns, n) ((n) ? (ns) / (double)(n) : 0.0)

  fprintf(stderr, "\n=== autotuning: perfil do caminho quente [%s] ===\n", tag);
  fprintf(stderr, "%-14s %12s %14s %14s\n", "trecho", "chamadas", "ns/chamada",
          "ns total");
  fprintf(stderr, "%-14s %12llu %14.1f %14.0f\n", "start (incl)",
          (unsigned long long)tot.start_n, AT_AVG(start_ns, tot.start_n),
          start_ns);
  fprintf(stderr, "%-14s %12llu %14.1f %14.0f\n", "  barreira",
          (unsigned long long)tot.bar_n, AT_AVG(bar_ns, tot.bar_n), bar_ns);
  fprintf(stderr, "%-14s %12llu %14.1f %14.0f\n", "end (incl)",
          (unsigned long long)tot.end_n, AT_AVG(end_ns, tot.end_n), end_ns);
  fprintf(stderr, "%-14s %12llu %14.1f %14.0f\n", "busca",
          (unsigned long long)tot.find_n, AT_AVG(find_ns, tot.find_n), find_ns);
  fprintf(stderr, "%-14s %12llu %14.1f %14.0f\n", "start excl.bar",
          (unsigned long long)tot.start_n,
          AT_AVG(start_ns - bar_ns, tot.start_n), start_ns - bar_ns);
  fprintf(stderr, "slots visitados por busca: %.2f  (total %llu)\n",
          tot.find_n ? (double)tot.find_steps / (double)tot.find_n : 0.0,
          (unsigned long long)tot.find_steps);
  fprintf(stderr, "TOTAL autotuning (start+end) = %.0f ns = %.3f ms\n\n",
          start_ns + end_ns, (start_ns + end_ns) / 1e6);

  if (reset)
    memset(__kmp_at_prof_tab, 0, sizeof(kmp_at_prof) * KMP_AT_PROF_MAX_THREADS);

#undef AT_AVG
}

/// Gancho de TESTE: imprime o perfil acumulado e zera os contadores, para o
/// benchmark separar a fase de exploracao da fase em regime. Sem isso o
/// relatorio final mistura as duas e o custo em regime fica mascarado pela
/// barreira, que so' existe durante a exploracao.
extern "C" void kmp_autotuning_debug_prof_dump(const char *tag) {
  __kmp_at_prof_report(tag ? tag : "?", /*reset=*/1);
}

void __kmp_autotuning_global_cleanup(void) {
  if (!TCR_4(__kmp_global_auto_initialized))
    return;

  const unsigned used = TCR_4(__kmp_autotuning_used);

  // O relatório é o motivo real deste gancho existir: liberar memória na saída
  // do processo não muda nada, mas este é o único ponto em que dá para dizer,
  // por loop, com que chunk o autotuning parou e se chegou a convergir.
  if (used > 0 && getenv("KMP_AUTOTUNING_QUIET") == NULL) {
    fprintf(stderr, "\n=== OpenMP scheduling autotuning ===\n");
    fprintf(stderr, "%-6s %-10s %-9s %-9s %-14s %s\n", "slot", "chunk",
            "medicoes", "convergiu", "ult.custo(s)", "loop");
    for (unsigned i = 0; i < used; ++i) {
      kmp_autotuning_info *info = &__kmp_autotuning_table[i];
      const char *src = info->loc ? info->loc->psource : "?";
      if (info->at == NULL) {
        fprintf(stderr, "%-6u %-10s %-9s %-9s %-14s %s\n", i, "-", "-",
                "desabil.", "-", src);
        continue;
      }
      fprintf(stderr, "%-6u %-10lld %-9u %-9s %-14.9f %s\n", i,
              (long long)info->at->getPoint(), info->at->getIter(),
              info->at->isEnd() ? "sim" : "nao", info->at->getRuntime(), src);
    }
    fprintf(stderr, "\n");
  }

  __kmp_at_prof_report("final", /*reset=*/0);

  for (unsigned i = 0; i < used; ++i) {
    Autotuning::Destroy(__kmp_autotuning_table[i].at);
    __kmp_autotuning_table[i].at = NULL;
    __kmp_autotuning_table[i].loc = NULL;
  }

  __kmp_free(__kmp_autotuning_table);
  __kmp_autotuning_table = NULL;
  __kmp_autotuning_used = 0;

  if (__kmp_at_keys != NULL) {
    __kmp_free(__kmp_at_keys);
    __kmp_free(__kmp_at_slot);
    __kmp_at_keys = NULL;
    __kmp_at_slot = NULL;
    __kmp_at_hash_mask = 0;
  }

  if (__kmp_at_cache != NULL) {
    __kmp_free(__kmp_at_cache);
    __kmp_at_cache = NULL;
  }

  if (__kmp_at_prof_tab != NULL) {
    __kmp_free(__kmp_at_prof_tab);
    __kmp_at_prof_tab = NULL;
  }
  __kmp_at_profile = FALSE;

  TCW_SYNC_4(__kmp_global_auto_initialized, FALSE);
}
