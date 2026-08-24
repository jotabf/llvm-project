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

Autotuning *Autotuning::Create(int64_t min, int64_t max, unsigned ignore) {
  const int DIM = 1;
  const int MIN_ERROR = 1;

  Autotuning *at =
      static_cast<Autotuning *>(__kmp_allocate(sizeof(Autotuning)));

  at->p_point = NULL;
  at->m_ignore = ignore + 1;
  at->m_iter = 0;
  at->m_t0 = 0.0;
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
  // __kmp_elapsed() é a mesma primitiva usada por omp_get_wtime()
  // (kmp_ftn_entry.h:1309) e é declarada sem guarda de arquitetura
  // (kmp.h:4020), ao contrário de __kmp_now_nsec().
  __kmp_elapsed(&m_t0);
}

void Autotuning::reset(unsigned level) {
  m_iter = 0;
  p_optimizer->reset(level);
}

void Autotuning::end() {
  if (!p_optimizer->isEnd()) {
    double t1;
    __kmp_elapsed(&t1);
    m_runtime = t1 - m_t0; // segundos
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

  __kmp_init_bootstrap_lock(&__kmp_autotuning_lock);

  __kmp_autotuning_table = static_cast<kmp_autotuning_info *>(
      __kmp_allocate(sizeof(kmp_autotuning_info) * __kmp_autotuning_slots));

  for (unsigned i = 0; i < __kmp_autotuning_slots; ++i) {
    kmp_autotuning_info *info = &__kmp_autotuning_table[i];
    TCW_SYNC_4(info->initialized, FALSE);
    TCW_SYNC_4(info->started, FALSE);
    TCW_SYNC_4(info->ended, FALSE);
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

  // Leitura sem lock: slots só são preenchidos, nunca reaproveitados, e
  // __kmp_autotuning_used só cresce. Ler um valor defasado de `used` no pior
  // caso faz perder um slot recém-criado, e o chamador tenta de novo.
  const unsigned used = TCR_4(__kmp_autotuning_used);
  for (unsigned i = 0; i < used; ++i) {
    if (__kmp_autotuning_table[i].loc == loc)
      return &__kmp_autotuning_table[i];
  }
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
  // Publica o loc antes de tornar o slot visível pelo contador.
  KMP_MB();
  TCW_SYNC_4(__kmp_autotuning_used, slot + 1);
  KMP_MB();

  __kmp_release_bootstrap_lock(&__kmp_autotuning_lock);
  return &__kmp_autotuning_table[slot];
}

void __kmp_end_autotuning(int gtid, ident_t *loc) {
  // Mesma guarda de símbolo fraco de __kmp_start_autotuning.
  if (&__KMP_NUM_AUTO_MODE == nullptr || loc == NULL ||
      !TCR_4(__kmp_global_auto_initialized))
    return;

  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc);

  // info->at == NULL: autotuning desabilitado para este loop. Tem de ser
  // testado ANTES de info->at->isEnd().
  if (info == NULL || !TCR_4(info->initialized) || info->at == NULL ||
      info->at->isEnd())
    return;

  // Tamanho do TIME que está executando este loop, não __kmp_nth (que é a
  // contagem de threads do processo inteiro e não bate com o time quando há
  // regiões aninhadas, num_threads menor, ou hot teams retidos).
  //
  // KMP_ATOMIC_ADD é fetch_add (kmp_os.h:1263): devolve o valor ANTIGO. Com um
  // time de T threads a última a chegar recebe T-1, por isso a comparação é
  // com nproc - 1 e não com nproc.
  const int nproc = __kmp_threads[gtid]->th.th_team_nproc;
  int count = KMP_ATOMIC_ADD(&info->count, 1);
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

void __kmp_autotuning_global_cleanup(void) {
  if (!TCR_4(__kmp_global_auto_initialized))
    return;

  const unsigned used = TCR_4(__kmp_autotuning_used);

  // O relatório é o motivo real deste gancho existir: liberar memória na saída
  // do processo não muda nada, mas este é o único ponto em que dá para dizer,
  // por loop, com que chunk o autotuning parou e se chegou a convergir.
  if (used > 0 && getenv("KMP_AUTOTUNING_QUIET") == NULL) {
    fprintf(stderr, "\n=== OpenMP scheduling autotuning ===\n");
    fprintf(stderr, "%-6s %-10s %-9s %-9s %-12s %s\n", "slot", "chunk",
            "medicoes", "convergiu", "ult.custo(s)", "loop");
    for (unsigned i = 0; i < used; ++i) {
      kmp_autotuning_info *info = &__kmp_autotuning_table[i];
      const char *src = info->loc ? info->loc->psource : "?";
      if (info->at == NULL) {
        fprintf(stderr, "%-6u %-10s %-9s %-9s %-12s %s\n", i, "-", "-",
                "desabil.", "-", src);
        continue;
      }
      fprintf(stderr, "%-6u %-10lld %-9u %-9s %-12.6f %s\n", i,
              (long long)info->at->getPoint(), info->at->getIter(),
              info->at->isEnd() ? "sim" : "nao", info->at->getRuntime(), src);
    }
    fprintf(stderr, "\n");
  }

  for (unsigned i = 0; i < used; ++i) {
    Autotuning::Destroy(__kmp_autotuning_table[i].at);
    __kmp_autotuning_table[i].at = NULL;
    __kmp_autotuning_table[i].loc = NULL;
  }

  __kmp_free(__kmp_autotuning_table);
  __kmp_autotuning_table = NULL;
  __kmp_autotuning_used = 0;
  TCW_SYNC_4(__kmp_global_auto_initialized, FALSE);
}
