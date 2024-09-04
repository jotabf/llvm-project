#include "kmp_autotuning.h"

volatile bool __kmp_global_auto_initialized = FALSE;
kmp_autotuning_info *__kmp_sched_autotunig_vector;

Autotuning *Autotuning::Create(int64_t min, int64_t max, unsigned ignore) {
  const int DIM = 1;
  const int MIN_ERROR = 1;

  Autotuning *at =
      static_cast<Autotuning *>(__kmp_allocate(sizeof(Autotuning)));

  at->p_point = NULL;
  // at->p_min = min;
  // at->p_max = max;
  at->m_ignore = ignore + 1;
  at->m_iter = 0;
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

  ++m_iter;
  m_t0 = clock();
}

void Autotuning::reset(unsigned level) {
  m_iter = 0;
  p_optimizer->reset(level);
}

void Autotuning::end() {
  if (!p_optimizer->isEnd()) {
    const clock_t m_t1 = clock();
    m_runtime = static_cast<double>(m_t1 - m_t0) / (double)CLOCKS_PER_SEC;
  }
}

void __kmp_autotuning_global_initialize() {
  if (TCR_4(__kmp_global_auto_initialized))
    return;
  __kmp_acquire_bootstrap_lock(&__kmp_initz_lock);
  if (TCR_4(__kmp_global_auto_initialized)) {
    __kmp_release_bootstrap_lock(&__kmp_initz_lock);
    return;
  }

  __kmp_sched_autotunig_vector = static_cast<kmp_autotuning_info *>(
      __kmp_allocate(sizeof(kmp_autotuning_info) * __KMP_NUM_AUTO_MODE));

  for (unsigned i = 0; i < __KMP_NUM_AUTO_MODE; ++i) {
    TCW_SYNC_4(__kmp_sched_autotunig_vector[i].initialized, FALSE);
    TCW_SYNC_4(__kmp_sched_autotunig_vector[i].release_start, FALSE);
    __kmp_sched_autotunig_vector[i].count = 0;
    __kmp_init_bootstrap_lock(&__kmp_sched_autotunig_vector[i].lock);
  }

  KMP_MB(); // Flush initialized
  TCW_SYNC_4(__kmp_global_auto_initialized, TRUE);

  __kmp_release_bootstrap_lock(&__kmp_initz_lock);
}

void __kmp_end_autotuning(int gtid, unsigned id) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(id);

  if (info == NULL || info->at->isEnd())
    return;

  auto count = KMP_ATOMIC_ADD(&info->count, 1);

  if (count == 1) {
    TCW_SYNC_4(info->release_start, FALSE);
    return;
  }

  if (count != __kmp_nth)
    return;

  info->at->end();
  KMP_ATOMIC_ST_REL(&info->count, 0);
}

kmp_autotuning_info *__kmp_find_autotuning_info(unsigned id) {
  if (id == 0)
    return NULL;
  return __kmp_sched_autotunig_vector + id - 1;
}