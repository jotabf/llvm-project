#include "kmp_autotuning.h"
#include <map>

volatile int __kmp_global_auto_initialized = FALSE;
ident_t **__kmp_sched_autotunig_locations;
int64_t *__kmp_sched_autotunig_locations_max; 
kmp_autotuning_info *__kmp_sched_autotunig_vector;
unsigned __KMP_NUM_AUTO_MODE = 100;

int64_t __kmp_end_max[256];

Autotuning *Autotuning::Create(int64_t min, int64_t max, unsigned ignore) {
  const int DIM = 1;
  const int MIN_ERROR = 1;

  Autotuning *at =
      static_cast<Autotuning *>(__kmp_allocate(sizeof(Autotuning)));

  at->p_point = NULL;
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

  if (p_optimizer->isEnd()) {
    p_point = p_optimizer->getMinPoint();
  }

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
    ++m_iter;
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

  // KMP_ASSERT(__KMP_NUM_AUTO_MODE > 0);

  __kmp_sched_autotunig_locations = static_cast<ident_t **>(
      __kmp_allocate(sizeof(ident_t *) * __KMP_NUM_AUTO_MODE));
  __kmp_sched_autotunig_locations_max = static_cast<int64_t *>(
      __kmp_allocate(sizeof(int64_t) * __KMP_NUM_AUTO_MODE));
  __kmp_sched_autotunig_vector = static_cast<kmp_autotuning_info *>(
      __kmp_allocate(sizeof(kmp_autotuning_info) * __KMP_NUM_AUTO_MODE));

  for (unsigned i = 0; i < __KMP_NUM_AUTO_MODE; i++) {
    __kmp_sched_autotunig_locations[i] = NULL;
  }
  for (unsigned i = 0; i < __KMP_NUM_AUTO_MODE; i++) {
    __kmp_sched_autotunig_locations_max[i] = 0;
  }

  for (unsigned i = 0; i < __KMP_NUM_AUTO_MODE; ++i) {
    TCW_SYNC_4(__kmp_sched_autotunig_vector[i].initialized, FALSE);
    TCW_SYNC_4(__kmp_sched_autotunig_vector[i].started, FALSE);
    TCW_SYNC_4(__kmp_sched_autotunig_vector[i].ended, TRUE);
    KMP_ATOMIC_ST_REL(&__kmp_sched_autotunig_vector[i].count, 0);
    __kmp_init_bootstrap_lock(&__kmp_sched_autotunig_vector[i].end_lock);
    __kmp_init_bootstrap_lock(&__kmp_sched_autotunig_vector[i].start_lock);
  }

  TCW_SYNC_4(__kmp_global_auto_initialized, TRUE);
  KMP_MB(); // Flush initialized

  __kmp_release_bootstrap_lock(&__kmp_initz_lock);
}

void __kmp_end_autotuning(int gtid, ident_t *loc) {
  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc, __kmp_end_max[gtid]);

  // printf("Ending autotuning in %p = %s\n", loc, loc->psource);

  if (info == NULL || !TCR_4(info->initialized) || info->at->isEnd())
    return;

  KMP_ASSERT2(info->at != NULL, "Autotuning was not initialized");

  int count = KMP_ATOMIC_ADD(&info->count, 1);
  if (count == TCR_4(__kmp_nth - 1)) {
    info->at->end();

    KMP_ATOMIC_ST_REL(&info->count, 0);
    TCW_SYNC_4(info->started, FALSE);
    TCW_SYNC_4(info->ended, TRUE);    
    KMP_MB();

    // __kmp_release_bootstrap_lock(&info->end_lock);
  }

  // if (TCR_4(info->ended))
  //   return;
  // __kmp_acquire_bootstrap_lock(&info->end_lock);
  // if (TCR_4(info->ended)) {
  //   __kmp_release_bootstrap_lock(&info->end_lock);
  //   return;
  // }
}

kmp_autotuning_info *__kmp_find_autotuning_info(ident_t *loc, int64_t max) {
  for (unsigned i = 0; i < __KMP_NUM_AUTO_MODE; i++) {
    if (__kmp_sched_autotunig_locations[i] == loc && __kmp_sched_autotunig_locations_max[i] == max)
      return __kmp_sched_autotunig_vector + i;
    else if (__kmp_sched_autotunig_locations[i] == NULL) {
      return NULL;
    }
  }
  return NULL;
}

kmp_autotuning_info *__kmp_create_autotuning_info(ident_t *loc, int64_t max) {
  
  printf("Creating new autotuning info in %p\n", loc);
  unsigned i = 0;
  for (; i < __KMP_NUM_AUTO_MODE; i++) {
    if (__kmp_sched_autotunig_locations[i] == NULL) {
      __kmp_sched_autotunig_locations[i] = loc;
      __kmp_sched_autotunig_locations_max[i] = max;
      return __kmp_sched_autotunig_vector + i;
    }
  }
  return NULL;
}