#include "kmp_autotuning.h"

#include <cmath>   // round
#include <cstring> // memcpy
#include <sstream>
#include <type_traits> // std::is_integral, std::is_floating_point

volatile bool __kmp_global_auto_initialized = FALSE;
kmp_autotuning_info *__kmp_sched_autotunig_vector;

static double error_converter(unsigned error, int min, int max) {
  return (2. * static_cast<double>(error)) / static_cast<double>(max - min);
}

Autotuning *Autotuning::Create(int min, int max, unsigned ignore) {

  Autotuning *at =
      static_cast<Autotuning *>(__kmp_allocate(sizeof(Autotuning)));

  const int DIM = 1;
  const int MIN_ERROR = 1;

  at->p_point = 0.0;
  // at->p_min = min;
  // at->p_max = max;
  at->m_ignore = ignore + 1;
  at->m_iter = 0;
  at->p_optimizer =
      NelderMead::Create(DIM, error_converter(MIN_ERROR, min, max));

  return at;
}

void Autotuning::start() {
  // End execution
  if (p_optimizer->isEnd())
    return;

  if ((m_iter % m_ignore) == 0) {
    double *value = p_optimizer->run(m_runtime);
    p_point = value[0];
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

void __kmp_init_autotuning(int gtid, unsigned id) {

  printf("Initializing autotuning %d GOMP %d\n", id, __KMP_NUM_AUTO_MODE);

  __kmp_autotuning_global_initialize();

  auto info = __kmp_find_autotuning_info(id);

  KMP_ASSERT2(info != NULL, "Sched Autotuning info was not initialized");

  if (TCR_4(info->initialized))
    return;
  __kmp_acquire_bootstrap_lock(&__kmp_initz_lock);
  if (TCR_4(info->initialized)) {
    __kmp_release_bootstrap_lock(&__kmp_initz_lock);
    return;
  }

  info->at = Autotuning::Create(1, 1);

  KMP_MB(); // Flush initialized
  TCW_SYNC_4(info->initialized, TRUE);

  __kmp_release_bootstrap_lock(&__kmp_initz_lock);
}

void __kmp_end_autotuning(int gtid, unsigned id) {
  printf("Ending autotuning %d\n", id);

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