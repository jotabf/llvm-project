#include "kmp_autotuning.h"

#include <cmath>   // round
#include <cstring> // memcpy
#include <sstream>
#include <type_traits> // std::is_integral, std::is_floating_point

kmp_autotuning_info *__kmp_sched_autotunig_head = NULL;
// kmp_autotuning_info *__kmp_sched_autotunig_last_used = NULL;

double error_converter(unsigned error, int min, int max) {
  return (2. * static_cast<double>(error)) / static_cast<double>(max - min);
}

Autotuning *Autotuning::Create(double min, double max, unsigned ignore) {

  Autotuning *at =
      static_cast<Autotuning *>(__kmp_allocate(sizeof(Autotuning)));

  at->p_point = 0.0;
  // at->p_min = min;
  // at->p_max = max;
  at->m_ignore = ignore + 1;
  at->m_iter = 0;
  at->p_optimizer = NelderMead::Create(1);

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

void __kmp_end_autotuning(int gtid, unsigned cid) {
  printf("Ending autotuning %d\n", cid);

  kmp_autotuning_info *info = __kmp_find_autotuning_info(cid);
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

kmp_autotuning_info *__kmp_find_autotuning_info(unsigned cid) {
  auto it = __kmp_sched_autotunig_head;

  while (it != NULL) {
    if (it->id == cid) {
      return it;
    }
    it = it->next;
  }

  return NULL;
}

kmp_autotuning_info *__kmp_create_autotuning_info(unsigned cid) {

  kmp_autotuning_info *at_info = static_cast<kmp_autotuning_info *>(
      __kmp_allocate(sizeof(kmp_autotuning_info)));

  TCW_SYNC_4(at_info->initialized, FALSE);
  TCW_SYNC_4(at_info->release_start, FALSE);
  at_info->count = 0;
  __kmp_init_bootstrap_lock(&at_info->lock);
  at_info->at = Autotuning::Create(1, 1);;
  at_info->next = __kmp_sched_autotunig_head;
  __kmp_sched_autotunig_head = at_info;

  KMP_MB();
  at_info->id = cid;

  return at_info;
}