//===-- kmp_autotuning.h - Scheduling autotuning mode management -*- C++-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file contains the declaration of autotuning functions that are used in
/// scheduling runtime to auto adjust the chunk size when selected the auto
/// mode.
///
/// Cada loop "auto" é identificado pelo ponteiro do seu ident_t (a localização
/// de origem que o clang já emite, uma global estática por call site), NÃO por
/// um id inteiro gerado em tempo de compilação. O id inteiro exigia que o
/// front-end soubesse o total de loops "auto" do programa, o que só funciona
/// num único arquivo-fonte: com mais de uma TU os contadores reiniciam em 1 e
/// colidem, e a global __KMP_NUM_AUTO_MODE vira definição forte duplicada.
///
//===----------------------------------------------------------------------===//

#ifndef KMP_AUTOTUNING_H
#define KMP_AUTOTUNING_H

#include "kmp.h"
#include "thirdparty/autotuning/NelderMead.h"
#include "thirdparty/autotuning/NumericalOptimizer.h"

#include <atomic>
#include <cmath> // log2, pow (usados em __kmp_autotuning_guess)

class Autotuning;

/// Emitida pelo clang (fraca) nas TUs que têm pelo menos um loop "auto".
/// Só o ENDEREÇO é usado, como "este binário tem autotuning"; o valor não
/// significa mais nada. Ler o valor de um símbolo fraco indefinido seria um
/// deref nulo.
extern __attribute__((weak)) const unsigned __KMP_NUM_AUTO_MODE;

/// Capacidade da tabela de loops "auto". Ajustável por KMP_AUTOTUNING_SLOTS.
#define KMP_AUTOTUNING_DEFAULT_SLOTS 256

struct kmp_autotuning_info {
  KMP_ALIGN_CACHE
  volatile int initialized = FALSE;
  volatile int started = FALSE;
  volatile int ended = FALSE;
  KMP_ALIGN_CACHE
  kmp_bootstrap_lock_t start_lock;
  std::atomic<int> count = 0;
  ident_t *loc = NULL; ///< chave: localização de origem do loop
  Autotuning *at = NULL; ///< NULL = autotuning desabilitado neste loop
};

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T> T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub);

void __kmp_autotuning_global_initialize(int gtid);

/// Libera a tabela e imprime o relatório final. Chamada de __kmp_cleanup().
void __kmp_autotuning_global_cleanup(void);

void __kmp_end_autotuning(int gtid, ident_t *loc);

/// Devolve a entrada de \p loc, ou NULL se ainda não existe.
kmp_autotuning_info *__kmp_find_autotuning_info(ident_t *loc);

/// Devolve a entrada de \p loc, criando-a se preciso. NULL se a tabela encheu.
kmp_autotuning_info *__kmp_get_autotuning_info(ident_t *loc);

///@brief Class for Autotuning
class Autotuning {

  int64_t *p_point; ///< Point in the search space
  unsigned m_ignore; ///< Number of iterations to ignore
  unsigned m_iter; ///< Iteration number

  NelderMead *p_optimizer; ///< Numerical optimizer instance

  double m_t0; ///< Instante de início, em segundos de relógio de parede
  double m_runtime; ///< Total time of a task

public:
  // Deleted constructors and assignment operators to prevent copying and moving
  auto operator=(Autotuning &&) -> Autotuning & = delete;
  auto operator=(Autotuning) -> Autotuning = delete;
  Autotuning(const Autotuning &) = delete;
  Autotuning(Autotuning &&) = delete;
  Autotuning() = delete;
  ~Autotuning() = delete;

  ///@brief Parameterized constructor
  ///@param min Minimum value of the search interval
  ///@param max Maximum value of the search interval
  ///@param ignore Number of iterations to ignore
  static Autotuning *Create(int64_t min, int64_t max, unsigned ignore = 0);

  ///@brief Destructor. Libera também o otimizador -- antes ele vazava.
  static void Destroy(Autotuning *at) {
    if (at == NULL)
      return;
    NelderMead::Destroy(at->p_optimizer);
    __kmp_free(at);
  }

  ///@brief Get the point in the search space
  ///@param i Index of the point
  ///@return The point in the search space
  int64_t getPoint(int i = 0) const {
    KMP_ASSERT(p_point != NULL);
    return p_point[i];
  }

  unsigned getIter() const { return m_iter; }

  ///@brief Último custo medido, em segundos. Só para o relatório final.
  double getRuntime() const { return m_runtime; }

  ///@brief Start a new iteration of the autotuning algorithm
  void start();

  ///@brief End the current iteration of the autotuning algorithm
  void end();

  ///@brief Check if the optimization has reached the end
  bool isEnd() const { return p_optimizer->isEnd(); }

  ///@brief Semeia um vértice do simplex inicial com um palpite.
  void setPoint(int64_t v, unsigned id, unsigned dim = 0) {
    p_optimizer->setPoint(v, id, dim);
  }

  ///@brief Set the limits of the search interval
  ///@param min Minimum value of the search interval
  ///@param max Maximum value of the search interval
  void setLimits(int64_t min, int64_t max) { p_optimizer->setLimits(min, max); }

  ///@brief Reset the autotuning and numerical optimizer
  ///@param level Reset level, depending on the Optimizer
  void reset(unsigned level);
};

/// Palpite inicial para o chunk, em vez de sortear os três vértices.
/// Portado do caminho -fortran: parte de trip_count/(2*nth) e recua por um
/// fator log2 amortecido pela razão áurea, o que dá um chunk moderado --
/// grande o bastante para diluir o custo de despacho, pequeno o bastante para
/// sobrar cauda para balancear.
static inline int64_t __kmp_autotuning_guess(int64_t ninter, int nth,
                                             int64_t min, int64_t max) {
  if (ninter <= 0 || nth <= 0)
    return min;
  const double ratio = static_cast<double>(ninter) / static_cast<double>(nth);
  if (ratio <= 1.0)
    return min;
  const double factor = log2(ratio) * (1.0 / 1.618);
  int64_t point =
      static_cast<int64_t>(ninter / (pow(2.0, factor) * 2.0 * nth));
  if (point < min)
    point = min;
  if (point > max)
    point = max;
  return point;
}

template <typename T>
void __kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {

  __kmp_autotuning_global_initialize(gtid);

  kmp_autotuning_info *info = __kmp_get_autotuning_info(loc);
  if (info == NULL) // tabela cheia: segue sem autotuning
    return;

  if (TCR_4(info->initialized))
    return;
  __kmp_acquire_bootstrap_lock(&info->start_lock);
  if (TCR_4(info->initialized)) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return;
  }

  const int nth = TCR_4(__kmp_nth);
  // lb/ub aqui são os limites NORMALIZADOS do clang (0 .. trip_count-1), então
  // ub + 1 é a contagem de iterações. min = lb + 1 e não lb: chunk 0 é
  // inválido.
  int64_t min = static_cast<int64_t>(lb + 1);
  int64_t max = static_cast<int64_t>((ub + 1) / static_cast<T>(nth * 2));

  if (min >= max) {
    // Intervalo degenerado: desabilita o autotuning para este loop, mas marca
    // initialized para não refazer esta tentativa a cada execução. info->at
    // continua NULL e é ESSE o sinal de "desabilitado" consumido por
    // __kmp_start_autotuning / __kmp_end_autotuning.
    TCW_SYNC_4(info->initialized, TRUE);
    KMP_MB();
    __kmp_release_bootstrap_lock(&info->start_lock);
    return;
  }

  Autotuning *at = Autotuning::Create(min, max);

  // Um dos três vértices começa no palpite; os outros dois continuam
  // aleatórios, para o simplex ter área.
  at->setPoint(__kmp_autotuning_guess(static_cast<int64_t>(ub) + 1, nth, min,
                                      max),
               0);

  info->at = at;
  TCW_SYNC_4(info->started, FALSE);

  // A publicação de info->at tem de ser ordenada ANTES do store de
  // initialized, senão em ARM outra thread vê initialized=TRUE com at ainda
  // NULL. TCW_SYNC_4 é store volátil puro, sem fence.
  KMP_MB();
  TCW_SYNC_4(info->initialized, TRUE);
  KMP_MB();

  KA_TRACE(20, ("__kmp_init_autotuning: T#%d initialized autotuning for %s in "
                "a range of (%lld,%lld).\n",
                gtid, loc ? loc->psource : "?", (long long)min, (long long)max));

  __kmp_release_bootstrap_lock(&info->start_lock);
}

template <typename T> T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub) {
  // __KMP_NUM_AUTO_MODE é um símbolo FRACO: se o executável não o define, o
  // endereço resolve para 0 e ler o VALOR seria um deref nulo.
  // Sem loc não há chave possível.
  if (&__KMP_NUM_AUTO_MODE == nullptr || loc == NULL)
    return 1;

  __kmp_init_autotuning(gtid, loc, lb, ub);

  kmp_autotuning_info *info = __kmp_find_autotuning_info(loc);

  // Ordem importa: estas saídas baratas vêm ANTES de pegar o start_lock.
  // info->at == NULL significa "autotuning desabilitado para este loop", e
  // initialized sozinho NÃO prova que info->at existe.
  if (info == NULL || !TCR_4(info->initialized) || info->at == NULL)
    return 1;

  if (info->at->isEnd())
    return info->at->getPoint();

  // Barreira portada do caminho -fortran. Sem ela, uma thread que corre à
  // frente pode ler info->started depois que a thread lenta da execução
  // ANTERIOR o zerou em __kmp_end_autotuning, sortear um ponto novo e executar
  // a mesma instância do loop com um chunk diferente do das colegas. Em
  // dynamic isso não é só perda de desempenho: o contador de iterações é
  // compartilhado e o chunk é privado (pr->u.p.parm1), então chunks
  // divergentes duplicam e perdem iterações.
  //
  // É seguro aqui porque todas as threads do time encontram o mesmo construto
  // de worksharing, e esta chamada acontece no topo de
  // __kmp_dispatch_init/__kmp_for_static_init, antes de qualquer estado de
  // dispatch ser tocado.
  __kmp_barrier(bs_plain_barrier, gtid, FALSE, 0, NULL, NULL);

  if (TCR_4(info->started))
    return info->at->getPoint();

  __kmp_acquire_bootstrap_lock(&info->start_lock);
  if (TCR_4(info->started)) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return info->at->getPoint();
  }

  int64_t min = static_cast<int64_t>(lb + 1);
  int64_t max =
      static_cast<int64_t>((ub + 1) / static_cast<T>(TCR_4(__kmp_nth) * 2));

  if (min >= max) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return 1;
  }

  info->at->setLimits(min, max);
  info->at->start();

  TCW_SYNC_4(info->started, TRUE);
  TCW_SYNC_4(info->ended, FALSE);
  KMP_MB();

  __kmp_release_bootstrap_lock(&info->start_lock);

  KA_TRACE(50, ("__kmp_start_autotuning: T#%d chunk %lld for %s in range "
                "(%lld,%lld).\n",
                gtid, (long long)info->at->getPoint(),
                loc ? loc->psource : "?", (long long)min, (long long)max));

  return info->at->getPoint();
}

#endif // KMP_AUTOTUNING_H
