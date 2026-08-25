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

/// ---------------------------------------------------------------------------
/// Profiler opcional do caminho quente (KMP_AT_PROFILE=1).
///
/// Existe para ATRIBUIR custo por trecho: sem isto, as diferencas em jogo
/// (dezenas a centenas de ns por execucao) ficam abaixo do ruido de uma
/// medicao ponta-a-ponta, e nao da para dizer qual otimizacao rendeu o que.
/// Desligado, o custo e' o teste de um int global -- previsivel e fora do
/// caminho de dados.
/// ---------------------------------------------------------------------------
struct kmp_at_prof {
  KMP_ALIGN_CACHE
  kmp_uint64 start_cyc, start_n; ///< __kmp_start_autotuning inteiro
  kmp_uint64 bar_cyc, bar_n;     ///< so a barreira dentro dele
  kmp_uint64 end_cyc, end_n;     ///< __kmp_end_autotuning inteiro
  kmp_uint64 find_cyc, find_n;   ///< buscas na tabela
  kmp_uint64 find_steps;         ///< slots visitados pelas buscas
};

#define KMP_AT_PROF_MAX_THREADS 1024

/// ---------------------------------------------------------------------------
/// Interruptor de otimizacoes (KMP_AT_OPT, mascara de bits; default: tudo on).
///
/// Serve para comparar A/B no MESMO binario, na mesma maquina, na mesma
/// rodada. Comparar builds diferentes em maquina compartilhada mistura o
/// efeito da mudanca com a deriva da maquina -- foi o que impediu de isolar o
/// custo da varredura linear na primeira tentativa.
///
/// O teste e' um branch sobre um global quente, previsto em praticamente 100%
/// das vezes, e vale para os dois lados da comparacao.
/// ---------------------------------------------------------------------------
#define KMP_AT_OPT_CACHE (1 << 0)  ///< cache (loc,info) por thread
#define KMP_AT_OPT_FROZEN (1 << 1) ///< caminho rapido via info->frozen/chunk
#define KMP_AT_OPT_KEYS (1 << 2)   ///< indice hash em vez de varredura linear
/// SO' PARA MEDICAO: pula a barreira de __kmp_start_autotuning. E' INCORRETO
/// com nowait -- serve para medir o teto do ganho de elimina-la, e e' seguro
/// apenas quando o loop ja' tem barreira implicita (o caso sem nowait).
#define KMP_AT_OPT_NOBAR (1 << 3)
/// Mede o custo com KMP_NOW() (TSC em x86, clock_gettime fora) em vez de
/// __kmp_elapsed(). DESLIGADO POR PADRAO -- medido, nao suposto.
///
/// __kmp_elapsed usa gettimeofday (resolucao 1 us) e devolve segundos
/// ABSOLUTOS da epoca num double, cujo ulp em ~1.79e9 s ja' e' 238 ns; o
/// relogio e' de fato grosseiro. Mas a variacao INTRINSECA de uma execucao do
/// loop medida com clock_gettime e' de 13% (loop de 198 us) a 31% (loop de
/// 1.6 us) -- ou seja, o ruido real do loop domina a quantizacao do relogio em
/// tudo que nao seja minusculo.
///
/// Ligar isto media explore/base PIOR de forma consistente (1.16 -> 1.29 e
/// 2.21 -> 2.76): com resolucao mais fina o Nelder-Mead passa a distinguir
/// pontos que so' diferem por ruido, e explora por mais iteracoes. O caminho
/// para atacar esse ruido e' MEDIA entre execucoes (m_ignore > 1), nao um
/// relogio melhor.
#define KMP_AT_OPT_NSEC (1 << 4)
#define KMP_AT_OPT_ALL (KMP_AT_OPT_CACHE | KMP_AT_OPT_FROZEN | KMP_AT_OPT_KEYS)

extern int __kmp_at_opt;

/// ---------------------------------------------------------------------------
/// KMP_AT_FORCE=1: autotuna todo loop schedule(dynamic) do programa, mesmo sem
/// a anotacao schedule(dynamic, auto).
///
/// Existe porque o front-end NAO e' o unico produtor de loops OpenMP: o flang
/// e o MLIR passam pelo OMPIRBuilder, que nao tem a cláusula "auto" do clang.
/// Sem este interruptor, medir autotuning em codigo Fortran (o NAS) exigiria
/// implementar a cláusula no flang primeiro. Com ele, basta compilar
/// normalmente e ligar a variavel na hora de rodar -- e o binario continua
/// servindo de baseline com KMP_AT_FORCE ausente, no MESMO executavel.
///
/// Escopo deliberado: SO' kmp_sch_dynamic_chunked. E' o unico schedule em que
/// o chunk tem o significado que o modelo de custo do Nelder-Mead assume (a
/// unidade de trabalho entregue por pedido). Em guided o chunk e' um piso, em
/// static ele nem chega ao dispatch. Alargar e' uma linha em
/// __kmp_at_force_applies().
/// ---------------------------------------------------------------------------
extern int __kmp_at_force;

/// Lido em __kmp_do_serial_initialize, antes de qualquer worksharing, para o
/// guard do caminho quente ser um teste de int e nao um getenv.
void __kmp_autotuning_env_initialize(void);

/// TRUE se KMP_AT_FORCE deve ligar o autotuning para \p schedule.
///
/// Aceita as duas codificacoes de dynamic NAO ordenado que chegam aqui:
/// kmp_sch_dynamic_chunked (35) e kmp_nm_dynamic_chunked (163, a variante
/// "nomerge"). As formas ORDENADAS (67 e 195) ficam de fora de proposito --
/// ordered serializa a entrega das iteracoes, e ajustar o chunk ali nao mede o
/// que o experimento quer medir.
static inline int __kmp_at_force_applies(enum sched_type schedule) {
  if (!__kmp_at_force)
    return FALSE;
  const enum sched_type s =
      SCHEDULE_WITHOUT_MODIFIERS(SCHEDULE_WITHOUT_MODE(schedule));
  return s == kmp_sch_dynamic_chunked || s == kmp_nm_dynamic_chunked;
}

extern int __kmp_at_profile;
extern kmp_at_prof *__kmp_at_prof_tab;

/// Contadores desta thread, ou NULL se o profiler esta desligado.
static inline kmp_at_prof *__kmp_at_prof_get(int gtid) {
  if (!__kmp_at_profile || __kmp_at_prof_tab == NULL || gtid < 0 ||
      gtid >= KMP_AT_PROF_MAX_THREADS)
    return NULL;
  return &__kmp_at_prof_tab[gtid];
}

/// Cronometra um escopo. RAII para nao ter de instrumentar cada `return`.
struct kmp_at_prof_scope {
  kmp_uint64 t0;
  kmp_uint64 *p_cyc;
  kmp_uint64 *p_n;
  kmp_at_prof_scope(kmp_uint64 *cyc, kmp_uint64 *n) : t0(0), p_cyc(cyc), p_n(n) {
    if (p_cyc)
      t0 = KMP_NOW();
  }
  ~kmp_at_prof_scope() {
    if (p_cyc) {
      *p_cyc += KMP_NOW() - t0;
      ++*p_n;
    }
  }
};

#define KMP_AT_PROF_SCOPE(prof, field)                                         \
  kmp_at_prof_scope __at_scope_##field((prof) ? &(prof)->field##_cyc : NULL,    \
                                       (prof) ? &(prof)->field##_n : NULL)

struct kmp_autotuning_info {
  KMP_ALIGN_CACHE
  volatile int initialized = FALSE;
  volatile int started = FALSE;
  volatile int ended = FALSE;
  /// Chunk publicado da instancia corrente, e "o otimizador ja' terminou".
  ///
  /// Existem para o caminho quente nao ter de perseguir ponteiros: ler
  /// info->at->isEnd() e info->at->getPoint() sao CINCO loads dependentes
  /// (info->at, at->p_optimizer, opt->m_step, at->p_point, p_point[0]) em tres
  /// alocacoes diferentes -- ate' tres faltas de cache. Aqui sao dois loads na
  /// mesma linha. A fonte da verdade continua sendo o otimizador; estes campos
  /// sao uma copia publicada sob info->start_lock.
  volatile int frozen = FALSE;
  volatile int64_t chunk = 1;
  KMP_ALIGN_CACHE
  kmp_bootstrap_lock_t start_lock;
  std::atomic<int> count = 0;
  ident_t *loc = NULL; ///< chave: localização de origem do loop
  Autotuning *at = NULL; ///< NULL = autotuning desabilitado neste loop
};

/// Garante a entrada de \p loc e devolve o ponteiro dela. Devolver o info em
/// vez de void poupa uma varredura da tabela em __kmp_start_autotuning.
template <typename T>
kmp_autotuning_info *__kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub);

template <typename T> T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub);

void __kmp_autotuning_global_initialize(int gtid);

/// Libera a tabela e imprime o relatório final. Chamada de __kmp_cleanup().
void __kmp_autotuning_global_cleanup(void);

void __kmp_end_autotuning(int gtid, ident_t *loc);

/// Devolve a entrada de \p loc, ou NULL se ainda não existe.
kmp_autotuning_info *__kmp_find_autotuning_info(ident_t *loc);

/// Devolve a entrada de \p loc, criando-a se preciso. NULL se a tabela encheu.
kmp_autotuning_info *__kmp_get_autotuning_info(ident_t *loc);

/// Cache de uma entrada por thread, com a chave verificada.
///
/// As tres chamadas de uma execucao (init, start, end) procuravam o MESMO loc
/// na mesma tabela linear, tres varreduras O(K). Guardar o par (loc, info) da
/// ultima consulta desta thread transforma as duas seguintes em uma comparacao
/// de ponteiro. A chave e' conferida, entao o cache nunca pode devolver a
/// entrada errada -- no pior caso erra e cai na varredura.
struct kmp_at_cache_ent {
  KMP_ALIGN_CACHE
  ident_t *loc;
  kmp_autotuning_info *info;
};

#define KMP_AT_CACHE_SLOTS 1024

extern kmp_at_cache_ent *__kmp_at_cache;

static inline kmp_autotuning_info *__kmp_at_cache_get(int gtid, ident_t *loc) {
  if (!(__kmp_at_opt & KMP_AT_OPT_CACHE))
    return NULL;
  if (__kmp_at_cache == NULL || gtid < 0 || gtid >= KMP_AT_CACHE_SLOTS)
    return NULL;
  kmp_at_cache_ent *e = &__kmp_at_cache[gtid];
  return e->loc == loc ? e->info : NULL;
}

static inline void __kmp_at_cache_put(int gtid, ident_t *loc,
                                      kmp_autotuning_info *info) {
  if (!(__kmp_at_opt & KMP_AT_OPT_CACHE))
    return;
  if (__kmp_at_cache == NULL || gtid < 0 || gtid >= KMP_AT_CACHE_SLOTS)
    return;
  kmp_at_cache_ent *e = &__kmp_at_cache[gtid];
  e->info = info;
  e->loc = loc; // chave por ultimo: quem le confere a chave antes de usar info
}

///@brief Class for Autotuning
class Autotuning {

  int64_t *p_point; ///< Point in the search space
  unsigned m_ignore; ///< Number of iterations to ignore
  unsigned m_iter; ///< Iteration number

  NelderMead *p_optimizer; ///< Numerical optimizer instance

  double m_t0; ///< Instante de início, em segundos (caminho __kmp_elapsed)
  kmp_uint64 m_tick0; ///< Instante de início em ticks (caminho KMP_NOW)
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
kmp_autotuning_info *__kmp_init_autotuning(int gtid, ident_t *loc, T lb, T ub) {

  __kmp_autotuning_global_initialize(gtid);

  kmp_autotuning_info *info = __kmp_at_cache_get(gtid, loc);
  if (info == NULL) {
    info = __kmp_get_autotuning_info(loc);
    if (info == NULL) // tabela cheia: segue sem autotuning
      return NULL;
    __kmp_at_cache_put(gtid, loc, info);
  }

  if (TCR_4(info->initialized))
    return info;
  __kmp_acquire_bootstrap_lock(&info->start_lock);
  if (TCR_4(info->initialized)) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return info;
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
    return info;
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
  return info;
}

template <typename T> T __kmp_start_autotuning(int gtid, ident_t *loc, T lb, T ub) {
  // __KMP_NUM_AUTO_MODE é um símbolo FRACO: se o executável não o define, o
  // endereço resolve para 0 e ler o VALOR seria um deref nulo. O teste do
  // ENDEREÇO é o marcador "este binário tem loop anotado com auto".
  //
  // KMP_AT_FORCE tem de furar esse marcador: quem produz o binário no caminho
  // Fortran é o flang, que não emite a global (só o clang a emite, e só nas TUs
  // com pelo menos um loop "auto"). Sem esta exceção o modo forçado sairia aqui
  // e não faria nada -- silenciosamente.
  //
  // Sem loc não há chave possível, e aí não há o que fazer nem forçado.
  if (loc == NULL || (&__KMP_NUM_AUTO_MODE == nullptr && !__kmp_at_force))
    return 1;

  kmp_at_prof *prof = __kmp_at_prof_get(gtid);
  KMP_AT_PROF_SCOPE(prof, start);

  // __kmp_init_autotuning ja' devolve a entrada; antes havia aqui uma segunda
  // varredura da tabela (__kmp_find_autotuning_info) pelo mesmo loc.
  kmp_autotuning_info *info = __kmp_init_autotuning(gtid, loc, lb, ub);

  if (info == NULL)
    return 1;

  // Caminho quente do loop ja' convergido: dois loads na mesma linha, sem
  // perseguir ponteiro nenhum. Depois que frozen e' publicado o chunk nao muda
  // mais, entao nao ha' o que sincronizar.
  if ((__kmp_at_opt & KMP_AT_OPT_FROZEN) && TCR_4(info->frozen))
    return static_cast<T>(info->chunk);

  // Ordem importa: estas saídas baratas vêm ANTES de pegar o start_lock.
  // info->at == NULL significa "autotuning desabilitado para este loop", e
  // initialized sozinho NÃO prova que info->at existe.
  if (!TCR_4(info->initialized) || info->at == NULL)
    return 1;

  if (info->at->isEnd())
    return (__kmp_at_opt & KMP_AT_OPT_FROZEN)
               ? static_cast<T>(info->chunk)
               : static_cast<T>(info->at->getPoint());

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
  if (!(__kmp_at_opt & KMP_AT_OPT_NOBAR)) {
    KMP_AT_PROF_SCOPE(prof, bar);
    __kmp_barrier(bs_plain_barrier, gtid, FALSE, 0, NULL, NULL);
  }

  if (TCR_4(info->started))
    return (__kmp_at_opt & KMP_AT_OPT_FROZEN)
               ? static_cast<T>(info->chunk)
               : static_cast<T>(info->at->getPoint());

  __kmp_acquire_bootstrap_lock(&info->start_lock);
  if (TCR_4(info->started)) {
    __kmp_release_bootstrap_lock(&info->start_lock);
    return static_cast<T>(info->chunk);
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

  // Publica o chunk (e o "congelou") ANTES de started: quem le started==TRUE
  // le info->chunk em seguida e tem de ver o valor desta instancia.
  info->chunk = info->at->getPoint();
  if (info->at->isEnd())
    TCW_SYNC_4(info->frozen, TRUE);
  KMP_MB();

  TCW_SYNC_4(info->started, TRUE);
  TCW_SYNC_4(info->ended, FALSE);
  KMP_MB();

  __kmp_release_bootstrap_lock(&info->start_lock);

  KA_TRACE(50, ("__kmp_start_autotuning: T#%d chunk %lld for %s in range "
                "(%lld,%lld).\n",
                gtid, (long long)info->chunk,
                loc ? loc->psource : "?", (long long)min, (long long)max));

  return static_cast<T>(info->chunk);
}

#endif // KMP_AUTOTUNING_H
