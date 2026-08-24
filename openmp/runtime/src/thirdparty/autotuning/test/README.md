# Banco de testes do Nelder-Mead (standalone)

Exercita `../NelderMead.cpp` **fora da libomp**, com um modelo sintético de
custo de chunk size. Não precisa do clang nem da libomp patchados — compila com
qualquer C++17:

```sh
make test
```

Os headers em `shim/` substituem `kmp.h`, `kmp_debug.h` e `kmp_os.h`, fornecendo
só os quatro símbolos que o `NelderMead.cpp` usa: `__kmp_allocate` (via `calloc`,
porque `___kmp_allocate_align` zera o bloco), `__kmp_free`, `KMP_ASSERT2` e
`KMP_BUILTIN_UNREACHABLE`. O objeto `NelderMead.o` é compilado a partir da
árvore real, então o teste sempre acompanha o código de produção.

## Modelo de custo

```
cost(x) = overhead / x  +  imbalance * x
```

Unimodal e convexo, com mínimo em `x* = sqrt(overhead / imbalance)`. O primeiro
termo modela o custo de despacho (`N/x` operações), o segundo o desbalanceamento
na cauda (proporcional ao chunk). Isso dá um ótimo *conhecido* contra o qual
medir a convergência.

## Cenários

| Cenário | Intervalo | Ótimo | O que exercita |
|---|---|---|---|
| `typical` | `[1, 6250]` | 250 | caso comum: N=100k, 8 threads |
| `small-range` | `[1, 64]` | 16 | intervalo curto |
| `tiny-range` | `[1, 4]` | 2 | intervalo menor que o simplex de 3 pontos |
| `offset-min` | `[101, 200]` | 150 | `min != 1` (inner loop de `distribute`) |
| `noisy` | `[1, 6250]` | 250 | medição com ruído de ±10% |
| `duplicate-seed` | — | — | `srand(time(NULL))` dentro de `Create()` |
| `degenerate` | — | — | `setLimits(min, min)` → `% 0` |

Cada cenário roda num **processo filho** (`fork`), com `alarm(15)` de watchdog.
Assim um `SIGFPE`, `SIGSEGV` ou loop infinito num cenário não esconde os demais:
o pai relata o sinal e segue.

## Critérios de falha

Um cenário de busca falha se:

- devolver algum ponto `<= 0` (chunk inválido);
- devolver algum ponto fora de `[min, max]`;
- não convergir (`isEnd()`) em 400 iterações;
- terminar a mais de 35% do ótimo real;
- disparar algum `KMP_ASSERT2` dentro do NelderMead.

## Estado conhecido

Com o `rand_gen()` corrigido (`% (m_max - m_min + 1) + m_min`), os cinco
cenários de busca passam. Continuam falhando, por bugs ainda abertos:

- **`degenerate`** → `SIGFPE`. `__kmp_start_autotuning` recalcula `min`/`max` a
  cada execução do loop e chama `setLimits()` sem checar `min < max`;
  `circ_mod()`/`rand_gen()` então fazem `% (m_max - m_min)` com módulo zero.
- **`duplicate-seed`** → dois otimizadores criados no mesmo segundo recebem o
  mesmo simplex inicial, porque `NelderMead::Create` chama `srand(time(NULL))`
  (que de quebra destrói a semente global da aplicação). Use um estado de PRNG
  por otimizador.

`noisy` converge, mas com erro bem maior que os demais (~20% contra <1%) — o
simplex fecha em cima de ruído. Se as medições reais forem ruidosas, vale
considerar média de várias execuções por ponto (é para isso que existe o
`m_ignore`, hoje sempre 1).
