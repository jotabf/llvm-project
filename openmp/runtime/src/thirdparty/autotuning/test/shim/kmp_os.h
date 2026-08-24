// Shim mínimo de "kmp_os.h".
#ifndef KMP_SHIM_KMP_OS_H
#define KMP_SHIM_KMP_OS_H

// A libomp define isto como __builtin_unreachable(). Aqui NÃO usamos
// __builtin_unreachable: quando KMP_ASSERT2 é não-fatal (modo relatório do
// harness) o fluxo continua, e um unreachable de verdade viraria UB.
#define KMP_BUILTIN_UNREACHABLE ((void)0)

#endif // KMP_SHIM_KMP_OS_H
