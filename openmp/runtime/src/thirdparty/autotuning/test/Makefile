# Harness standalone do Nelder-Mead. Não precisa do clang/libomp patchados.
#
#   make        -> compila
#   make test   -> roda tudo
#   make clean
#
# Os headers em shim/ substituem kmp.h / kmp_debug.h / kmp_os.h; por isso
# shim/ vem ANTES de .. no include path.

CXX      ?= c++
CXXFLAGS ?= -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter
SRCDIR    = ..
INCLUDES  = -Ishim -I$(SRCDIR)

OBJS = nm_harness.o NelderMead.o

nm_harness: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

HDRS = $(SRCDIR)/NelderMead.h $(SRCDIR)/NumericalOptimizer.h \
       shim/kmp.h shim/kmp_debug.h shim/kmp_os.h

nm_harness.o: nm_harness.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ nm_harness.cpp

# Compilado a partir da árvore real: o harness testa o código de produção.
NelderMead.o: $(SRCDIR)/NelderMead.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c -o $@ $(SRCDIR)/NelderMead.cpp

.PHONY: test clean
test: nm_harness
	./nm_harness

clean:
	rm -f $(OBJS) nm_harness
