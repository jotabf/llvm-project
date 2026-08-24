#!/bin/sh

MODE=Release

base_dir=$HOME/LLVM/build/$HOSTNAME/$MODE-c/
builddir=$base_dir/llvm
builddir_flang=$base_dir/flang
builddir_offload=$base_dir/offload
builddir_clang=$base_dir/clang
builddir_omp=$base_dir/omp
INSTALLDIR=$HOME/LLVM/install/$HOSTNAME/$MODE-c/
litpath=$(pwd)/utils/lit
CORES=$(($(nproc)/4))

export LD_LIBRARY_PATH=$INSTALLDIR/lib:$LD_LIBRARY_PATH

mkdir -p $builddir $builddir_offload $builddir_clang $builddir_omp $builddir_flang $INSTALLDIR
# rm -rfv $INSTALLDIR

clear

echo -e "\033[32m >> Installing LLVM\033[0m"
# if [ "$1" = "--clear" ]; then rm -rfv $builddir; fi
# cmake -G Ninja -S llvm -B $builddir \
#       -DLLVM_INSTALL_UTILS=ON \
#       -DLLVM_ENABLE_PROJECTS="clang;mlir;flang;openmp" \
#       -DCMAKE_BUILD_TYPE=$MODE \
#       -DLLVM_LIT_ARGS=-v \
#       -DCMAKE_CXX_LINK_FLAGS="-Wl,-rpath,$LD_LIBRARY_PATH" \
#       -DCMAKE_INSTALL_PREFIX=$INSTALLDIR || exit 1
# ninja -j$CORES -C $builddir install || exit 1

echo -e "\033[32m >> Installing OpenMP\033[0m"
# if [ "$1" = "--clear" ]; then rm -rfv $builddir_omp; fi
# cmake -G Ninja -S openmp \
#       -B $builddir_omp \
#       -DCMAKE_BUILD_TYPE=$MODE \
#       -DLLVM_ROOT=$INSTALLDIR \
#       -DCMAKE_INSTALL_PREFIX=$INSTALLDIR || exit 1
ninja -j$CORES -C $builddir_omp install || exit 1
