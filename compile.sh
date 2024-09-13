#!/bin/sh

MODE=Release

builddir=$HOME/LLVM/build/$HOSTNAME/$MODE/llvm
builddir_flang=$HOME/LLVM/build/$HOSTNAME/$MODE/flang
builddir_offload=$HOME/LLVM/build/$HOSTNAME/$MODE/offload
builddir_clang=$HOME/LLVM/build/$HOSTNAME/$MODE/clang
builddir_omp=$HOME/LLVM/build/$HOSTNAME/$MODE/omp
installdir=$HOME/LLVM/install/$HOSTNAME/$MODE
litpath=$(pwd)/utils/lit

export LD_LIBRARY_PATH=$installdir/lib:$LD_LIBRARY_PATH

mkdir -p $builddir $builddir_offload $builddir_clang $builddir_omp $builddir_flang
mkdir -p $installdir
# rm -rfv $builddir $builddir_offload $builddir_clang $builddir_omp $builddir_flang
# rm -rfv $installdir

clear

echo -e "\033[32m >> Installing LLVM\033[0m"
cmake -G Ninja -S llvm -B $builddir \
      -DLLVM_INSTALL_UTILS=ON \
      -DCMAKE_BUILD_TYPE=$MODE \
      -DLLVM_ENABLE_PROJECTS="clang;mlir;flang;openmp" \
      -DCMAKE_INSTALL_PREFIX=$installdir || exit 1
ninja -C $builddir install || exit 1

echo -e "\033[32m >> Installing Clang\033[0m"
cmake -G Ninja -S clang \
      -B $builddir_clang \
      -DLLVM_EXTERNAL_LIT=$litpath \
      -DLLVM_INCLUDE_TESTS=OFF \
      -DCMAKE_BUILD_TYPE=$MODE \
      -DCMAKE_INSTALL_PREFIX=$installdir \
      -DLLVM_ROOT=$installdir || exit 1
ninja -C $builddir_clang install || exit 1

# echo -e "\033[32m >> Installing Offload\033[0m"
# cmake -G Ninja -S offload -B $builddir_offload \
#       -DCMAKE_BUILD_TYPE=$MODE \
#       -DLLVM_EXTERNAL_LIT=$litpath \
#       -DLLVM_ROOT=$installdir \
#       -DCMAKE_C_COMPILER=$installdir/bin/clang \
#       -DCMAKE_CXX_COMPILER=$installdir/bin/clang++ \
#       -DCMAKE_INSTALL_PREFIX=$installdir || exit 1
# ninja -C $builddir_offload install || exit 1

echo -e "\033[32m >> Installing OpenMP\033[0m"
cmake -G Ninja -S openmp \
      -B $builddir_omp \
      -DCMAKE_BUILD_TYPE=$MODE \
      -DLLVM_ROOT=$installdir \
      -DCMAKE_INSTALL_PREFIX=$installdir || exit 1
ninja -C $builddir_omp install || exit 1

#
#
#
# cmake -G Ninja -S llvm -B build-llvm -DLLVM_INSTALL_UTILS=ON -DLLVM_EXTERNAL_LIT=$HOME/utils/lit -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=install
# ninja -C build-llvm install
# cmake -G Ninja -S clang -B build-clang -DLLVM_INCLUDE_TESTS=OFF -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=install -DLLVM_ROOT=install
# ninja -C build-clang install
# cmake -G Ninja -S openmp -B build-omp -DCMAKE_BUILD_TYPE=Debug -DLLVM_ROOT=install -DCMAKE_INSTALL_PREFIX=install
# ninja -C build-omp install
 
# export LD_LIBRARY_PATH=/home/joao.fernandes/miniconda3/lib/:$LD_LIBRARY_PATH
# export LD_LIBRARY_PATH=/home/joao.fernandes/miniconda3/envs/llvm/x86_64-conda-linux-gnu/sysroot/usr/lib64/:$LD_LIBRARY_PATH
# export LD_LIBRARY_PATH=/home/joao.fernandes/miniconda3/envs/llvm/lib/:$LD_LIBRARY_PATH