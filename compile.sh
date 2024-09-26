#!/bin/sh

MODE=Debug

base_dir=$HOME/LLVM/build/$HOSTNAME/$MODE-f/
builddir=$base_dir/llvm
builddir_flang=$base_dir/flang
builddir_offload=$base_dir/offload
builddir_clang=$base_dir/clang
builddir_omp=$base_dir/omp
INSTALLDIR=$HOME/LLVM/install/$HOSTNAME/$MODE-f/
litpath=$(pwd)/utils/lit

export LD_LIBRARY_PATH=$INSTALLDIR/lib:$LD_LIBRARY_PATH

mkdir -p $builddir $builddir_offload $builddir_clang $builddir_omp $builddir_flang $INSTALLDIR
# rm -rfv $INSTALLDIR

clear

echo -e "\033[32m >> Installing LLVM\033[0m"
if [ "$1" = "--clear" ]; then rm -rfv $builddir; fi
cmake -G Ninja -S llvm -B $builddir \
      -DLLVM_INSTALL_UTILS=ON \
      -DLLVM_ENABLE_PROJECTS="clang;mlir;flang;openmp" \
      -DCMAKE_BUILD_TYPE=$MODE \
      -DLLVM_LIT_ARGS=-v \
      -DCMAKE_CXX_LINK_FLAGS="-Wl,-rpath,$LD_LIBRARY_PATH" \
      -DCMAKE_INSTALL_PREFIX=$INSTALLDIR || exit 1
ninja -C $builddir install || exit 1

# echo -e "\033[32m >> Installing Flang\033[0m"
# if [ "$1" = "--clear" ]; then rm -rfv $builddir_flang; fi
# cmake \
#   -G Ninja -S flang -B $builddir_flang \
#   -DCMAKE_BUILD_TYPE=Release \
#   -DCMAKE_CXX_STANDARD=17 \
#   -DCMAKE_CXX_LINK_FLAGS="-Wl,-rpath,$LD_LIBRARY_PATH" \
#   -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
#   -DFLANG_ENABLE_WERROR=ON \
#   -DLLVM_TARGETS_TO_BUILD=host \
#   -DLLVM_ENABLE_ASSERTIONS=ON \
#   -DLLVM_BUILD_MAIN_SRC_DIR=$builddir/lib/cmake/llvm \
#   -DLLVM_EXTERNAL_LIT=$builddir/bin/llvm-lit \
#   -DLLVM_LIT_ARGS=-v \
#   -DLLVM_DIR=$builddir/lib/cmake/llvm \
#   -DCLANG_DIR=$builddir/lib/cmake/clang \
#   -DMLIR_DIR=$builddir/lib/cmake/mlir || exit 1
# ninja -C $builddir_flang install || exit 1

# echo -e "\033[32m >> Installing Clang\033[0m"
# if [ "$1" = "--clear" ]; then rm -rfv $builddir_clang; fi
# cmake -G Ninja -S clang \
#       -B $builddir_clang \
#       -DLLVM_EXTERNAL_LIT=$litpath \
#       -DLLVM_INCLUDE_TESTS=OFF \
#       -DCMAKE_BUILD_TYPE=$MODE \
#       -DCMAKE_INSTALL_PREFIX=$INSTALLDIR \
#       -DLLVM_ROOT=$INSTALLDIR || exit 1
# ninja -C $builddir_clang install || exit 1

echo -e "\033[32m >> Installing OpenMP\033[0m"
if [ "$1" = "--clear" ]; then rm -rfv $builddir_omp; fi
cmake -G Ninja -S openmp \
      -B $builddir_omp \
      -DCMAKE_BUILD_TYPE=$MODE \
      -DLLVM_ROOT=$INSTALLDIR \
      -DCMAKE_INSTALL_PREFIX=$INSTALLDIR || exit 1
ninja -C $builddir_omp install || exit 1
