#!/bin/sh

builddir=$DATA/LLVM/build/llvm
installdir=$DATA/LLVM/install/llvm

builddir_clang=$DATA/LLVM/build/clang
installdir_clang=$DATA/LLVM/install/clang
litpath=$DATA/utils/lit

builddir_omp=$DATA/LLVM/build/omp
installdir_omp=$DATA/LLVM/install/omp

mkdir -p $builddir $builddir_clang $builddir_omp
mkdir -p $installdir $installdir_clang $installdir_omp

# rm -rfv $builddir $builddir_clang $builddir_omp 
# rm -rfv $installdir $installdir_clang $installdir_omp

# . /vol0004/apps/oss/spack/share/spack/setup-env.sh
# spack load gcc@13.2.0%gcc@8.5.0 arch=linux-rhel8-skylake_avx512
# spack load ninja@1.11.1%gcc@13.2.0 arch=linux-rhel8-cascadelake
# spack load cmake@3.27.7%gcc@13.2.0 arch=linux-rhel8-cascadelake
# spack load /so5pyv6 #python@3.11.6%gcc@13.2.0 

clear

# echo -e "\033[32m >> Installing LLVM\033[0m"
# cmake -G Ninja -S llvm -B $builddir \
#       -DLLVM_INSTALL_UTILS=ON \
#       -DLLVM_ENABLE_ASSERTIONS=ON \
#       -DCMAKE_BUILD_TYPE=Release \
#       -DCMAKE_INSTALL_PREFIX=$installdir || exit 1   
# ninja -C $builddir install || exit 1

# echo -e "\033[32m >> Installing Clang\033[0m"
# cmake -G Ninja -S clang \
#       -B $builddir_clang \
#       -DLLVM_EXTERNAL_LIT=$litpath \
#       -DLLVM_INCLUDE_TESTS=OFF \
#       -DCMAKE_BUILD_TYPE=Release \
#       -DCMAKE_INSTALL_PREFIX=$installdir_clang \
#       -DLLVM_ENABLE_ASSERTIONS=ON \
#       -DCMAKE_CXX_FLAGS="-lstdc++" \
#       -DLLVM_ROOT=$installdir || exit 1
# ninja -C $builddir_clang install || exit 1

echo -e "\033[32m >> Installing OpenMP\033[0m"
# cmake -G Ninja -S openmp \
#       -B $builddir_omp \
#       -DCMAKE_BUILD_TYPE=Release \
#       -DLLVM_ROOT=$installdir \
#       -DCMAKE_INSTALL_PREFIX=$installdir_omp || exit 1
ninja -C $builddir_omp || exit 1
# ninja -C $builddir_omp install || exit 1