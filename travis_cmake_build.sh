#/*============================================================================
#
#  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

function cmake_build {
  echo "Starting travis_cmake_build.sh"
  pwd
  echo "Requested python version:${PYTHON_VERSION}"
  cmake --version
  python --version
  mkdir build
  cd build
  cmake -DSKSURGERYPCLCPP_PYTHON_VERSION:STRING=${PYTHON_VERSION} -DBUILD_SUPERBUILD:BOOL=ON -DBUILD_TESTING:BOOL=OFF ..
  make -j 2
  cd SKSURGERYPCLCPP-build
#  ctest .
  cd ../../
  echo "Finished travis_cmake_build.sh"
}
