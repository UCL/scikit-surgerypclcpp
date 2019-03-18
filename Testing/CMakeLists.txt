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

set(DATA_DIR ${CMAKE_SOURCE_DIR}/Testing/Data)
set(TMP_DIR ${CMAKE_BINARY_DIR}/Testing/Temporary)

set(TEST_CASES
  sksCommandLineArgsTest
  sksICPTest
)

foreach(_test_case ${TEST_CASES})
  add_executable(${_test_case} ${_test_case}.cpp sksCatchMain.cpp)
  target_link_libraries(${_test_case} ${SKSURGERYPCLCPP_LIBRARY})
  target_link_libraries(${_test_case} ${PCL_LIBRARIES})
endforeach()


if (SKSURGERYPCLCPP_USE_CUDA)
  set(CUDA_TEST_CASES
    mpCudaTest
  )
  foreach(_test_case ${CUDA_TEST_CASES})
    cuda_add_executable(${_test_case} ${_test_case}.cu sksCatchMain.cpp)
    target_link_libraries(${_test_case} ${ALL_LIBRARIES})
  endforeach()
endif()

# Adding tests is done outside the above loop,
# as each test will need different arguments.

##################################################################################################
# Dont forget its:  add_test(<test name (globally unique) > <exe name> <argument1> <argument2>
##################################################################################################

add_test(1File ${EXECUTABLE_OUTPUT_PATH}/sksCommandLineArgsTest ${DATA_DIR}/input.txt)
add_test(ICP ${EXECUTABLE_OUTPUT_PATH}/sksICPTest)
if (SKSURGERYPCLCPP_USE_CUDA)
  add_test(Cuda ${EXECUTABLE_OUTPUT_PATH}/mpCudaTest)
endif()
