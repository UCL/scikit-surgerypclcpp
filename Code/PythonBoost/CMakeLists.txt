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

if(NOT BUILDING_GUIS)

  if (NOT SKSURGERYPCLCPP_PYTHON_MODULE_NAME OR SKSURGERYPCLCPP_PYTHON_MODULE_NAME STREQUAL "")
    set(SKSURGERYPCLCPP_PYTHON_MODULE_NAME "sksurgerypclpython" CACHE STRING "Basename for python module." FORCE)
  endif()

  set(SKSURGERYPCLCPP_PYTHON_SRCS)

  list(APPEND SKSURGERYPCLCPP_PYTHON_SRCS
    sksConversionUtils.cpp
    sksIterativeClosestPointWrapper.cpp
    sksDownSamplePointsWrapper.cpp
    sksRemoveOutlierPointsWrapper.cpp
    sksPassThroughFilterWrapper.cpp
    sksRadiusRemovalFilterWrapper.cpp
    sksFeatureMatchWrapper.cpp
    sksGeneralizedIterativeClosestPointWrapper.cpp
    sksLibPython.cpp
  )

  if (WIN32)
    if(NOT BUILD_SHARED)
      add_definitions(-DBOOST_PYTHON_STATIC_LIB)
      add_definitions(-DBOOST_NUMPY_STATIC_LIB)
    endif()
  endif()

  PYTHON_ADD_MODULE(${SKSURGERYPCLCPP_PYTHON_MODULE_NAME} ${SKSURGERYPCLCPP_PYTHON_SRCS})
  target_link_libraries(${SKSURGERYPCLCPP_PYTHON_MODULE_NAME} ${ALL_LIBRARIES})
  if(APPLE)
    set_target_properties(${SKSURGERYPCLCPP_PYTHON_MODULE_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
  endif()

  install(TARGETS ${SKSURGERYPCLCPP_PYTHON_MODULE_NAME} DESTINATION ${SKSURGERYPCLCPP_INSTALL_LIB_DIR})

endif()
