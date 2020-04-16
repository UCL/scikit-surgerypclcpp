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

set(SKSURGERYPCLCPP_LIBRARY_NAME sksurgerypclcpp)

set(SKSURGERYPCLCPP_LIBRARY_SRCS
  sksBasicTypes.cpp
  sksMyFunctions.cpp
  sksException.cpp
  sksIterativeClosestPoint.cpp
  sksDownSamplePointCloud.cpp
)

set(SKSURGERYPCLCPP_LIBRARY_HDRS
  sksExceptionMacro.h
)

add_library(${SKSURGERYPCLCPP_LIBRARY_NAME} ${SKSURGERYPCLCPP_LIBRARY_HDRS} ${SKSURGERYPCLCPP_LIBRARY_SRCS})

if(BUILD_SHARED_LIBS)
  if(WIN32)
    add_definitions(-DSKSURGERYPCLCPP_WINDOWS_EXPORT)
  endif(WIN32)
endif(BUILD_SHARED_LIBS)

target_link_libraries(${SKSURGERYPCLCPP_LIBRARY_NAME} ${ALL_THIRD_PARTY_LIBRARIES})

SKSURGERYPCLCPP_INSTALL_HEADERS()
SKSURGERYPCLCPP_INSTALL_LIBRARY(${SKSURGERYPCLCPP_LIBRARY_NAME})

