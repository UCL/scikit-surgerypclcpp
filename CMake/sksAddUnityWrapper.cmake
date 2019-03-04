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

option(BUILD_UNITY_WRAPPER "Build Unity Wrapper." OFF)
mark_as_advanced(BUILD_UNITY_WRAPPER)

if(BUILD_UNITY_WRAPPER AND BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build Shared Libraries" FORCE)
  message("Forcing BUILD_SHARED_LIBS to OFF as you want a Unity module.")
endif()

if(BUILD_UNITY_WRAPPER)
  set(SKSURGERYPCLCPP_UNITY_PLUGIN_DIR "" CACHE PATH "Path to Unity project Assets dir")
  if( "${SKSURGERYPCLCPP_UNITY_PLUGIN_DIR}" STREQUAL "")
    message(FATAL_ERROR "If you turn BUILD_UNITY_WRAPPER, you must specify SKSURGERYPCLCPP_UNITY_PLUGIN_DIR.")
  endif()
endif()
