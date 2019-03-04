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

configure_file(${CMAKE_SOURCE_DIR}/Usesksurgerypclcpp.cmake.in ${CMAKE_BINARY_DIR}/Usesksurgerypclcpp.cmake @ONLY IMMEDIATE)
configure_file(${CMAKE_SOURCE_DIR}/sksurgerypclcppConfig.cmake.in ${CMAKE_BINARY_DIR}/sksurgerypclcppConfig.cmake @ONLY IMMEDIATE)
if(NOT BUILDING_GUIS)
  install(FILES ${CMAKE_BINARY_DIR}/Usesksurgerypclcpp.cmake DESTINATION . COMPONENT CONFIG)
  install(FILES ${CMAKE_BINARY_DIR}/sksurgerypclcppConfig.cmake DESTINATION . COMPONENT CONFIG)
endif()
