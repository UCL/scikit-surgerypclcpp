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

  set(SKSURGERYPCLCPP_UNITY_NAME ${SKSURGERYPCLCPP_LIBRARY_NAME}Unity)

  set(SKSURGERYPCLCPP_UNITY_SRCS
    sksUnityWrapper.cpp
  )

  set(SKSURGERYPCLCPP_UNITY_HDRS
    sksUnityWrapper.h
  )

  # Thanks to: https://stackoverflow.com/questions/15120951/generating-a-bundle-file-with-cmake-on-mac-osx?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
  add_library(${SKSURGERYPCLCPP_UNITY_NAME} MODULE ${SKSURGERYPCLCPP_UNITY_HDRS} ${SKSURGERYPCLCPP_UNITY_SRCS})
  if(APPLE)
    set_target_properties(${SKSURGERYPCLCPP_UNITY_NAME} PROPERTIES BUNDLE TRUE)
  endif()

  target_link_libraries(${SKSURGERYPCLCPP_UNITY_NAME} PRIVATE ${SKSURGERYPCLCPP_LIBRARIES} ${ALL_THIRD_PARTY_LIBRARIES})
  install(TARGETS ${SKSURGERYPCLCPP_UNITY_NAME} DESTINATION ${SKSURGERYPCLCPP_INSTALL_LIB_DIR})

  if( NOT "${SKSURGERYPCLCPP_UNITY_PLUGIN_DIR}" STREQUAL "")
    install(TARGETS ${SKSURGERYPCLCPP_UNITY_NAME} DESTINATION ${SKSURGERYPCLCPP_UNITY_PLUGIN_DIR})
  endif()

endif()
