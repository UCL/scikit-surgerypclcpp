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

macro(SKSURGERYPCLCPP_INSTALL_LIBRARY)

  set(ARGS ${ARGN})
  set(install_directories "")
  list(FIND ARGS DESTINATION _destination_index)
  if(_destination_index GREATER -1)
    message(SEND_ERROR "SKSURGERYPCLCPP_INSTALL_LIBRARAY macro must not be called with a DESTINATION parameter.")
  else()

    if(NOT BUILDING_GUIS)
        install(TARGETS ${ARGS}
                ARCHIVE DESTINATION ${SKSURGERYPCLCPP_INSTALL_LIB_DIR}
                LIBRARY DESTINATION ${SKSURGERYPCLCPP_INSTALL_LIB_DIR}
                RUNTIME DESTINATION ${SKSURGERYPCLCPP_INSTALL_BIN_DIR}
               )
     endif()

  endif()
endmacro()
