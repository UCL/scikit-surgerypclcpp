/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksQtVTKControllerWin32ExportHeader_h
#define sksQtVTKControllerWin32ExportHeader_h

/**
* \file sksQtVTKControllerWin32ExportHeader.h
* \brief Header to sort Windows dllexport/dllimport.
*/

#if (defined(_WIN32) || defined(WIN32)) && !defined(SKSURGERYPCLCPP_STATIC)
  #ifdef SKSURGERYPCLCPP_QTVTKCONTROLLER_WINDOWS_EXPORT
    #define SKSURGERYPCLCPP_QTVTKCONTROLLERWINEXPORT __declspec(dllexport)
  #else
    #define SKSURGERYPCLCPP_QTVTKCONTROLLERWINEXPORT __declspec(dllimport)
  #endif
#else
  #define SKSURGERYPCLCPP_QTVTKCONTROLLERWINEXPORT
#endif

#endif
