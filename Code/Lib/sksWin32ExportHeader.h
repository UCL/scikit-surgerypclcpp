/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksWin32ExportHeader_h
#define sksWin32ExportHeader_h

/**
* \file sksWin32ExportHeader.h
* \brief Header to sort Windows dllexport/dllimport.
*/

#if (defined(_WIN32) || defined(WIN32)) && !defined(SKSURGERYPCLCPP_STATIC)
  #ifdef SKSURGERYPCLCPP_WINDOWS_EXPORT
    #define SKSURGERYPCLCPP_WINEXPORT __declspec(dllexport)
  #else
    #define SKSURGERYPCLCPP_WINEXPORT __declspec(dllimport)
  #endif  /* SKSURGERYPCLCPP_WINEXPORT */
#else
/* linux/mac needs nothing */
  #define SKSURGERYPCLCPP_WINEXPORT
#endif

#endif
