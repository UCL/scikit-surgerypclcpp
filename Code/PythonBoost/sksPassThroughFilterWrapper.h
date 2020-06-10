/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksPassThroughFilterWrapper_h
#define sksPassThroughFilterWrapper_h

#include <boost/python/numpy.hpp>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

SKSURGERYPCLCPP_WINEXPORT np::ndarray PassThroughFilterWrapper(const np::ndarray& input,
                                                               char fieldName,
                                                               float minDistance,
                                                               float maxDistance,
                                                               bool insideInterval
                                                               );
}

#endif
