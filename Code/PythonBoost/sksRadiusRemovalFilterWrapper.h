/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksRadiusRemovalFilterWrapper_h
#define sksRadiusRemovalFilterWrapper_h

#include <boost/python/numpy.hpp>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

SKSURGERYPCLCPP_WINEXPORT np::ndarray RadiusRemovalFilterWrapper(const np::ndarray& input,
                                                                 float radius,
                                                                 unsigned int minNumberOfNeighbours
                                                                );
}

#endif
