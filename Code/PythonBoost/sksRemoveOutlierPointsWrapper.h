/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksRemoveOutlierPointsWrapper_h
#define sksRemoveOutlierPointsWrapper_h

#include <boost/python/numpy.hpp>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

SKSURGERYPCLCPP_WINEXPORT np::ndarray RemoveOutlierPointsWrapper(const np::ndarray& input,
                                                                 float meanK,
                                                                 float stdDev
                                                                );
}
#endif
