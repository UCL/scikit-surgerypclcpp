/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksIterativeClosestPointWrapper_h
#define sksIterativeClosestPointWrapper_h

#include <boost/python/numpy.hpp>
#include <sksIterativeClosestPoint.h>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

SKSURGERYPCLCPP_WINEXPORT double IterativeClosestPointWrapper(const np::ndarray& source,
                                                              const np::ndarray& target,
                                                              unsigned int maxNumberOfIterations,
                                                              float maxCorrespondenceDistance,
                                                              float transformationEpsilon,
                                                              float fitnessEpsilon,
                                                              np::ndarray& result
                                                              );
}
#endif
