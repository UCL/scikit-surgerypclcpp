/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksFeatureWrapRANSACWrapper_h
#define sksFeatureWrapRANSACWrapper_h

#include <boost/python/numpy.hpp>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

SKSURGERYPCLCPP_WINEXPORT double FeatureMatchRANSACWrapper(const np::ndarray& source,
                                                           const np::ndarray& target,
                                                           float siftNormalSearchRadius,
                                                           float ransacInlierThreshold,
                                                           unsigned int ransacMaximumIterations,
                                                           float icpTransformationEpsilon,
                                                           unsigned int icpMaximumIterations,
                                                           np::ndarray& result,
                                                           np::ndarray& transformedSource
                                                           );
}
#endif
