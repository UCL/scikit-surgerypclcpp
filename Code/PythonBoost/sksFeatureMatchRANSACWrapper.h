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

SKSURGERYPCLCPP_WINEXPORT void FeatureMatchRANSACWrapper(const np::ndarray& source,
                                                         const np::ndarray& target,
                                                         float siftNormalSearchRadius,
                                                         float siftMinScale,
                                                         unsigned int siftNumOctaves,
                                                         unsigned int siftNumScalesPerOctave,
                                                         float siftMinContrast,
                                                         int siftKSearch,
                                                         float ransacInlierThreshold,
                                                         unsigned int ransacMaximumIterations,
                                                         np::ndarray& result,
                                                         np::ndarray& transformedSource
                                                         );
}
#endif
