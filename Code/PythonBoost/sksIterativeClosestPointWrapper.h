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
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

/**
  @brief Implementation of Iterative Closest Point (ICP) matching source to target.
  @param source [Nx3] source point cloud, where each row is x, y, z coordinates.
  @param target [Nx3] target point cloud, where each row is x, y, z coordinates.
  @param maxNumberOfIterations maximum number of iterations
  @param maxCorrespondenceDistance distance above which point matches are discared (in metres?).
  @param transformationEpsilon if the change in the transformation is below this threshold, the algorithm exits.
  @param fitnessEpsilon if the change in the cost function is below this threshold, the algorithm exits.
  @param useLM if true, does an LM-ICP variant, if false, normal ICP.
*/
SKSURGERYPCLCPP_WINEXPORT double IterativeClosestPointWrapper(const np::ndarray& source,
                                                              const np::ndarray& target,
                                                              unsigned int maxNumberOfIterations,
                                                              float maxCorrespondenceDistance,
                                                              float transformationEpsilon,
                                                              float fitnessEpsilon,
                                                              bool useLM,
                                                              np::ndarray& result,
                                                              np::ndarray& transformedSource
                                                              );
}
#endif
