/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksGeneralizedIterativeClosestPointWrapper_h
#define sksGeneralizedIterativeClosestPointWrapper_h

#include <boost/python/numpy.hpp>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

/**
  @brief Implementation of Generalized Iterative Closest Point (ICP) matching source to target.
  @param source [Nx3] source point cloud, where each row is x, y, z coordinates.
  @param target [Nx3] target point cloud, where each row is x, y, z coordinates.
  @param normalSearchRadius radius over which to search to compute surface normals.
  @param angleThresholdInDegreesForNormalCorrespondence angle in degrees between surface normals, above which, matches are discarded.
  @param maxNumberOfIterations maximum number of iterations in for main ICP loop.
  @param maxCorrespondenceDistance distance above which point matches are discarded.
  @param transformationEpsilon if the change in the transformation is below this threshold, the algorithm exits.
  @param fitnessEpsilon if the change in the cost function is below this threshold, the algorithm exits.
  @param result output 4x4 matrix, must be pre-allocated
  @param transformedSource transformed source, must be pre-allocated.
*/
SKSURGERYPCLCPP_WINEXPORT double GeneralizedIterativeClosestPointWrapper(const np::ndarray& source,
                                                                         const np::ndarray& target,
                                                                         float normalSearchRadius,
                                                                         float angleThresholdInDegreesForNormalCorrespondence,
                                                                         unsigned int maxNumberOfIterations,
                                                                         float maxCorrespondenceDistance,
                                                                         float transformationEpsilon,
                                                                         float fitnessEpsilon,
                                                                         np::ndarray& result,
                                                                         np::ndarray& transformedSource
                                                                         );
}
#endif
