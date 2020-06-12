/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksFeatureWrapWrapper_h
#define sksFeatureWrapWrapper_h

#include <boost/python/numpy.hpp>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

/**
  @Brief Computes an initial rigid registration based on SIFT keypoints, FPFH descriptors, RANSAC to find a close match, and then SVD on matched points.
  @param normalSearchRadius radius to search to calculate surface normal for each input point.
  @param siftMinScale SIFT minimum scale. (TODO: what does this mean? see pcl::SIFTSIFTKeypoint)
  @param siftNumOctaves SIFT number of octaves/scales.  (TODO: what does this mean?)
  @param siftNumScalesPerOctave SIFT number of scales per octave.  (TODO: what does this mean?)
  @param siftMinContrast SIFT minimum contrast. (TODO: what does this mean?)
  @param siftKSearch SIFT k nearest neighbours to search.
  @param ransacInlierThreshold distance threshold to determine inliers when testing potential transformations.
  @param ransacMaximumIterations maximum iterations to test.
  @param result output 4x4 matrix
  @param transformedSource the transformed source data set. Must be pre-allocated.
*/
SKSURGERYPCLCPP_WINEXPORT void FeatureMatchWrapper(const np::ndarray& source,
                                                   const np::ndarray& target,
                                                   float normalSearchRadius,
                                                   float siftMinScale,
                                                   unsigned int siftNumOctaves,
                                                   unsigned int siftNumScalesPerOctave,
                                                   float siftMinContrast,
                                                   unsigned int siftKSearch,
                                                   float ransacInlierThreshold,
                                                   unsigned int ransacMaximumIterations,
                                                   np::ndarray& result,
                                                   np::ndarray& transformedSource
                                                   );
}
#endif
