/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksGeneralizedIterativeClosestPoint_h
#define sksGeneralizedIterativeClosestPoint_h

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "sksWin32ExportHeader.h"
#include <stdexcept>

namespace sks {

SKSURGERYPCLCPP_WINEXPORT double GeneralizedIterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                                                  float normalSearchRadius,
                                                                  float angleThresholdInDegreesForNormalCorrespondence,
                                                                  unsigned int icpMaxNumberOfIterations,
                                                                  float icpMaxCorrespondenceDistance,
                                                                  float icpTransformationEpsilon,
                                                                  float icpFitnessEpsilon,
                                                                  Eigen::Matrix4f& result
                                                                  );


} // end namespace

#endif