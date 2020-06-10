/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksIterativeClosestPoint.h"

namespace sks {

//-----------------------------------------------------------------------------
double IterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                             unsigned int maxNumberOfIterations,
                             float maxCorrespondenceDistance,
                             float transformationEpsilon,
                             float fitnessEpsilon,
                             Eigen::Matrix4f& result
                             )
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(maxNumberOfIterations);
  icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
  icp.setTransformationEpsilon(transformationEpsilon);
  icp.setEuclideanFitnessEpsilon(fitnessEpsilon);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  result = icp.getFinalTransformation();
  double residual = icp.getFitnessScore();

  return residual;
}

} // end namespace
