/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksIterativeClosestPoint.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

namespace sks {

//-----------------------------------------------------------------------------
double IterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                             unsigned int maxNumberOfIterations,
                             float maxCorrespondenceDistance,
                             float transformationEpsilon,
                             float fitnessEpsilon,
                             bool useLM,
                             Eigen::Matrix4f& result
                             )
{
  pcl::PointCloud<pcl::PointXYZ> Final;
  double residual = std::numeric_limits<double>::max();

  if (useLM)
  {
    pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(maxNumberOfIterations);
    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setTransformationEpsilon(transformationEpsilon);
    icp.setEuclideanFitnessEpsilon(fitnessEpsilon);
    icp.align(Final);
    result = icp.getFinalTransformation();
    residual = icp.getFitnessScore();
  }
  else
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations(maxNumberOfIterations);
    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setTransformationEpsilon(transformationEpsilon);
    icp.setEuclideanFitnessEpsilon(fitnessEpsilon);
    icp.align(Final);
    result = icp.getFinalTransformation();
    residual = icp.getFitnessScore();
  }

  return residual;
}

} // end namespace
