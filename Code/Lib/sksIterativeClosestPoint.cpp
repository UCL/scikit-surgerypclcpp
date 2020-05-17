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

namespace sks {

//-----------------------------------------------------------------------------
double IterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                             Eigen::Matrix4f& result)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  result = icp.getFinalTransformation();
  double residual = icp.getFitnessScore();

  return residual;
}

} // end namespace
