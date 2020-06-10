/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksFeatureMatchRANSAC.h"
#include <pcl/keypoints/harris_3d.h>

namespace sks {

//-----------------------------------------------------------------------------
double FeatureMatchRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                          Eigen::Matrix4f& result
                          )
{

  pcl::PointCloud<pcl::PointXYZ> Final;
  double residual = std::numeric_limits<double>::max();

  return residual;
}

} // end namespace
