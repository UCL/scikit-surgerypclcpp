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
double IterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr fixed,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr moving,
                             Eigen::Matrix4f& result)
{
  return 0.0;
}

} // end namespace
