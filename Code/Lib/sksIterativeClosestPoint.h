/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksIterativeClosestPoint_h
#define sksIterativeClosestPoint_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "sksWin32ExportHeader.h"

namespace sks {

SKSURGERYPCLCPP_WINEXPORT double IterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                                       Eigen::Matrix4f& result);


} // end namespace

#endif