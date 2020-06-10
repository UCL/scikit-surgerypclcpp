/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksFeatureMatchRANSAC_h
#define sksFeatureMatchRANSAC_h

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "sksWin32ExportHeader.h"
#include <stdexcept>

namespace sks {

SKSURGERYPCLCPP_WINEXPORT double FeatureMatchRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                                    Eigen::Matrix4f& result
                                                    );


} // end namespace

#endif