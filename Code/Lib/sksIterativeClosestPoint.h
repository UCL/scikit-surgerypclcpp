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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "sksWin32ExportHeader.h"
#include <stdexcept>

namespace sks {

SKSURGERYPCLCPP_WINEXPORT double IterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                                       unsigned int maxNumberOfIterations,
                                                       float maxCorrespondenceDistance,
                                                       float transformationEpsilon,
                                                       float fitnessEpsilon,
                                                       bool useLM,
                                                       Eigen::Matrix4f& result
                                                       );


} // end namespace

#endif