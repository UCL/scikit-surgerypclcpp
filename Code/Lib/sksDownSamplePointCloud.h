/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksDownSamplePointCloud_h
#define sksDownSamplePointCloud_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sksWin32ExportHeader.h"

namespace sks {

SKSURGERYPCLCPP_WINEXPORT
const pcl::PointCloud<pcl::PointXYZ>::Ptr DownSamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                               float vx, float vy, float vz
                                                              );



} // end namespace

#endif