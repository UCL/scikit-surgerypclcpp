/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksPassThroughFilter_h
#define sksPassThroughFilter_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sksWin32ExportHeader.h"

namespace sks {

SKSURGERYPCLCPP_WINEXPORT
const pcl::PointCloud<pcl::PointXYZ>::Ptr PassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                            const char fieldName, float minDistance, float maxDistance, bool insideInterval);

} // end namespace

#endif