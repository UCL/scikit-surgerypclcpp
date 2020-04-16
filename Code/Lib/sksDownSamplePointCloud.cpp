/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksDownSamplePointCloud.h"

#include <pcl/filters/voxel_grid.h>

namespace sks {

//-----------------------------------------------------------------------------
const pcl::PointCloud<pcl::PointXYZ>::Ptr DownSamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                               float vx, float vy, float vz
                                                               )
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(vx, vy, vz);
  filter.filter(*output);
  return output;
}

} // end namespace
