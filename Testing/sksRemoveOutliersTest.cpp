/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "catch.hpp"
#include "sksCatchMain.h"

#include "sksRemoveOutliersFromPointCloud.h"

TEST_CASE( "Remove Outliers Test", "[Outlier]" ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PCDReader reader;
  reader.read(sks::argv[1], *cloud);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  cloud_filtered = sks::RemoveOutliersFromPointCloud(cloud, 50, 1.0);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::io::savePCDFileASCII (sks::argv[2], *cloud_filtered);
  REQUIRE(cloud->size() == 460400);
  REQUIRE(cloud_filtered->size() == 451410);
}

