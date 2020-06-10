/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include "sksCatchMain.h"
#include "pcl/io/ply_io.h"
#include <sksFeatureMatchRANSAC.h>
#include <iostream>
#include <vector>

TEST_CASE( "Feature Match test", "[RANSAC]" ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile<pcl::PointXYZ>(sks::argv[1], *sourceCloud);
  std::cout << "Loaded:" << sks::argv[1] << ", which has:" << sourceCloud->size() << " points." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPLYFile<pcl::PointXYZ>(sks::argv[2], *targetCloud);
  std::cout << "Loaded:" << sks::argv[2] << ", which has:" << targetCloud->size() << " points." << std::endl;

  Eigen::Matrix4f finalTransform;

  double residual = sks::FeatureMatchRANSAC(sourceCloud,
                                            targetCloud,
                                            finalTransform);
  std::cout << "FeatureMatch residual=" << residual << std::endl;
  std::cout << "FeatureMatch matrix=" << finalTransform << std::endl;
  REQUIRE(residual < 0.00001);
  REQUIRE(finalTransform(0, 3) == 1);
  REQUIRE(finalTransform(1, 3) == 1);
  REQUIRE(finalTransform(2, 3) == 1);
}

