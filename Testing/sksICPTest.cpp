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
#include <sksIterativeClosestPoint.h>

#include <iostream>
#include <vector>

TEST_CASE( "Translation test", "[ICP]" ) {

  unsigned long int numberOfPoints = 3;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);
  sourceCloud->points.resize(numberOfPoints);

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZ>);
  targetCloud->points.resize(numberOfPoints);

  sourceCloud->points[0].x = 0;
  sourceCloud->points[0].y = 0;
  sourceCloud->points[0].z = 0;
  sourceCloud->points[1].x = 1;
  sourceCloud->points[1].y = 0;
  sourceCloud->points[1].z = 0;
  sourceCloud->points[2].x = 0;
  sourceCloud->points[2].y = 1;
  sourceCloud->points[2].z = 0;

  for (unsigned int i = 0; i < numberOfPoints; i++)
  {
    targetCloud->points[i].x = sourceCloud->points[i].x + 1;
    targetCloud->points[i].y = sourceCloud->points[i].y + 1;
    targetCloud->points[i].z = sourceCloud->points[i].z + 1;
  }

  Eigen::Matrix4f finalTransform;

  double residual = sks::IterativeClosestPoint(sourceCloud, targetCloud, finalTransform);
  std::cout << "ICP residual=" << residual << std::endl;
  std::cout << "ICP matrix=" << finalTransform << std::endl;
  REQUIRE(residual < 0.00001);
  REQUIRE(finalTransform(0, 3) == 1);
  REQUIRE(finalTransform(1, 3) == 1);
  REQUIRE(finalTransform(2, 3) == 1);
}

