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
#include "sksDownSamplePointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <sksGeneralizedIterativeClosestPoint.h>
#include <iostream>
#include <vector>

TEST_CASE( "Generalized Iterative Closest Point", "[GICP]" ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read(sks::argv[1], *sourceCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceFiltered (new pcl::PointCloud<pcl::PointXYZ>());
  sourceFiltered = sks::DownSamplePointCloud(sourceCloud, 0.01, 0.01, 0.01);

  std::cout << "Loaded:" << sks::argv[1] << ", which downsampled to:" << sourceFiltered->size() << " points." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read(sks::argv[2], *targetCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetFiltered (new pcl::PointCloud<pcl::PointXYZ>());
  targetFiltered = sks::DownSamplePointCloud(targetCloud, 0.01, 0.01, 0.01);

  std::cout << "Loaded:" << sks::argv[2] << ", which downsampled to:" << targetFiltered->size() << " points." << std::endl;

  Eigen::Matrix4f initialTransform;
  initialTransform.setIdentity();
  initialTransform(0, 3) = -0.001;
  initialTransform(1, 3) = -0.002;
  initialTransform(2, 3) = 0.003;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceOffset(new pcl::PointCloud<pcl::PointXYZ>);
  sourceOffset->points.resize(sourceFiltered->points.size());
  pcl::transformPointCloud(*sourceFiltered, *sourceOffset, initialTransform);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSourceCloud (new pcl::PointCloud<pcl::PointXYZ>);
  transformedSourceCloud->points.resize(sourceFiltered->points.size());

  Eigen::Matrix4f finalTransform;

  double residual = sks::GeneralizedIterativeClosestPoint(sourceOffset,
                                                          targetFiltered,
                                                          0.02,   // Normal search radius
                                                          45,     // Angle threshold when checking correspondences
                                                          1000,   // ICP max iterations
                                                          0.1,    // ICP max correspondence distance
                                                          0.0001, // ICP transformation epsilon
                                                          0.0001, // ICP cost function epsilon
                                                          finalTransform,
                                                          transformedSourceCloud
                                                          );

  std::cout << "GICP matrix=" << finalTransform << std::endl;
  std::cout << "GICP residual=" << residual << std::endl;

  REQUIRE(std::fabs(finalTransform(0, 3) - -initialTransform(0, 3)) < 0.001);
  REQUIRE(std::fabs(finalTransform(1, 3) - -initialTransform(1, 3)) < 0.001);
  REQUIRE(std::fabs(finalTransform(2, 3) - -initialTransform(2, 3)) < 0.001);
}

