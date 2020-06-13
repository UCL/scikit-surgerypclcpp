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
#include <sksFeatureMatch.h>
#include <iostream>
#include <vector>

TEST_CASE( "Feature Match test", "[Feature Match]" ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read(sks::argv[1], *sourceCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceFiltered (new pcl::PointCloud<pcl::PointXYZ>());
  sourceFiltered = sks::DownSamplePointCloud(sourceCloud, 0.05, 0.05, 0.05);

  std::cout << "Loaded:" << sks::argv[1] << ", which downsampled to:" << sourceFiltered->size() << " points." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read(sks::argv[2], *targetCloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetFiltered (new pcl::PointCloud<pcl::PointXYZ>());
  targetFiltered = sks::DownSamplePointCloud(targetCloud, 0.05, 0.05, 0.05);

  std::cout << "Loaded:" << sks::argv[2] << ", which downsampled to:" << targetFiltered->size() << " points." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSourceCloud (new pcl::PointCloud<pcl::PointXYZ>);
  transformedSourceCloud->points.resize(sourceFiltered->size());

  Eigen::Matrix4f initialTransform;
  initialTransform.setIdentity();
  initialTransform(0, 3) = -1;
  initialTransform(1, 3) = -2;
  initialTransform(2, 3) = 3;

  typename pcl::PointCloud<pcl::PointXYZ>::Ptr sourceTransformed(new pcl::PointCloud<pcl::PointXYZ>);
  sourceTransformed->points.resize(sourceFiltered->points.size());

  pcl::transformPointCloud(*sourceFiltered, *sourceTransformed, initialTransform);

  Eigen::Matrix4f finalTransform;

  sks::FeatureMatch(sourceTransformed,
                    targetFiltered,
                    0.1,                     // Normal radius search.
                    0.1,                     // SIFT min scale
                    3,                       // SIFT num octaves
                    4,                       // SIFT num scales per octave
                    0.0,                     // SIFT min contrast
                    20,                      // SIFT KSearch
                    0.05,                    // RANSAC inlier threshold.
                    1000,                    // RANSAC iterations
                    finalTransform,
                    transformedSourceCloud);

  std::cout << "FeatureMatch matrix=" << finalTransform << std::endl;

  REQUIRE(std::fabs(finalTransform(0, 3) - -initialTransform(0, 3)) < 0.001);
  REQUIRE(std::fabs(finalTransform(1, 3) - -initialTransform(1, 3)) < 0.001);
  REQUIRE(std::fabs(finalTransform(2, 3) - -initialTransform(2, 3)) < 0.001);
}

