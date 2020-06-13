/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksMyFunctions.h"
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <iostream>

namespace sks {

//-----------------------------------------------------------------------------
int MyFirstAddFunction(int a, int b)
{
  return a + b;
}


//-----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
                                                      float normalSearchRadius)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr treeN(new pcl::search::KdTree<pcl::PointXYZ>());

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normalEstimation;
  normalEstimation.setInputCloud(input);
  normalEstimation.setSearchMethod(treeN);
  normalEstimation.setRadiusSearch(normalSearchRadius);

  pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
  normalEstimation.compute(*normals);

  // Copy the xyz info from input and add it to normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i < normals->points.size(); ++i)
  {
    normals->points[i].x = input->points[i].x;
    normals->points[i].y = input->points[i].y;
    normals->points[i].z = input->points[i].z;
  }

  return normals;
}

} // end namespace
