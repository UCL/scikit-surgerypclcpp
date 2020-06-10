/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksConversionUtils_h
#define sksConversionUtils_h

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/python/numpy.hpp>

namespace np = boost::python::numpy;

namespace sks
{
  void CheckInputIs3DFloat(const np::ndarray& input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertInputToPointCloud(const np::ndarray& input);

  np::ndarray ConvertPointCloudToNumpy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
}

#endif
