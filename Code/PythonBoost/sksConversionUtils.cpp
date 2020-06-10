/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksConversionUtils.h"
#include "sksExceptionMacro.h"

namespace sks
{

//------------------------------------------------------------------------------
void CheckInputIs3DFloat(const np::ndarray& input)
{
  if (input.get_nd() != 2)
  {
    sksExceptionThrow() << "input matrix does not have 2 dimensions";
  }
  if (input.shape(1) != 3)
  {
    sksExceptionThrow() << "input matrix does not have 3 columns";
  }
  if (input.get_dtype() != np::dtype::get_builtin<double>())
  {
    sksExceptionThrow() << "input matrix is not float type";
  }
}


//------------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertInputToPointCloud(const np::ndarray& input)
{
  unsigned long int numberOfPoints = input.shape(0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.resize(numberOfPoints);

  for (unsigned long int i = 0; i < numberOfPoints; i++)
  {
    cloud->points[i].x = boost::python::extract<double>(input[i][0]);
    cloud->points[i].y = boost::python::extract<double>(input[i][1]);
    cloud->points[i].z = boost::python::extract<double>(input[i][2]);
  }

  return cloud;
}


//------------------------------------------------------------------------------
np::ndarray ConvertPointCloudToNumpy(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  unsigned long int outputNumberOfPoints = cloud->size();
  np::ndarray output = np::zeros(boost::python::make_tuple(outputNumberOfPoints, 3),
                                 np::dtype::get_builtin<double>());

  for (unsigned long int i = 0; i < outputNumberOfPoints; i++)
  {
    output[i][0] = cloud->points[i].x;
    output[i][1] = cloud->points[i].y;
    output[i][2] = cloud->points[i].z;
  }

  return output;
}

} // end namespace
