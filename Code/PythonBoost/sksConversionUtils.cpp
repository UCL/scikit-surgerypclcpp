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


//------------------------------------------------------------------------------
void CheckInputsForRegistration(const np::ndarray& source, const np::ndarray& target, const np::ndarray& transformedSource, np::ndarray& result)
{
  if (result.get_nd() != 2)
  {
    sksExceptionThrow() << "result matrix does not have 2 dimensions";
  }
  if (result.shape(0) != 4)
  {
    sksExceptionThrow() << "result matrix does not have 4 rows";
  }
  if (result.shape(1) != 4)
  {
    sksExceptionThrow() << "result matrix does not have 4 columns";
  }
  if (result.get_dtype() != np::dtype::get_builtin<double>())
  {
    sksExceptionThrow() << "result matrix is not float type";
  }
  if (source.get_nd() != 2)
  {
    sksExceptionThrow() << "source matrix does not have 2 dimensions";
  }
  if (source.shape(1) != 3)
  {
    sksExceptionThrow() << "source matrix does not have 3 columns";
  }
  if (source.shape(0) < 3)
  {
    sksExceptionThrow() << "source matrix has less than 3 rows";
  }
  if (source.get_dtype() != np::dtype::get_builtin<double>())
  {
    sksExceptionThrow() << "source matrix is not float type";
  }
  if (target.get_nd() != 2)
  {
    sksExceptionThrow() << "target matrix does not have 2 dimensions";
  }
  if (target.shape(1) != 3)
  {
    sksExceptionThrow() << "source matrix does not have 3 columns";
  }
  if (target.get_dtype() != np::dtype::get_builtin<double>())
  {
    sksExceptionThrow() << "target matrix is not float type";
  }
  if (transformedSource.get_nd() != 2)
  {
    sksExceptionThrow() << "transformedSource matrix does not have 2 dimensions";
  }
  if (transformedSource.shape(1) != 3)
  {
    sksExceptionThrow() << "transformedSource matrix does not have 3 columns";
  }
  if (transformedSource.get_dtype() != np::dtype::get_builtin<double>())
  {
    sksExceptionThrow() << "transformedSource matrix is not float type";
  }
  if (transformedSource.shape(0) != source.shape(0))
  {
    sksExceptionThrow() << "transformedSource does not have the same number of rows as source.";
  }
  if (transformedSource.shape(1) != source.shape(1))
  {
    sksExceptionThrow() << "transformedSource does not have the same number of columns as source.";
  }
}


//------------------------------------------------------------------------------
void ConvertEigenToNumpy(const Eigen::Matrix4f& input, np::ndarray& output)
{
  double *outputPtr = reinterpret_cast<double*>(output.get_data());
  for (int r = 0; r < 4; ++r)
  {
    for (int c = 0; c < 4; c++)
    {
      outputPtr[r*4 + c] = input(r,c);
    }
  }
}


//------------------------------------------------------------------------------
void ConvertPointCloudToNumpy(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, np::ndarray& output)
{
  unsigned long int outputNumberOfPoints = input->size();

  for (unsigned long int i = 0; i < outputNumberOfPoints; i++)
  {
    output[i][0] = input->points[i].x;
    output[i][1] = input->points[i].y;
    output[i][2] = input->points[i].z;
  }
}

} // end namespace
