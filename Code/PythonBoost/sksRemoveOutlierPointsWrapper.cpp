/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksRemoveOutlierPointsWrapper.h"
#include "sksExceptionMacro.h"
#include <sksRemoveOutliersFromPointCloud.h>
#include <boost/python/extract.hpp>


namespace sks
{

//-----------------------------------------------------------------------------
np::ndarray RemoveOutlierPointsWrapper(const np::ndarray& input,
                                       float meanK,
                                       float stdDev
                                      )
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

  unsigned long int numberOfPoints = input.shape(0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);
  sourceCloud->points.resize(numberOfPoints);

  for (unsigned long int i = 0; i < numberOfPoints; i++)
  {
    sourceCloud->points[i].x = boost::python::extract<double>(input[i][0]);
    sourceCloud->points[i].y = boost::python::extract<double>(input[i][1]);
    sourceCloud->points[i].z = boost::python::extract<double>(input[i][2]);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr result = sks::RemoveOutliersFromPointCloud(sourceCloud, meanK, stdDev);

  unsigned long int outputNumberOfPoints = result->size();
  np::ndarray output = np::zeros(boost::python::make_tuple(outputNumberOfPoints, 3),
                                 np::dtype::get_builtin<double>());

  for (unsigned long int i = 0; i < outputNumberOfPoints; i++)
  {
    output[i][0] = result->points[i].x;
    output[i][1] = result->points[i].y;
    output[i][2] = result->points[i].z;
  }

  return output;
}

} // end namespace