/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksIterativeClosestPointWrapper.h"
#include "sksExceptionMacro.h"
#include <boost/python/extract.hpp>


namespace sks
{

//-----------------------------------------------------------------------------
double IterativeClosestPointWrapper(const np::ndarray& source,
                                    const np::ndarray& target,
                                    const unsigned int maxNumberOfIterations,
                                    const float maxCorrespondenceDistance,
                                    const float transformationEpsilon,
                                    const float fitnessEpsilon,
                                    np::ndarray& result
                                   )
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

  unsigned long int numberOfPoints = source.shape(0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointXYZ>);
  sourceCloud->points.resize(numberOfPoints);

  for (unsigned long int i = 0; i < numberOfPoints; i++)
  {
    sourceCloud->points[i].x = boost::python::extract<double>(source[i][0]);
    sourceCloud->points[i].y = boost::python::extract<double>(source[i][1]);
    sourceCloud->points[i].z = boost::python::extract<double>(source[i][2]);
  }

  numberOfPoints = target.shape(0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud (new pcl::PointCloud<pcl::PointXYZ>);
  targetCloud->points.resize(numberOfPoints);

  for (unsigned long int i = 0; i < numberOfPoints; i++)
  {
    targetCloud->points[i].x = boost::python::extract<double>(target[i][0]);
    targetCloud->points[i].y = boost::python::extract<double>(target[i][1]);
    targetCloud->points[i].z = boost::python::extract<double>(target[i][2]);
  }

  Eigen::Matrix4f finalTransform;

  double residual = sks::IterativeClosestPoint(sourceCloud,
                                               targetCloud,
                                               maxNumberOfIterations,
                                               maxCorrespondenceDistance,
                                               transformationEpsilon,
                                               fitnessEpsilon,
                                               finalTransform);

  double *resultPtr = reinterpret_cast<double*>(result.get_data());
  for (int r = 0; r < 4; ++r)
  {
    for (int c = 0; c < 4; c++)
    {
      resultPtr[r*4 + c] = finalTransform(r,c);
    }
  }
  return residual;
}

} // end namespace