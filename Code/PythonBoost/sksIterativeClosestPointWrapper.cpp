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
#include "sksConversionUtils.h"
#include <sksIterativeClosestPoint.h>
#include <boost/python/extract.hpp>


namespace sks
{

//-----------------------------------------------------------------------------
double IterativeClosestPointWrapper(const np::ndarray& source,
                                    const np::ndarray& target,
                                    unsigned int maxNumberOfIterations,
                                    float maxCorrespondenceDistance,
                                    float transformationEpsilon,
                                    float fitnessEpsilon,
                                    bool useLM,
                                    np::ndarray& result,
                                    np::ndarray& transformedSource
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud = ConvertInputToPointCloud(source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud = ConvertInputToPointCloud(target);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSourceCloud = ConvertInputToPointCloud(transformedSource);

  Eigen::Matrix4f finalTransform;

  double residual = sks::IterativeClosestPoint(sourceCloud,
                                               targetCloud,
                                               maxNumberOfIterations,
                                               maxCorrespondenceDistance,
                                               transformationEpsilon,
                                               fitnessEpsilon,
                                               useLM,
                                               finalTransform,
                                               transformedSourceCloud);

  double *resultPtr = reinterpret_cast<double*>(result.get_data());
  for (int r = 0; r < 4; ++r)
  {
    for (int c = 0; c < 4; c++)
    {
      resultPtr[r*4 + c] = finalTransform(r,c);
    }
  }

  unsigned long int outputNumberOfPoints = transformedSourceCloud->size();

  for (unsigned long int i = 0; i < outputNumberOfPoints; i++)
  {
    transformedSource[i][0] = transformedSourceCloud->points[i].x;
    transformedSource[i][1] = transformedSourceCloud->points[i].y;
    transformedSource[i][2] = transformedSourceCloud->points[i].z;
  }

  return residual;
}

} // end namespace
