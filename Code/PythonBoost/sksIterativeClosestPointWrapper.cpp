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
  CheckInputsForRegistration(source, target, transformedSource, result);

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

  ConvertEigenToNumpy(finalTransform, result);
  ConvertPointCloudToNumpy(transformedSourceCloud, transformedSource);

  return residual;
}

} // end namespace
