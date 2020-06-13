/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksGeneralizedIterativeClosestPointWrapper.h"
#include "sksExceptionMacro.h"
#include "sksConversionUtils.h"
#include <sksGeneralizedIterativeClosestPoint.h>
#include <boost/python/extract.hpp>


namespace sks
{

//-----------------------------------------------------------------------------
double GeneralizedIterativeClosestPointWrapper(const np::ndarray& source,
                                               const np::ndarray& target,
                                               float normalSearchRadius,
                                               float angleThresholdInDegreesForNormalCorrespondence,
                                               unsigned int maxNumberOfIterations,
                                               float maxCorrespondenceDistance,
                                               float transformationEpsilon,
                                               float fitnessEpsilon,
                                               np::ndarray& result,
                                               np::ndarray& transformedSource
                                               )
{
  CheckInputsForRegistration(source, target, transformedSource, result);

  pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud = ConvertInputToPointCloud(source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud = ConvertInputToPointCloud(target);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSourceCloud = ConvertInputToPointCloud(transformedSource);

  Eigen::Matrix4f finalTransform;

  double residual = sks::GeneralizedIterativeClosestPoint(sourceCloud,
                                                          targetCloud,
                                                          normalSearchRadius,
                                                          angleThresholdInDegreesForNormalCorrespondence,
                                                          maxNumberOfIterations,
                                                          maxCorrespondenceDistance,
                                                          transformationEpsilon,
                                                          fitnessEpsilon,
                                                          finalTransform,
                                                          transformedSourceCloud);

  ConvertEigenToNumpy(finalTransform, result);
  ConvertPointCloudToNumpy(transformedSourceCloud, transformedSource);

  return residual;
}

} // end namespace
