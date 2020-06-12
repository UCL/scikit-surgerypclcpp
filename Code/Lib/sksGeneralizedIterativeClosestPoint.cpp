/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksGeneralizedIterativeClosestPoint.h"
#include "sksMyFunctions.h"
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <math.h>

namespace sks {

//-----------------------------------------------------------------------------
double GeneralizedIterativeClosestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                        float normalSearchRadius,
                                        float angleThresholdInDegreesForNormalCorrespondence,
                                        unsigned int icpMaxNumberOfIterations,
                                        float icpMaxCorrespondenceDistance,
                                        float icpTransformationEpsilon,
                                        float icpFitnessEpsilon,
                                        Eigen::Matrix4f& result
                                        )
{
  #define Scalar float
  double residual = std::numeric_limits<double>::max();

  pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals = computeNormals(source, normalSearchRadius);
  pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals = computeNormals(target, normalSearchRadius);

  pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal, Scalar>::Ptr transformationEstimation(new pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal, Scalar>);

  pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr corSurfNorm (new pcl::registration::CorrespondenceRejectorSurfaceNormal);
  corSurfNorm->setThreshold(acos(M_PI * angleThresholdInDegreesForNormalCorrespondence/180.0));
  corSurfNorm->initializeDataContainer<pcl::PointNormal, pcl::PointNormal>();
  corSurfNorm->setInputSource<pcl::PointNormal>(sourceNormals);
  corSurfNorm->setInputNormals<pcl::PointNormal, pcl::PointNormal>(sourceNormals);
  corSurfNorm->setInputTarget<pcl::PointNormal>(targetNormals);
  corSurfNorm->setTargetNormals<pcl::PointNormal, pcl::PointNormal>(targetNormals);

  pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr corRegTrimmed (new pcl::registration::CorrespondenceRejectorVarTrimmed);
  corRegTrimmed->setInputSource<pcl::PointNormal>(sourceNormals);
  corRegTrimmed->setInputTarget<pcl::PointNormal>(targetNormals);

  pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  icp.setTransformationEstimation(transformationEstimation);
  icp.addCorrespondenceRejector(corSurfNorm);
  icp.addCorrespondenceRejector(corRegTrimmed);
  icp.setInputSource(sourceNormals);
  icp.setInputTarget(targetNormals);
  icp.setMaximumIterations(icpMaxNumberOfIterations);
  icp.setMaxCorrespondenceDistance(icpMaxCorrespondenceDistance);
  icp.setTransformationEpsilon(icpTransformationEpsilon);
  icp.setEuclideanFitnessEpsilon(icpFitnessEpsilon);

  pcl::PointCloud<pcl::PointNormal>::Ptr transformedSource(new pcl::PointCloud<pcl::PointNormal>);
  icp.align(*transformedSource);
  result = icp.getFinalTransformation();
  residual = icp.getFitnessScore();

  return residual;
}

} // end namespace
