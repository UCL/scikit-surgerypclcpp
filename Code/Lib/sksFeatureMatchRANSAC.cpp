/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksFeatureMatchRANSAC.h"
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <iostream>

namespace sks {

//-----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointWithScale>::Ptr computeSIFTPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, float normalSearchRadius)
{
  const float siftMinScale = 0.01f;
  const int siftNumOctaves = 3;
  const int siftNScalesPerOctave = 4;
  const float siftMinContrast = 0.001f;

  pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr treeN(new pcl::search::KdTree<pcl::PointXYZ>());

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normalEstimation;
  normalEstimation.setInputCloud(input);
  normalEstimation.setSearchMethod(treeN);
  normalEstimation.setRadiusSearch(normalSearchRadius);
  normalEstimation.compute(*normals);

  // Copy the xyz info from input and add it to normals as the xyz field in PointNormals estimation is zero
  for(size_t i = 0; i < normals->points.size(); ++i)
  {
    normals->points[i].x = input->points[i].x;
    normals->points[i].y = input->points[i].y;
    normals->points[i].z = input->points[i].z;
  }

  // Estimate the SIFT interest points using normals values from xyz as the Intensity variants
  pcl::PointCloud<pcl::PointWithScale>::Ptr result(new pcl::PointCloud<pcl::PointWithScale>);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());

  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  sift.setSearchMethod(tree);
  sift.setScales(siftMinScale, siftNumOctaves, siftNScalesPerOctave);
  sift.setMinimumContrast(siftMinContrast);
  sift.setInputCloud(normals);
  sift.compute(*result);

  return result;
}


//-----------------------------------------------------------------------------
double FeatureMatchRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                          float siftNormalSearchRadius,
                          float ransacInlierThreshold,
                          unsigned int ransacMaximumIterations,
                          float icpTransformationEpsilon,
                          unsigned int icpMaximumIterations,
                          Eigen::Matrix4f& result,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSource
                          )
{

  #define Scalar float
  double residual = std::numeric_limits<double>::max();

  pcl::PointCloud<pcl::PointWithScale>::Ptr sourceSIFT = computeSIFTPoints(source, siftNormalSearchRadius);
  pcl::PointCloud<pcl::PointWithScale>::Ptr targetSIFT = computeSIFTPoints(target, siftNormalSearchRadius);

  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale>::Ptr corRejSAC (new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale>);
  corRejSAC->setInputSource(sourceSIFT);
  corRejSAC->setInputTarget(targetSIFT);
  corRejSAC->setInlierThreshold(ransacInlierThreshold);
  corRejSAC->setMaximumIterations(ransacMaximumIterations);
  corRejSAC->setRefineModel(false);

  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, Scalar>::Ptr corEst(new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, Scalar>);
  corEst->setInputSource(source);
  corEst->setInputTarget(target);

  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, Scalar>::Ptr transEst(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, Scalar>);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, Scalar> icp;
  icp.setCorrespondenceEstimation(corEst);
  icp.setTransformationEstimation(transEst);
  icp.addCorrespondenceRejector(corRejSAC);
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(icpMaximumIterations);
  icp.setTransformationEpsilon(icpTransformationEpsilon);
  icp.align(*transformedSource);

  result = icp.getFinalTransformation();
  residual = icp.getFitnessScore();

  return residual;
}

} // end namespace
