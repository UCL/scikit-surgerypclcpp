/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksFeatureMatch.h"
#include "sksMyFunctions.h"
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/icp.h>
#include <iostream>

namespace sks {

//-----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointWithScale>::Ptr computeSIFTPoints(const pcl::PointCloud<pcl::PointNormal>::ConstPtr pointNormals,
                                                            float minScale,
                                                            unsigned int numOctaves,
                                                            unsigned int numScalesPerOctave,
                                                            float minContrast,
                                                            int kSearch
                                                            )
{
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());

  pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
  sift.setInputCloud(pointNormals);
  sift.setSearchSurface(pointNormals);
  sift.setSearchMethod(tree);
  sift.setScales(minScale, numOctaves, numScalesPerOctave);
  sift.setMinimumContrast(minContrast);
  sift.setKSearch(kSearch);

  pcl::PointCloud<pcl::PointWithScale>::Ptr siftKeyPoints(new pcl::PointCloud<pcl::PointWithScale>);
  sift.compute(*siftKeyPoints);

  return siftKeyPoints;
}


//-----------------------------------------------------------------------------
pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFeatures(const pcl::PointCloud<pcl::PointNormal>::ConstPtr points,
                                                           const pcl::PointCloud<pcl::PointNormal>::ConstPtr keyPoints,
                                                           float normalSearchRadius
                                                           )
{
  pcl::Feature<pcl::PointNormal, pcl::FPFHSignature33>::Ptr featureExtractor (new pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>);
  featureExtractor->setSearchMethod(pcl::search::Search<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  featureExtractor->setRadiusSearch(normalSearchRadius * 2.0);
  featureExtractor->setSearchSurface(points);
  featureExtractor->setInputCloud(keyPoints);

  typename pcl::FeatureFromNormals<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33>::Ptr featureFromNormals
    = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> > (featureExtractor);
  featureFromNormals->setInputNormals(points);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
  featureExtractor->compute (*features);

  return features;
}


//-----------------------------------------------------------------------------
void FeatureMatch(const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                  float normalSearchRadius,
                  float siftMinScale,
                  unsigned int siftNumOctaves,
                  unsigned int siftNumScalesPerOctave,
                  float siftMinContrast,
                  int siftKSearch,
                  float ransacInlierThreshold,
                  unsigned int ransacMaximumIterations,
                  Eigen::Matrix4f& transformationMatrix,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedSource
                  )
{

  #define Scalar float

  pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormals = computeNormals(source,
                                                                        normalSearchRadius);

  pcl::PointCloud<pcl::PointWithScale>::Ptr sourceSIFT = computeSIFTPoints(sourceNormals,
                                                                           siftMinScale,
                                                                           siftNumOctaves,
                                                                           siftNumScalesPerOctave,
                                                                           siftMinContrast,
                                                                           siftKSearch
                                                                           );
  typename pcl::PointCloud<pcl::PointNormal>::Ptr sourceKpts(new pcl::PointCloud<pcl::PointNormal>);
  sourceKpts->points.resize(sourceSIFT->points.size());
  pcl::copyPointCloud(*sourceSIFT, *sourceKpts);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures = computeFeatures(sourceNormals,
                                                                              sourceKpts,
                                                                              normalSearchRadius
                                                                              );

  pcl::PointCloud<pcl::PointNormal>::Ptr targetNormals = computeNormals(target, normalSearchRadius);

  pcl::PointCloud<pcl::PointWithScale>::Ptr targetSIFT = computeSIFTPoints(targetNormals,
                                                                           siftMinScale,
                                                                           siftNumOctaves,
                                                                           siftNumScalesPerOctave,
                                                                           siftMinContrast,
                                                                           siftKSearch
                                                                           );

  typename pcl::PointCloud<pcl::PointNormal>::Ptr targetKpts(new pcl::PointCloud<pcl::PointNormal>);
  targetKpts->points.resize(targetSIFT->points.size());
  pcl::copyPointCloud(*targetSIFT, *targetKpts);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures = computeFeatures(targetNormals,
                                                                              targetKpts,
                                                                              normalSearchRadius
                                                                              );

  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corEst;
  corEst.setInputSource(sourceFeatures);
  corEst.setInputTarget(targetFeatures);

  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  corEst.determineReciprocalCorrespondences(*correspondences);

  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> rejector;
  rejector.setInputSource(sourceKpts);
  rejector.setInputTarget(targetKpts);
  rejector.setInlierThreshold(ransacInlierThreshold);
  rejector.setMaximumIterations(ransacMaximumIterations);
  rejector.setRefineModel(false);
  rejector.setInputCorrespondences(correspondences);

  pcl::Correspondences correspondencesFiltered;
  rejector.getCorrespondences(correspondencesFiltered);

  pcl::registration::TransformationEstimation<pcl::PointNormal, pcl::PointNormal>::Ptr transformationEstimation(new pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal>);
  transformationEstimation->estimateRigidTransformation(*sourceKpts, *targetKpts, correspondencesFiltered, transformationMatrix);

  pcl::transformPointCloud(*source, *transformedSource, transformationMatrix);
}

} // end namespace
