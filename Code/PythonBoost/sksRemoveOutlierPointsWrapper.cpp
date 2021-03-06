/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksRemoveOutlierPointsWrapper.h"
#include "sksConversionUtils.h"
#include <sksRemoveOutliersFromPointCloud.h>


namespace sks
{

//-----------------------------------------------------------------------------
np::ndarray RemoveOutlierPointsWrapper(const np::ndarray& input,
                                       float meanK,
                                       float stdDev
                                      )
{
  CheckInputIs3DFloat(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = ConvertInputToPointCloud(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud = sks::RemoveOutliersFromPointCloud(inputCloud, meanK, stdDev);

  np::ndarray result = ConvertPointCloudToNumpy(resultCloud);

  return result;
}

} // end namespace