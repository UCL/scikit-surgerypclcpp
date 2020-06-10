/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksDownSamplePointsWrapper.h"
#include "sksConversionUtils.h"
#include <sksDownSamplePointCloud.h>


namespace sks
{

//-----------------------------------------------------------------------------
np::ndarray DownSamplePointsWrapper(const np::ndarray& input,
                                    float vx,
                                    float vy,
                                    float vz
                                   )
{
  CheckInputIs3DFloat(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = ConvertInputToPointCloud(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud = sks::DownSamplePointCloud(inputCloud, vx, vy, vz);

  np::ndarray result = ConvertPointCloudToNumpy(resultCloud);

  return result;
}

} // end namespace