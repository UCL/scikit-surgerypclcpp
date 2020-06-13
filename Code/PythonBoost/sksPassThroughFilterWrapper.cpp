/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksPassThroughFilterWrapper.h"
#include "sksConversionUtils.h"
#include <sksPassThroughFilter.h>


namespace sks
{

//-----------------------------------------------------------------------------
np::ndarray PassThroughFilterWrapper(const np::ndarray& input,
                                     char fieldName,
                                     float minDistance,
                                     float maxDistance,
                                     bool insideInterval
                                    )
{
  CheckInputIs3DFloat(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = ConvertInputToPointCloud(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud = sks::PassThroughFilter(inputCloud, fieldName, minDistance, maxDistance, insideInterval);

  np::ndarray result = ConvertPointCloudToNumpy(resultCloud);

  return result;
}

} // end namespace