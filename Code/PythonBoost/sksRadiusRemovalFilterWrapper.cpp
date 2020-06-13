/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksRadiusRemovalFilterWrapper.h"
#include "sksConversionUtils.h"
#include <sksRadiusRemovalFilter.h>


namespace sks
{

//-----------------------------------------------------------------------------
np::ndarray RadiusRemovalFilterWrapper(const np::ndarray& input,
                                       float radius,
                                       unsigned int minNumberOfNeighbours
                                      )
{
  CheckInputIs3DFloat(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = ConvertInputToPointCloud(input);

  pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud = sks::RadiusRemovalFilter(inputCloud, radius, minNumberOfNeighbours);

  np::ndarray result = ConvertPointCloudToNumpy(resultCloud);

  return result;
}

} // end namespace