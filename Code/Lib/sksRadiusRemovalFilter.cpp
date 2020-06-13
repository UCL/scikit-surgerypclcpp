/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksPassThroughFilter.h"
#include <pcl/filters/radius_outlier_removal.h>

namespace sks {

//-----------------------------------------------------------------------------
const pcl::PointCloud<pcl::PointXYZ>::Ptr RadiusRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                              float radius, unsigned int minNumberOfNeighbours)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusFilter;
  radiusFilter.setInputCloud(input);
  radiusFilter.setRadiusSearch(radius);
  radiusFilter.setMinNeighborsInRadius(minNumberOfNeighbours);
  radiusFilter.filter(*output);

  return output;
}

} // end namespace
