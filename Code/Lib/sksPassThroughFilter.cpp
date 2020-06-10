/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksPassThroughFilter.h"
#include <pcl/filters/passthrough.h>

namespace sks {

//-----------------------------------------------------------------------------
const pcl::PointCloud<pcl::PointXYZ>::Ptr PassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                                            const char fieldName, float minDistance, float maxDistance, bool insideInterval)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName(std::string(1, fieldName));
  pass.setFilterLimits(minDistance, maxDistance);
  pass.setFilterLimitsNegative(!insideInterval);
  pass.filter(*output);

  return output;
}

} // end namespace
