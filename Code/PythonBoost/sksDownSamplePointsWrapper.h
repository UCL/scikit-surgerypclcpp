/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksDownSamplePointsWrapper_h
#define sksDownSamplePointsWrapper_h

#include <boost/python/numpy.hpp>
#include "sksWin32ExportHeader.h"

namespace np = boost::python::numpy;

namespace sks
{

/**
  @brief Filter to down-sample a point cloud using the pcl::VoxelGrid filter.
  @param input [Nx3] point cloud, where each row is x, y, z coordinates.
  @param vx size of voxel grid in x direction.
  @param vy size of voxel grid in y direction.
  @param vz size of voxel grid in z direction.
*/
SKSURGERYPCLCPP_WINEXPORT np::ndarray DownSamplePointsWrapper(const np::ndarray& input,
                                                              float vx,
                                                              float vy,
                                                              float vz
                                                             );
}
#endif
