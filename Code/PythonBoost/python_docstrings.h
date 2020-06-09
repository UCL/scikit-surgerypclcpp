/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef DOCSTRINGS_H
#define DOCSTRINGS_H

using namespace std;

auto icp_docstring = R"mydelimiter(
    Run Iterative Closest Point algorithm.
    
    :param source: Source pointcloud
    :type source: np.ndarray
    :param target: Target point cloud
    :type target: np.ndarray
    :return: 4x4 Transform matrix
    )mydelimiter";

auto downsample_docstring = R"mydelimiter(
    Downsample point cloud.

    :param input: Input pointcloud
    :type input: np.ndarray
    :param vx: X  voxel size (metres)
    :type vx: float
    :param vy: Y voxel size (metres)
    :type vy: float
    :param vz: Z voxel size (metres)
    :type vz: float
    :return: Modified point cloud
    :rtype: np.ndarray
    )mydelimiter";

auto remove_outlier_docstring = R"mydelimiter(
    Remove outliers from point cloud.

    :param input: Input pointcloud
    :type input: np.ndarray
    :param meanK: Number of points to use for mean distance estimation
    :type meanK: float
    :param stdDev: Standard deviation multiplier thershold. All points outside
    mean +/- stdDev are considred outliers.
    :return: Output pointcloud
    :rtype: np.darray
    )mydelimiter";

#endif
