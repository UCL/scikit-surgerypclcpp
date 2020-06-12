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

/* Docstrings should use C++ string literals, for easier multi line comments.
Equal indentations on each line, for Sphinx compatiblity. See
(https://pybind11.readthedocs.io/en/stable/advanced/misc.html#generating-documentation-using-sphinx)
*/

auto icp_docstring = R"(

    Run Iterative Closest Point (ICP) algorithm.

    :param source: [Nx3] source point cloud
    :type source: np.ndarray
    :param target: [Mx3] target point cloud
    :type target: np.ndarray
    :param maxNumberOfIterations: maximum number of iterations in the ICP loop
    :type maxNumberOfIterations: unsigned int
    :param maxCorrespondenceDistance: maximum distance between corresponding points, above which point is discarded
    :type maxCorrespondenceDistance: float
    :param transformationEpsilon: small change in transformation, below which causes early termination of ICP loop
    :type transformationEpsilon: float
    :param fitnessEpsilon: small change in cost function, below which causes early termination of ICP loop
    :type fitnessEpsilon: float
    :param useLM: if True, use LM-ICP, else use normal ICP.
    :type useLM: bool
    :return: 4x4 Transform matrix, transformed source cloud
    :rtype: np.ndarray, np.ndarray
    )";

auto downsample_docstring = R"(

    Downsample point cloud using a voxel grid.

    :param input: Input point cloud
    :type input: np.ndarray
    :param vx: X  voxel size (metres)
    :type vx: float
    :param vy: Y voxel size (metres)
    :type vy: float
    :param vz: Z voxel size (metres)
    :type vz: float
    :return: Output point cloud
    :rtype: np.ndarray
    )";

auto remove_outlier_docstring = R"(

    Remove statistical outliers from point cloud.

    :param input: Input point cloud
    :type input: np.ndarray
    :param meanK: Number of points to use for mean distance estimation
    :type meanK: float
    :param stdDev: Standard deviation multiplier thershold. All points outside
    mean +/- stdDev are considred outliers.
    :return: Output point cloud
    :rtype: np.darray
    )";

auto passthrough_docstring = R"(

    Filter points based on distance.

    :param input: Input point cloud
    :type input: np.ndarray
    :param fieldName: single character, either x, y or z, representing the axis.
    :type fieldName: char
    :param minDistance: minimum distance in chosen axis
    :type minDistance: float
    :param maxDistance: maximum distance in chosen axis
    :type maxDistance: float
    :param insideInterval: if True, points inside interval are kept, if False, points outside interval are kept.
    :type insideInterval: bool
    :return: Output point cloud
    :rtype: np.ndarray
    )";


auto radiusremoval_docstring = R"(

    Filter points based on neighbours within a certain radius.

    :param input: Input point cloud
    :type input: np.ndarray
    :param radius: radius around each point (metres)
    :type radius: float
    :param minNumberOfNeighbours: minimum number of points to have within the given radius, without which, point is discarded
    :type minNumberOfNeighbours: unsigned int
    :return: Output point cloud
    :rtype: np.ndarray
    )";


auto featurematch_docstring = R"(

    Do rigid registration based on SIFT keypoints, FPFH descriptors, RANSAC correspondence matching, and SVD of matched points.

    :param source: [Nx3] source point cloud
    :type source: np.ndarray
    :param target: [Mx3] target point cloud
    :type target: np.ndarray
    :param normalSearchRadius: radius around each point (metres)
    :type normalSearchRadius: float
    :param siftMinScale: minimum scale for SIFT operator
    :type siftMinScale: float
    :param siftNumOctaves: number of octaves for SIFT operator
    :type siftNumOctaves: unsigned int
    :param siftNumScalesPerOctave: number of scales per octave for SIFT operator
    :type siftNumScalesPerOctave: unsigned int
    :param siftMinContrast: minimum contrast (difference in normal vector component) for SIFT operator
    :type siftMinContrast: float
    :param siftKSearch: number of neighbours for SIFT to search over
    :type siftKSearch: unsigned int
    :param ransacInlierThreshold: distance within which to call points inlier matches (metres)
    :type ransacInlierThreshold: float
    :param ransacMaximumIterations: maximum number of RANSAC iterations.
    :type ransacMaximumIterations: unsigned int
    :return: 4x4 Transform matrix, transformed source cloud
    :rtype: np.ndarray, np.ndarray
    )";

#endif
