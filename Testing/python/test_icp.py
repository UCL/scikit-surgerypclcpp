# -*- coding: utf-8 -*-

import sys
import copy
import pytest
import six
import numpy as np
import numpy.testing as npt
import sksurgerypclpython as sks


def test_fran_cut_icp():
    # Loading plain-text versions of VTK fran_cut data set.
    source_points_file = 'Testing/Data/SurfaceBasedRegistrationData/fran_cut_transformed.txt'
    source_points = np.loadtxt(source_points_file)
    transformed_source_points = copy.deepcopy(source_points)
    target_points_file = 'Testing/Data/SurfaceBasedRegistrationData/fran_cut.txt'
    target_points = np.loadtxt(target_points_file)
    result = np.eye(4)
    residual = sks.icp(source_points,
                       target_points,
                       100,                        # Number of iterations
                       sys.float_info.max,         # Max correspondence distance, so sys.float_info.max means "unused" in this test
                       0.00001,                    # Transformation epsilon
                       0.00001,                    # Cost function epsilon
                       False,                      # Use LM-ICP
                       result,                     # Output 4x4
                       transformed_source_points)  # Output transformed points

    six.print_("\nResidual=" + str(residual))
    six.print_("Matrix=" + str(result))
    assert residual < 0.1

    residual = sks.icp(source_points,
                       target_points,
                       100,                        # Number of iterations
                       sys.float_info.max,         # Max correspondence distance, so sys.float_info.max means "unused" in this test
                       0,                          # Transformation epsilon, so setting zero also means "unused"
                       0,                          # Cost function epsilon, so setting zero also means "unused"
                       True,                       # Use LM-ICP
                       result,                     # Output 4x4
                       transformed_source_points)  # Output transformed points

    six.print_("Residual=" + str(residual))
    six.print_("Matrix=" + str(result))
    assert residual < 0.1

    npt.assert_almost_equal(result[0][3], 0, decimal=2)
    npt.assert_almost_equal(result[1][3], 0, decimal=2)
    npt.assert_almost_equal(result[2][3], 0, decimal=2)
