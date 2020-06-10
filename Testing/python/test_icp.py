# -*- coding: utf-8 -*-

import sys
import pytest
import six
import numpy as np
import numpy.testing as npt
import sksurgerypclpython as sks


def test_fran_cut_icp():
    # Loading plain-text versions of VTK fran_cut data set.
    source_points_file = 'Testing/Data/SurfaceBasedRegistrationData/fran_cut_transformed.txt'
    source_points = np.loadtxt(source_points_file)
    target_points_file = 'Testing/Data/SurfaceBasedRegistrationData/fran_cut.txt'
    target_points = np.loadtxt(target_points_file)
    result = np.eye(4)
    residual = sks.iterative_closest_point(source_points, target_points, 10, sys.float_info.max, -sys.float_info.max, 0, False, result)
    six.print_("Residual=" + str(residual))
    six.print_("Matrix=" + str(result))
    assert residual < 0.1
    residual = sks.iterative_closest_point(source_points, target_points, 10, sys.float_info.max, -sys.float_info.max, 0, True, result)
    six.print_("Residual=" + str(residual))
    six.print_("Matrix=" + str(result))
    assert residual < 0.1
    npt.assert_almost_equal(result[0][3], 0, decimal=2)
    npt.assert_almost_equal(result[1][3], 0, decimal=2)
    npt.assert_almost_equal(result[2][3], 0, decimal=2)
