# -*- coding: utf-8 -*-

import sys
import copy
import pytest
import six
import numpy as np
import numpy.testing as npt
import sksurgerypclpython as sks


def test_generalised_icp():
    source_points_file = 'Testing/Data/SurfaceBasedRegistrationData/fran_cut_transformed.txt'
    source_points = np.loadtxt(source_points_file)
    transformed_source_points = copy.deepcopy(source_points)
    target_points_file = 'Testing/Data/SurfaceBasedRegistrationData/fran_cut.txt'
    target_points = np.loadtxt(target_points_file)
    result = np.eye(4)
    residual = sks.gicp(source_points,
                        target_points,
                        0.001,              # Normal radius search.
                        45,                 # Angle in degrees, above which points are discarded.
                        100,                # Number of iterations
                        sys.float_info.max, # Max correspondence distance, so sys.float_info.max means "unused" in this test
                        0,                  # Transformation epsilon, so setting zero also means "unused"
                        0,                  # Cost function epsilon, so setting zero also means "unused"
                        result,
                        transformed_source_points
                        )
    six.print_("\nResidual=" + str(residual))
    six.print_("Matrix=" + str(result))
    assert residual < 0.1

