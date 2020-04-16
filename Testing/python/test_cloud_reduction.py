# -*- coding: utf-8 -*-

import pytest
import six
import numpy as np
import numpy.testing as npt
import sksurgerypclpython as sks


def test_cloud_reduction():
    # Loading plain-text versions of VTK fran_cut data set.
    source_points_file = 'Testing/Data/SurfaceBasedRegistrationData/fran_cut.txt'

    source_points = np.loadtxt(source_points_file)
    six.print_("source_points.shape=" + str(source_points.shape))

    voxel_reduced_surface = sks.down_sample_points(source_points, 0.01, 0.01, 0.01)  # in metres?
    six.print_("voxel_reduced_surface.shape=" + str(voxel_reduced_surface.shape))

    outlier_reduced_surface = sks.remove_outlier_points(voxel_reduced_surface, 50, 1.0)
    six.print_("outlier_reduced_surface.shape=" + str(outlier_reduced_surface.shape))

    assert voxel_reduced_surface.shape[0] < source_points.shape[0]
    assert outlier_reduced_surface.shape[0] < voxel_reduced_surface.shape[0]
