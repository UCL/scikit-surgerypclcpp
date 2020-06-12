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

    voxel_reduced_surface = sks.down_sample_points(source_points, 0.01, 0.01, 0.01)
    six.print_("voxel_reduced_surface.shape=" + str(voxel_reduced_surface.shape))

    outlier_reduced_surface = sks.remove_outlier_points(voxel_reduced_surface, 50, 1.0)
    six.print_("outlier_reduced_surface.shape=" + str(outlier_reduced_surface.shape))

    pass_through_filtered = sks.pass_through_filter(outlier_reduced_surface, 'z', -1, -0.05, True)
    six.print_("pass_through_filtered.shape=" + str(pass_through_filtered.shape))

    radius_filtered = sks.radius_removal_filter(pass_through_filtered, 0.01, 1)
    six.print_("radius_filtered.shape=" + str(radius_filtered.shape))

    assert voxel_reduced_surface.shape[0] < source_points.shape[0]
    assert outlier_reduced_surface.shape[0] < voxel_reduced_surface.shape[0]
    assert radius_filtered.shape[0] < outlier_reduced_surface.shape[0]
    assert radius_filtered.shape[0] < pass_through_filtered.shape[0]
