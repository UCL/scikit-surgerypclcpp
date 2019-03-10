# -*- coding: utf-8 -*-

import pytest
import six
import numpy as np
import sksurgerypclpython as sks


def test_basic_icp():
    s = np.zeros((3, 3))
    s[1][0] = 1
    s[2][1] = 1
    t = s + 1
    r = np.eye(4)
    d = sks.iterative_closest_point(s, t, r)
    six.print_("Residual=" + str(d))
    six.print_("Matrix=" + str(r))
    assert r[0][3] == 1
    assert r[1][3] == 1
    assert r[2][3] == 1
