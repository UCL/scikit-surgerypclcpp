scikit-surgerypclcpp
====================

[![Build Status](https://travis-ci.com/UCL/scikit-surgerypclcpp.svg?branch=master)](https://travis-ci.com/UCL/scikit-surgerypclcpp)
[![Build status](https://ci.appveyor.com/api/projects/status/yt3gf13onu9q5wx5/branch/master?svg=true
)](https://ci.appveyor.com/project/MattClarkson/scikit-surgerypclcpp)


scikit-surgerypclcpp implements image guided surgery algorithms, using [PCL](http://pointclouds.org/), in C++ and wrapped in Python.

scikit-surgerypclcpp is part of the 
[SNAPPY](https://weisslab.cs.ucl.ac.uk/WEISS/PlatformManagement/SNAPPY/wikis/home) software project, 
developed at the [Wellcome EPSRC Centre for Interventional and Surgical Sciences](http://www.ucl.ac.uk/weiss), 
part of [University College London (UCL)](http://www.ucl.ac.uk/).


Features
--------

* Support for Python Wheels, thanks to [Matthew Brett's multibuild](https://github.com/matthew-brett/multibuild).
* Iterative Closest Point (ICP) algorithm to register two N (rows) x 3 (columns, x, y, z) point sets.


Installing
----------

You can pip install the latest Python package as follows:

```
pip install scikit-surgerypclcpp
```


Developing
==========

Cloning
-------

You can clone the repository using the following command:

```
git clone https://github.com/UCL/scikit-surgerypclcpp
```


Build instructions
------------------

Still not for the faint-hearted. It depends if you are a C++ developer familiar
with CMake or a hybrid C++/Python developer primarily interested in writing
Python extensions.

The simplest advice really is to read ```appveyor.yml```, as this will always
be up to date. 


Preferred Branching Workflow for Contributions.
-----------------------------------------------

We welcome contributions to this project. Please use the following workflow.

 1. Raise issue in this project's Github Issue Tracker.
 2. Fork repository.
 3. Create a feature branch called ```<issue-number>-<some-short-description>```
    replacing ```<issue-number>``` with the Github issue number
    and ```<some-short-description>``` with your description of the thing you are implementing.
 4. Code on that branch.
 5. Push to your remote when ready.
 6. Create pull request.
 7. We will review code, suggest and required changes and merge to master when it is ready.


Licensing and copyright
-----------------------

Copyright 2018 University College London.
scikit-surgeryopencvcpp is released under the BSD-3 license. 
Please see the [license file](https://github.com/UCL/scikit-surgeryopencvcpp/blob/master/LICENSE.txt) for details.


Acknowledgements
----------------

Supported by [Wellcome](https://wellcome.ac.uk/) and the [EPSRC](https://www.epsrc.ac.uk/).

The project was generated, using 
[CMakeCatchTemplate](https://github.com/MattClarkson/CMakeCatchTemplate) 
and [CMakeTemplateRenamer](https://github.com/MattClarkson/CMakeTemplateRenamer).