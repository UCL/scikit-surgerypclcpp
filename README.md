scikit-surgerypclcpp
====================

[![Build Status](https://travis-ci.com/UCL/scikit-surgerypclcpp.svg?branch=master)](https://travis-ci.com/UCL/scikit-surgerypclcpp)
[![Build status](https://ci.appveyor.com/api/projects/status/yt3gf13onu9q5wx5/branch/master?svg=true)](https://ci.appveyor.com/project/MattClarkson/scikit-surgerypclcpp/branch/master)




scikit-surgerypclcpp implements image guided surgery algorithms, using [PCL](http://pointclouds.org/), in C++ and wrapped in Python.

scikit-surgerypclcpp is part of the 
[SNAPPY](https://weisslab.cs.ucl.ac.uk/WEISS/PlatformManagement/SNAPPY/wikis/home) software project, 
developed at the [Wellcome EPSRC Centre for Interventional and Surgical Sciences](http://www.ucl.ac.uk/weiss), 
part of [University College London (UCL)](http://www.ucl.ac.uk/).


Features
--------

* Support for Python Wheels, thanks to [Matthew Brett's multibuild](https://github.com/matthew-brett/multibuild).
* Iterative Closest Point (ICP) algorithm to register two N (rows) x 3 (columns, x, y, z) point sets.
* Downsampling of point clouds via ```pcl::VoxelGrid``` filter.
* Removal of outlier points from point clouds via ```pcl::StatisticalOutlierRemoval```.

Look in ```Code/PythonBoost/sksLibPython.cpp``` for python method names, and in the containing folder,
to see header files with the method signatures.


Caveat
------

As of 2020-05-19, there are a few build issues, that are proving problematic, and we
have limited time to resolve them.

* C++ tests are turned off, so currently the CI builds build the python wheel and then runs python unit tests.
* To turn C++ tests on, add the ctest commands into ```travis_cmake_build.sh``` and ```appveyor.yml``` and turn ```-DBUILD_TESTING:BOOL=ON```
* If you turn C++ tests on, Mac should work fine, Linux has problems liking to LZ4 due to FLANN 1.9.1, and on Windows, we get multiply defined symbols due to Boost.
* Python unit testing was seg-faulting on Linux due to using collections to gather unit tests, so we are currently calling each test one at a time.
* So, if you want to add a python unit test, you need to add to ```config.sh``` for each new pytest file.

The long-term solution may be to work more closely with the PCL build team. So, change the MetaBuild to use PCL's docker image on
Linux, and mimic their build environment on Windows/Mac, rather than having our own. 
Most of the build issues appear to be due to templating more than anything, so a more thorough understanding
there would help.


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