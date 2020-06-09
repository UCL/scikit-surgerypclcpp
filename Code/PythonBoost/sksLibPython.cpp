/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/numpy.hpp>
#include "sksIterativeClosestPointWrapper.h"
#include "sksDownSamplePointsWrapper.h"
#include "sksRemoveOutlierPointsWrapper.h"
#include "sksException.h"
#include "python_docstrings.h"

#include <ostream>
#include <sstream>

namespace sks {

void translate_exception(Exception const& e)
{
  std::ostringstream ss;
  ss << e.GetDescription();
  ss << " in file:" << e.GetFileName();
  ss << ", line:" << e.GetLineNumber();
  PyErr_SetString(PyExc_RuntimeError, ss.str().c_str());
}

// The name of the module should match that in CMakeLists.txt
BOOST_PYTHON_MODULE(sksurgerypclpython)
{
  boost::python::numpy::initialize();
  boost::python::register_exception_translator<Exception>(&translate_exception);
  boost::python::def("iterative_closest_point", sks::IterativeClosestPointWrapper, icp_docstring);
  boost::python::def("down_sample_points", sks::DownSamplePointsWrapper, downsample_docstring);
  boost::python::def("remove_outlier_points", sks::RemoveOutlierPointsWrapper, remove_outlier_docstring);
}

}  // end namespace sks
