/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "sksIterativeClosestPointWrapper.h"

#include <iostream>

namespace sks
{

//-----------------------------------------------------------------------------
double IterativeClosestPointWrapper(const np::ndarray& fixed,
                                    const np::ndarray& moving,
                                    np::ndarray& result
                                   )
{
  std::cerr << "Matt, fixed shape=" << fixed.shape(0) << ", " << fixed.shape(1) << std::endl;
  std::cerr << "Matt, moving shape=" << moving.shape(0) << ", " << moving.shape(1) << std::endl;
  std::cerr << "Matt, result shape=" << result.shape(0) << ", " << result.shape(1) << std::endl;
  return 0.0;
}

} // end namespace