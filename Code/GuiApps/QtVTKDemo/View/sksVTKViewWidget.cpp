/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/
#include "sksVTKViewWidget.h"
#include <sksExceptionMacro.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>

#include <cassert>
#include <vtkNew.h>

namespace sks
{

//-----------------------------------------------------------------------------
VTKViewWidget::VTKViewWidget(QWidget* parent)
#ifdef BUILD_VTK_OpenGL
: QVTKWidget2(parent)
#else
: QVTKOpenGLWidget(parent)
#endif
{

#ifdef BUILD_VTK_OpenGL2
  m_RenderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  this->SetRenderWindow(m_RenderWindow);
#endif

}


//-----------------------------------------------------------------------------
VTKViewWidget::~VTKViewWidget()
{
}


//-----------------------------------------------------------------------------
void VTKViewWidget::AddRenderer(vtkRenderer* r)
{
  if (r == nullptr)
  {
    sksExceptionThrow() << "Renderer is NULL";
  }

  this->GetRenderWindow()->AddRenderer(r);
}


//-----------------------------------------------------------------------------
void VTKViewWidget::Render()
{
  this->GetRenderWindow()->Render();
}

} // end namespace
