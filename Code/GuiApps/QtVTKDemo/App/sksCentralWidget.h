/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksCentralWidget_h
#define sksCentralWidget_h

#include "ui_sksCentralWidget.h"
#include <sksVTKViewWidget.h>
#include <QWidget>

namespace sks
{

/**
* \class CentralWidget
* \brief Demo widget to group a VTKViewWidget with ControlPanelWidget.
*/
class CentralWidget : public QWidget, public Ui_CentralWidget
{
  Q_OBJECT

public:

  CentralWidget(QWidget* parent);
  virtual ~CentralWidget();

  VTKViewWidget* GetVTKViewWidget() const;

public slots:

  void SetIntensityRange(int low, int high);

signals:

  void WindowValuesChanged(int low, int high);
  void DoSomethingPressed();

}; // end class

} // end namespace

#endif
