/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef sksMainWindow_h
#define sksMainWindow_h

#include "ui_sksMainWindow.h"
#include <sksVolumeRenderingModel.h>

#include <QMainWindow>

namespace sks
{

class VTKViewWidget;

/**
* \class MainWindow
* \brief Demo Widget provides main window, and connects it to Model.
*/
class MainWindow : public QMainWindow, public Ui_MainWindow
{
  Q_OBJECT

public:

  MainWindow(sks::VolumeRenderingModel* model);
  virtual ~MainWindow();
  void ConnectRenderer();

private slots:

  void OnFileOpen();

private:

  sks::VolumeRenderingModel* m_Model;
  sks::VTKViewWidget*        m_Viewer;

}; // end class

} // end namespace

#endif
