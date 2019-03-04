/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <QApplication>
#include <QDesktopWidget>
#include <QSurfaceFormat>
#include "sksurgerypclcppConfigure.h"
#include "sksOpenGLWidget.h"
#include "sksOpenGLMainWindow.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
  app.setOrganizationName("UCL");
  app.setApplicationName("QOpenGLDemo");
  app.setApplicationVersion(QString(SKSURGERYPCLCPP_VERSION_STRING));

  QSurfaceFormat fmt;
  fmt.setDepthBufferSize(24);
  fmt.setVersion(3, 2);
  fmt.setProfile(QSurfaceFormat::CoreProfile);
  QSurfaceFormat::setDefaultFormat(fmt);

  sks::OpenGLMainWindow mainWindow;
  sks::OpenGLWidget::setTransparent(false);
  if (sks::OpenGLWidget::isTransparent())
  {
    mainWindow.setAttribute(Qt::WA_TranslucentBackground);
    mainWindow.setAttribute(Qt::WA_NoSystemBackground, false);
  }
  mainWindow.resize(mainWindow.sizeHint());
  int desktopArea = QApplication::desktop()->width() *
                    QApplication::desktop()->height();
  int widgetArea = mainWindow.width() * mainWindow.height();
  if (((float)widgetArea / (float)desktopArea) < 0.75f)
  {
    mainWindow.show();
  }
  else
  {
    mainWindow.showMaximized();
  }
  return app.exec();
}
