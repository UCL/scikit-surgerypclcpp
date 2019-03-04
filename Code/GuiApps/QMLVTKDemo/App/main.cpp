/*=============================================================================

  SKSURGERYPCLCPP: Image-guided surgery functions, in C++, using PCL.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/
#include <QGuiApplication>
#include "sksurgerypclcppConfigure.h"
#include <sksTriangleModel.h>
#include <sksCubeModel.h>
#include <sksQQuickVTKView.h>

int main(int argc, char *argv[])
{
  QGuiApplication app(argc, argv);
  app.setOrganizationName("UCL");
  app.setApplicationName("QMLVTKDemo");
  app.setApplicationVersion(QString(SKSURGERYPCLCPP_VERSION_STRING));

  qmlRegisterType<sks::TriangleModel>("QMLVTKDemo", 1, 0, "TriangleModel");
  qmlRegisterType<sks::CubeModel>("QMLVTKDemo", 1, 0, "CubeModel");

  QSurfaceFormat fmt;
  fmt.setDepthBufferSize(24);
  fmt.setVersion(3, 2);
  fmt.setProfile(QSurfaceFormat::CoreProfile);
  QSurfaceFormat::setDefaultFormat(fmt);

  sks::QQuickVTKView view;
  view.setResizeMode(QQuickView::SizeRootObjectToView);
  view.setSource(QUrl("qrc:/main.qml"));
  view.show();
  view.SetEnabled(true);

  return app.exec();

}

