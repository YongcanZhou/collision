#include "pointswidget.h"

#include <vtkAutoInit.h>
// #include <vtkRenderWindow.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
// VTK_MODULE_INIT(vtkRenderingOpenGL2);
// VTK_MODULE_INIT(vtkInteractionStyle);

pointsWidget::pointsWidget(QWidget *parent) : QWidget(parent)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//char strfilepath[256] = "C:/Users/taye709/Desktop/pcl_test/Project1/rabbit.pcd";
	//pcl::io::loadPCDFile(strfilepath, *cloud);
	//pcl::visualization::CloudViewer viewer("Cloud Viewer");   //创建viewer对象
	//viewer.showCloud(cloud);
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);


}
