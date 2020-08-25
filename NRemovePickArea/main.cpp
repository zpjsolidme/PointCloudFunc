#include <pcl\point_cloud.h>//点云类型
#include <pcl\point_types.h>//点类型
#include <pcl\visualization\pcl_visualizer.h>//可视化类

using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));





void main()
{
	/**********构建点云*************/
	cloud->width = 100;
	cloud->height = 100;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width*cloud->height);
	for (int h = 0; h < cloud->height; h++)
	{
		for (int w = 0; w < cloud->width; w++)
		{
			cloud->points[h*cloud->width + w].x = w*0.1f;
			cloud->points[h*cloud->width + w].y = h*0.1f;
			//cloud->points[h*cloud->width + w].z = sin(w*0.1);
			cloud->points[h*cloud->width + w].z = 1;
		}
	}

	viewer->addPointCloud(cloud, "srccloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "srccloud");
	viewer->addCoordinateSystem();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(10000);
		std::cout << "1" << std::endl;
		//viewer->spin();
	}



}



