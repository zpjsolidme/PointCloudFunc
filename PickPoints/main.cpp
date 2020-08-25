
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;

struct callback_args
{
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vierwe"));


void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args*)args;
	if (event.getPointIndex()==-1)
	{
		return;
	}
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);

	//画选中的点云
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout
		<< current_point.x << " " 
		<< current_point.y << " " 
		<< current_point.z << std::endl;
	
	char cpz[20];
	sprintf(cpz, "%.2f", current_point.z);
	string pz = cpz;
	viewer->addText(pz,100* current_point.x, 100*current_point.y,cpz,0);


}


void main()
{


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
			cloud->points[h*cloud->width +w].z = sin(w*0.1);
			//cloud->points[h*cloud->width + w].z = 0;
		}
	}	

	std::cout << cloud->points.size() << std::endl;
	viewer->addPointCloud(cloud, "plane");
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);



	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "x");
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");


	//添加点选取窗体回调
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
	viewer->spin();
	std::cout << "done." << std::endl;
	cloud_mutex.unlock();

	//pcl::visualization::CloudViewer viewer("CloudViewer");
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	
	//cloud->width = 100;
	//cloud->height = 100;
	//cloud->is_dense = false;
	//cloud->points.resize(cloud->width*cloud->height);
	//for (int h = 0; h < cloud->height; h++)
	//{
	//	for (int w = 0; w < cloud->width; w++)
	//	{
	//		cloud->points[h*cloud->width + w].x = w*0.1;
	//		cloud->points[h*cloud->width + w].y = h*0.1f;
	//		//cloud.points[h*cloud.width +w].z = sin(j*0.1);
	//		cloud->points[h*cloud->width + w].z = 0;
	//	}
	//}
	//viewer.showCloud(cloud);




	system("pause");


}