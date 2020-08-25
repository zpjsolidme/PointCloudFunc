#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>

#include <pcl/filters/extract_indices.h>



using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//原始点云
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);//框选点云

pcl::PointCloud<pcl::PointXYZ>::Ptr FINALcloud(new pcl::PointCloud<pcl::PointXYZ>());//删除后的点云

std::vector<int> totalIndices; //保存点索引号
std::vector <int> FINALIndices;//剩余点索引

int num = 0;
int cloudsize = 0;

//查找剩余点
bool myfind(int temp1, const vector<int>& temp)
{
	for (int i = 0; i < temp.size(); i++)
	{
		if (temp1 == temp[i])
			return true;
	}
	return false;
}

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	std::vector< int > indices;//选中点序列
	pcl::PointCloud<pcl::PointXYZ>::iterator startpoint = cloud->begin();

	if (event.getPointsIndices(indices) == -1)
		return;

	//for (int i = 0; i < indices.size(); ++i)
	//{				
	//	std::vector< int> totalIndices; //保存点索引号		
	//	//clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
	//	totalIndices.push_back(indices[i]);
	//	//cloud->erase(starindices + indices[i]);		
	//	//if (i==indices.size()-1)
	//	//{
	//	//	pcl::copyPointCloud(*cloud, *FINALcloud);
	//	//	viewer->removeAllPointClouds();
	//	//	viewer->addPointCloud(FINALcloud, "test1");
	//	//}
	//}

	//totalIndices.clear();	
	//for (int i = 0; i < indices.size(); ++i)
	//{
		//clicked_points_3d->points.push_back(cloud->points.at(indices[i]));	
		//totalIndices.push_back(indices[i]);
		//cloud->erase(startpoint +indices[i]);//删除单点使用			
		//startpoint++;
		//Sleep(10);
		//cout << indices[i] << endl;
	//}

	int indicesSize = indices.size();
	if (indicesSize < 1) 
	{
		return;
	}
	else 
	{
		//int cnt = indices[0];
		//for (int i = 0, iend = indices.size() - 1; i < iend; ++i) {
		//	for (int j = indices[i] + 1, jend = indices[i + 1]; j < jend; ++j) {
		//		cloud->points[cnt] = cloud->points[j];
		//		cnt++;
		//	}
		//}
		//for (int i = indices[indices.size() - 1]+1, iend = cloud->points.size(); i < iend; ++i) {
		//	cloud->points[cnt] = cloud->points[i];
		//	cnt++;
		//}
		//cloud->points.resize(cnt);
		 
		viewer->removeAllPointClouds();

		boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices);
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		extract.setInputCloud(cloud);
		extract.setIndices(index_ptr);
		extract.setNegative(true);
		extract.filter(*FINALcloud);

		//int cnt = cloud->points.size() - 1;
		//for (int i = 0, iend = indices.size(); i < iend; ++i) {
		//	cloud->points[indices[i]] = cloud->points[cnt];
		//	cnt--;
		//}
		//cloud->points.resize(cnt);

	}

	{
		//int cnt = cloud->points.size() - 1;
		//for (int i = 0, iend = indices.size(); i < iend; ++i) {
		//	cloud->points[indices[i]] = cloud->points[cnt];
		//	cnt--;
		//}
		//cloud->points.resize(cnt);
	}

	//for (int i = 0; i < cloudsize; i++)
	//{
	//	if (myfind(i,indices)==false)
	//	{
	//		FINALIndices.push_back(i);
	//	}
	//}


	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	//std::stringstream ss;
	//std::string cloudName;
	//ss << num++;
	//ss >> cloudName;
	//cloudName += "_cloudName";
	
	//viewer->addPointCloud(clicked_points_3d, red, cloudName);
	//viewer->removeAllPointClouds();

	viewer->removeAllPointClouds();
	viewer->addPointCloud(FINALcloud);

	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);
	//pcl::copyPointCloud(*cloud, FINALIndices, *FINALcloud);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fildColor(cloud, 0, 255, 0);
	//viewer->addPointCloud(FINALcloud, fildColor, "test");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "test");


	
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	//std::stringstream ss;
	//std::string cloudName;
	//ss << num++;
	//ss >> cloudName;
	//cloudName += "_cloudName";

	//viewer->addPointCloud(clicked_points_3d, red, cloudName);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);

	//vector<int>FINALIndices;
	//for (int i = 0; i <35947; ++i)
	//{
	//	if (myfind(i, totalIndices) == false)
	//	{
	//		FINALIndices.push_back(i);
	//	}	
	//}

	//viewer->removeAllPointClouds();
	//pcl::copyPointCloud(*cloud, FINALIndices, *FINALcloud);	
	//viewer->addPointCloud(FINALcloud);

}

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

	//for (int i = 0; i < cloud->points.size(); i++)
	//{
	//	strcloudIndices.push_back(i);
	//}
	//cloudsize=cloud->size();



	//if (pcl::io::loadPLYFile("bunny.ply", *cloud))
	//{
	//	std::cerr << "ERROR: Cannot open file " << std::endl;
	//	return;
	//}

	/**********构建点云*************/


	//startpoint = cloud->begin();

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fildColor(cloud,0,255,0);
	//viewer->addPointCloud(cloud, fildColor,"test");

	viewer->addPointCloud(cloud,"test");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "test");
	viewer->addCoordinateSystem();
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);
	
	//viewer->removeAllPointClouds();
	//viewer->addPointCloud(cloud, "test");

	//pcl::PointCloud<pcl::PointXYZ>::iterator starindices = cloud->begin();
	//cloud->erase(starindices);
	//viewer->removeAllPointClouds();
	//viewer->addPointCloud(cloud, "test");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(10);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));

	}

	////////////////////////////////②框选（要移除）的点云存在另外一个窗口viewer2中：	
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
	//pcl::PointCloud<pcl::PointXYZ>::Ptr OUTcloud(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::copyPointCloud(*cloud, totalIndices, *OUTcloud);
	//viewer2->addPointCloud(OUTcloud, "test2");
	//viewer2->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	//while (!viewer2->wasStopped())
	//{
	// viewer2->spinOnce(100);
	// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	////////////////////////////////③移除后剩余的点云存在另外一个窗口viewer3中:
	//vector<int>FINALIndices;
	//for (int i = 0; i < 35947; i++)
	//{
	//	if (myfind(i, totalIndices) == false)
	//		FINALIndices.push_back(i);
	//}
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("viewer3"));
	//pcl::PointCloud<pcl::PointXYZ>::Ptr FINALcloud(new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::copyPointCloud(*cloud, FINALIndices, *FINALcloud);
	//viewer3->addPointCloud(FINALcloud, "test3");
	//viewer3->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	//while (!viewer3->wasStopped())
	//{
	//	viewer3->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	system("pause");
}