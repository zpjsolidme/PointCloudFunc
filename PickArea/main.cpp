#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <sstream>


using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr < pcl::visualization::PCLVisualizer >viewer(new pcl::visualization::PCLVisualizer("viewer"));

//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);


int readEcdFile(char* file, int *rows, int *cols, float* xInterval, float* yInterval, std::vector<float> *xs, std::vector<float> *ys, std::vector<float> *zs);



vector<int> PickAreaPara;//区域选取参数

//区域选择  
pcl::PointCloud<pcl::PointXYZ>::Ptr PickArea_Pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
int PickCloudNum = 0;//选取点云计数的点云块


//点选择
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ PointUse;
typedef pcl::PointCloud<PointT> PointCloudT;

struct PickPoints
{
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


double sum;
double sum1;

std::ofstream outFile;

void PickPoint_CallBack(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct PickPoints* data = (struct PickPoints*)args;
	if (event.getPointIndex() == -1)
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


	//std::cout
	//	<< current_point.x << " "
	//	<< current_point.y << " "
	//	<< current_point.z << std::endl;
	//char cpz[20];
	//sprintf(cpz, "%.2f", current_point.z);
	//string pz = cpz;
	//viewer->addText(pz, 100 * current_point.x, 100 * current_point.y, cpz, 0);


}

void PickArea_CallBack(const pcl::visualization::AreaPickingEvent& event, void* args)
{

	std::vector<int> indices;
	if (event.getPointsIndices(indices)==-1)
	{
		return;
	}
	for (int i = 0; i < indices.size(); ++i)
	{

		PickArea_Pointcloud->points.push_back(cloud->points.at(indices[i]));
		outFile << indices[i] << ',' << PickArea_Pointcloud->points[i].x << ',' << PickArea_Pointcloud->points[i].y << ',' << PickArea_Pointcloud->points[i].z << endl;
		//cloud->erase(indeices[i]);
	}

	////求均方高度
	//double Sq;//均方高度
	//double Sku;//翘度值;
	//double A;//面积
	//int N=PickArea_Pointcloud->size();
	//A = N*0.05*0.05;
	//for (int i = 0; i < N; i++)
	//{
	//	sum+=(PickArea_Pointcloud->points[i].z)*(PickArea_Pointcloud->points[i].z)*0.05*0.05;	
	//}
	//Sq = sqrt(sum / A);
	//
	//for (int i = 0; i < N; i++)
	//{
	//	sum1 += (PickArea_Pointcloud->points[i].z)*
	//			(PickArea_Pointcloud->points[i].z)*
	//			(PickArea_Pointcloud->points[i].z)*
	//			(PickArea_Pointcloud->points[i].z)*0.05*0.05;
	//	cout << (PickArea_Pointcloud->points[i].z) << endl;
	//}
	//Sku = sum1 / (A*Sq*Sq*Sq*Sq);
	//cout << Sku<<endl;

	std::stringstream ss;
	std::string cloudName;
	ss << PickCloudNum++;
	ss >> cloudName;
	cloudName += "_cloudName";


		
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> Color(PickArea_Pointcloud, 255, 0, 0);//调节选中点单一颜色
																										  																										  
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> Color (PickArea_Pointcloud, "z");//按坐标位置渲染
	//viewer->removeAllPointClouds();
	//viewer->addPointCloud(PickArea_Pointcloud, Color, cloudName);

	//保存选中的点
	//viewer->removeAllPointClouds();
	viewer->addPointCloud(PickArea_Pointcloud, Color, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloudName);//选中点大小
	
}

void main()
{

	//outFile.open("data.csv", ios::out);
	//outFile << "indices" << ',' << "X" << ',' << "Y" << ',' << "Z" << endl;

	//std::string CloudPath = "C:\\Users\\peijiang zhang.O-NET\\Desktop\\0102量规_\\0angle_1mm.ecd";
	//int rows;
	//int cols;
	//float xInterval;
	//float yInterval;
	//std::vector<float> xs;
	//std::vector<float> ys;
	//std::vector<float> zs;
	//readEcdFile((char*)CloudPath.c_str(), &rows, &cols, &xInterval, &yInterval, &xs, &ys, &zs);
	//
	//pcl::PointCloud<pcl::PointXYZ>& cloud1 = *cloud;
	//cloud1.width = rows*cols;
	//cloud1.height = 1;
	//cloud1.is_dense = false;
	//cloud1.points.resize(cloud1.width*cloud1.height);
	//for (int i = 0; i<cloud1.points.size(); i++)
	//{
	//	cloud1.points[i].x = xs[i];
	//	cloud1.points[i].y = ys[i];
	//	cloud1.points[i].z = zs[i];
	//}
	//pcl::io::savePCDFileASCII("0angle_1mm.pcd", cloud1);


	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vierwe"));

	cloud->width = 155;
	cloud->height = 300;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width*cloud->height);
	for (int h = 0; h < cloud->height; h++)
	{
		for (int w = 0; w < cloud->width; w++)
		{
			cloud->points[h*cloud->width + w].x = w*0.01f;
			cloud->points[h*cloud->width + w].y = h*0.01f;
			cloud->points[h*cloud->width + w].z = sin(w*0.02);
			//cloud->points[h*cloud->width + w].z = 10;
		}
	}


	viewer->addPointCloud(cloud, "cloud");
	viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	viewer->setBackgroundColor(1.0, 2.0, 3.0);
	viewer->registerAreaPickingCallback(PickArea_CallBack, (void*)&cloud);//区域选择回调
	viewer->addCoordinateSystem();

	//添加点选取窗体回调
	struct PickPoints cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(PickPoint_CallBack, (void*)&cb_args);


	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10));
	}

	system("pause");

}

int readEcdFile(char* file, int *rows, int *cols, float* xInterval, float* yInterval, std::vector<float> *xs, std::vector<float> *ys, std::vector<float> *zs)
{
	if ((xs == NULL) || (ys == NULL) || (zs == NULL) || (file == NULL) || (rows == NULL) || (cols == NULL) ||
		(xInterval == NULL) || (yInterval == NULL)) {
		return -1;
	}
	int &rsltRows = *rows;
	int &rsltCols = *cols;
	float &rsltXInterval = *xInterval;
	float &rsltYInterval = *yInterval;
	std::vector<float> &rsltXs = *xs;
	std::vector<float> &rsltYs = *ys;
	std::vector<float> &rsltZs = *zs;

	float zoomRate = 0.5f;
	std::vector<int> data;

	ifstream fp(file, std::ios_base::binary);
	if (!fp) {
		return -1;
	}

	int headSize = 2560;
	int *headData = new int[headSize];

	fp.read((char*)headData, headSize * sizeof(int));

	rsltCols = headData[1];
	rsltRows = headData[2];
	rsltXInterval = ((double*)(headData + 4))[0];
	rsltYInterval = ((double*)(headData + 4))[1];
	delete[] headData;

	data.resize(rsltCols * rsltRows);
	fp.read((char*)&data[0], sizeof(int) * data.size());
	fp.close();

	rsltXs.resize(rsltRows*rsltCols);
	rsltYs.resize(rsltRows*rsltCols);
	rsltZs.resize(rsltRows*rsltCols);

	int xCnt = NULL;
	int yCnt = NULL;

	for (int i = 0, iend = rsltZs.size(); i<iend; ++i) {

		rsltXs[i] = rsltXInterval*xCnt;
		rsltYs[i] = rsltYInterval*yCnt;
		rsltZs[i] = (float)data[i] / 100000.0;
		if (++xCnt == rsltCols)
		{
			xCnt = NULL;
			yCnt++;
		}
	}

	return 0;


}


//int PeckTest(vector<float> &points, float &xIntervect, float &yIntervect, int &pointscout, float &volume, float &area);
//int PeckTest(vector<float> &points, float &xIntervect, float &yIntervect, int &pointscout, float &volume,float &area)
//{
//	
//
//
//
//
//	return 0;
//}
