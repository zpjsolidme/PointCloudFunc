 //点云文件
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

//点类型
#include <pcl/point_types.h>
//KD树
#include <pcl/kdtree/kdtree_flann.h>
//特征提取
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

//重构
#include <pcl/surface/gp3.h> //贪婪投影三角化
#include <pcl/surface/poisson.h>
//可视化
#include <pcl/visualization/pcl_visualizer.h>

//多线程
#include <boost/thread/thread.hpp>


void main()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	
	pcl::io::loadPCDFile("lengjiao.pcd", cloud_blob);

	pcl::io::savePLYFile("new",cloud_blob);


	pcl::fromPCLPointCloud2(cloud_blob, *cloud_ptr);
	

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; //法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线
	pcl::search::KdTree < pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针

	//定义树指针
	tree->setInputCloud(cloud_ptr);//用Cloud构建tree对象
	n.setInputCloud(cloud_ptr);
	n.setSearchMethod(tree);
	n.setKSearch(8);
	n.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_ptr, *normals, *cloud_with_normals); //连接字段


	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);//点云构建搜索树

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; //定义三角化对象
	pcl::PolygonMesh triangles; //存储最终三角化的网络模型
	gp3.setSearchRadius(30); //设置连接点之间的最大距离，（即是三角形最大边长）
	//gp3.setSearchRadius(10); //设置连接点之间的最大距离，（即是三角形最大边长）

	// 设置各参数值
	gp3.setMu(30); //30设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化-----2.5
	gp3.setMaximumNearestNeighbors(10); //设置样本点可搜索的邻域个数----------100
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45
	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120
	gp3.setNormalConsistency(false); //设置该参数保证法线朝向一致


	gp3.setInputCloud(cloud_with_normals); //设置输入点云为有向点云
	gp3.setSearchMethod(tree2); //设置搜索方式
	gp3.reconstruct(triangles); //重建提取三角化	
	

	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_ptr, "y");
	
	//vtkSmartPointer<vtkDataArray> scalars;
	//fildColor.getColor(scalars);
	//vtkIdType numTuples = scalars->GetNumberOfTuples();//获取元组总数
	//for (vtkIdType tupleIdx = 0; tupleIdx < numTuples; tupleIdx++)
	//{
	//	double *tuple = scalars->GetTuple(tupleIdx);
	//	for (int j = 0; j < 3; j++)
	//	{
	//		double var = tuple[j];
	//		std::cout << var << std::endl;
	//	}
	//}
		
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);

	//viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, fildColor, "sample");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample");
	viewer->addPolygonMesh(triangles);

	//viewer->addPointCloud(cloud_ptr);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	system("pause");


}