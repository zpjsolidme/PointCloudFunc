
//�����ļ�
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

//������
#include <pcl/point_types.h>
//KD��
#include <pcl/kdtree/kdtree_flann.h>
//������ȡ
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

//�ع�
#include <pcl/surface/gp3.h> //̰��ͶӰ���ǻ�
#include <pcl/surface/poisson.h>
//���ӻ�
#include <pcl/visualization/pcl_visualizer.h>

//���߳�
#include <boost/thread/thread.hpp>


void main()
{

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//cloud_ptr->width = 1500;
	//cloud_ptr->height = 1500;
	//cloud_ptr->is_dense = false;
	//cloud_ptr->points.resize(cloud_ptr->width*cloud_ptr->height);
	//for (int h = 0; h < cloud_ptr->height; h++)
	//{
	//	for (int w = 0; w < cloud_ptr->width; w++)
	//	{
	//		cloud_ptr->points[h*cloud_ptr->width + w].x = w*0.1f;
	//		cloud_ptr->points[h*cloud_ptr->width + w].y = h*0.1f;
	//		cloud_ptr->points[h*cloud_ptr->width + w].z = sin(w*0.1);
	//		//cloud->points[h*cloud->width + w].z = 0;
	//	}
	//}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	
	//pcl::io::loadPCDFile("lengjiao.pcd", cloud_blob);
	//pcl::io::loadPLYFile("bunny.ply", cloud_blob);
	pcl::io::loadPLYFile("new.ply", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud_ptr);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; //���߹��ƶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ���
	pcl::search::KdTree < pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd��ָ��


	//������ָ��
	tree->setInputCloud(cloud_ptr);//��Cloud����tree����
	n.setInputCloud(cloud_ptr);
	n.setSearchMethod(tree);
	n.setKSearch(8);
	n.compute(*normals);



	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_ptr, *normals, *cloud_with_normals); //�����ֶ�


	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);//���ƹ���������

	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; //�������ǻ�����
	pcl::PolygonMesh triangles; //�洢�������ǻ�������ģ��
	gp3.setSearchRadius(10); //�������ӵ�֮��������룬���������������߳���
	//gp3.setSearchRadius(10); //�������ӵ�֮��������룬���������������߳���


	// ���ø�����ֵ
	gp3.setMu(30); //30���ñ���������������ڵ����Զ����Ϊ2.5��Ϊ��ʹ�õ����ܶȵı仯-----30
	gp3.setMaximumNearestNeighbors(10); //������������������������----------100
	gp3.setMaximumSurfaceAngle(M_PI / 2); //����ĳ�㷨�߷���ƫ�������㷨�ߵ����Ƕ�45-----4
	gp3.setMinimumAngle(M_PI / 18); // �������ǻ���õ����������ڽǵ���С�ĽǶ�Ϊ10---------18
	gp3.setMaximumAngle(2 * M_PI / 6); // �������ǻ���õ����������ڽǵ����Ƕ�Ϊ120--------2*pi*3
	gp3.setNormalConsistency(false); //���øò�����֤���߳���һ��


	gp3.setInputCloud(cloud_with_normals); //�����������Ϊ�������
	gp3.setSearchMethod(tree2); //����������ʽ
	gp3.reconstruct(triangles); //�ؽ���ȡ���ǻ�	
	

	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_ptr, "y");
	
	//vtkSmartPointer<vtkDataArray> scalars;
	//fildColor.getColor(scalars);
	//vtkIdType numTuples = scalars->GetNumberOfTuples();//��ȡԪ������
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
	//viewer->addPolygonMesh(triangles);

	viewer->addPointCloud(cloud_ptr);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	system("pause");


}