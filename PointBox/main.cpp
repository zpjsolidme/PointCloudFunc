#include <string>
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <Eigen\Core>
#include <pcl\common\transforms.h>
#include <pcl\common\common.h>
#include <pcl\common\pca.h>
#include <pcl\visualization\pcl_visualizer.h>



using namespace std;
typedef pcl::PointXYZ PointType;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vierwe"));

int main()
{
	cloud->width = 50;
	cloud->height = 50;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width*cloud->height);
	for (int h = 0; h < cloud->height; h++)
	{
		for (int w = 0; w < cloud->width; w++)
		{
			//cloud->points[h*cloud->width + w].x = w;
			//cloud->points[h*cloud->width + w].y = h;
			//cloud->points[h*cloud->width + w].z = 0;
			cloud->points[h*cloud->width + w].x = w*0.1f;
			cloud->points[h*cloud->width + w].y = h*0.1f;
			cloud->points[h*cloud->width + w].z = sin(w*0.1);
			
		}
	}


	//viewer->addPointCloud(cloud, "cloud");
	//viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");
	//viewer->addCoordinateSystem();



	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f>  eigen_solver(covariance, Eigen::ComputeEigenvectors);

	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
	std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//}


	//另一种计算点云协方差矩阵特征值特征向量的方法
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PCA<pcl::PointXYZ> pca;
	//pca.setInputCloud(cloud);
	//pca.project(*cloud, *cloudPCAprojection);
	//std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征值
	//std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
	//




	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose(); //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//-R*t
	tm_inv = tm.inverse();

	std::cout << "变换矩阵tm(4x4):\n" << tm << std::endl;
	std::cout << "逆变矩阵tm'(4x4):\n" << tm_inv << std::endl;


	//viewer->addPointCloud(cloud, "cloud");
	//viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample");
	
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);

	PointType min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

	std::cout << "型心c1(3x1):\n" << c1 << std::endl;

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);

	Eigen::Vector3f whd, whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3; //点云平均尺度，用于设置主方向箭头大小

	std::cout << "width1=" << whd1(0) << endl;
	std::cout << "heght1=" << whd1(1) << endl;
	std::cout << "depth1=" << whd1(2) << endl;
	std::cout << "scale1=" << sc1 << endl;

	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f bboxT1(c1);

	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f bboxT(c);

	//变换到原点的点云主方向
	PointType op;
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	PointType pcaX;
	pcaX.x = sc1 * px(0);
	pcaX.y = sc1 * px(1);
	pcaX.z = sc1 * px(2);
	PointType pcaY;
	pcaY.x = sc1 * py(0);
	pcaY.y = sc1 * py(1);
	pcaY.z = sc1 * py(2);
	PointType pcaZ;
	pcaZ.x = sc1 * pz(0);
	pcaZ.y = sc1 * pz(1);
	pcaZ.z = sc1 * pz(2);

	//初始点云的主方向
	PointType cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
	PointType pcX;
	pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
	PointType pcY;
	pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
	PointType pcZ;
	pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;

	//visualization
	pcl::visualization::PCLVisualizer viewer;
	pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); //转换到原点的点云相关
	viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
	viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");
	
	//viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
	//viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
	//viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

	pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 0, 0); //输入的初始点云相关
	viewer.addPointCloud(cloud, color_handler, "cloud");
	viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");

	//viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	//viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	//viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
	viewer.addCoordinateSystem(0.5f*sc1);
	viewer.setBackgroundColor(1.0, 1.0, 1.0);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}

	system("pause");

}

