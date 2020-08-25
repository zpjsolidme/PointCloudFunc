#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\visualization\pcl_visualizer.h>
#include <pcl\visualization\cloud_viewer.h>
#include <vector>

#define Pi 3.1415926;

using namespace std;

typedef pcl::PointXYZ Point;
typedef pcl::PointNormal PointNormal;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vierwe"));

void PaintCircle(PointNormal &planeNormal, Point &centerPoint, double R,char &name)
{
	//圆心
	int cx = centerPoint.x, cy = centerPoint.y, cz = centerPoint.z;

	//圆半径
	double r = R;

	//圆所在平面的法向量
	double nx = planeNormal.x, ny = planeNormal.y, nz = planeNormal.z;

	//圆所在平面的一个向量
	double ux = ny,  uy = -nx, uz = 0;

	//圆所在平面内的正交向量
	double vx = nx*nz, vy = ny*nz, vz = -nx*nx - ny*ny;
	

	double sqrtU = sqrt(ux*ux + uy*uy + uz*uz);
	double sqrtV = sqrt(vx*vx + vy*vy + vz*vz);

	double ux_ = (1 / sqrtU)*ux;
	double uy_ = (1 / sqrtU)*uy;
	double uz_ = (1 / sqrtU)*uz;

	double vx_ = (1 / sqrtU)*vx;
	double vy_ = (1 / sqrtU)*vy;
	double vz_ = (1 / sqrtU)*vz;

	double xi, yi, zi;
	double t = 0;
	double angle = (t / 180.0)*Pi;


	vector<double> x, y, z;

	while (t < 360.0)
	{
		xi = cx + r*(ux_*cos(angle) + vx_*sin(angle));
		yi = cy + r*(uy_*cos(angle) + vy_*sin(angle));
		zi = cz + r*(uz_*cos(angle) + vz_*sin(angle));
		x.push_back(xi);
		y.push_back(yi);
		z.push_back(zi);


		t = t + 1;
		angle = (t / 180.0)*Pi;
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr theroyCirclePoints(new pcl::PointCloud<pcl::PointXYZ>);

	theroyCirclePoints->resize(x.size());
	for (int i = 0; i < x.size(); i++)
	{
		(*theroyCirclePoints)[i].x = x[i];
		(*theroyCirclePoints)[i].y = y[i];
		(*theroyCirclePoints)[i].z = z[i];
		cout << x[i] << "," << y[i] << "," << z[i] << endl;

	}

	//viewer->removePointCloud("theroyCircleCloud");
	viewer->addCoordinateSystem();


	//viewer->addPointCloud(theroyCirclePoints, "ads");
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> theroryCircleCloud_color(theroyCirclePoints, 255, 0, 0);

	////点云颜色渲染
	viewer->addPointCloud(theroyCirclePoints, theroryCircleCloud_color, &name);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5, &name);

	viewer->resetCamera();


}

void main()
{
	PointNormal YZplane;
	YZplane.x = 1;
	YZplane.y = 0;
	YZplane.z = 0;

	Point CenterPoint;
	CenterPoint.x = 0;
	CenterPoint.y = 0;
	CenterPoint.z = 0;

	char *name1 = "111";
	char *name2 = "222";

	PaintCircle(YZplane, CenterPoint, 2.0,*name1);

	PaintCircle(YZplane, CenterPoint,10, *name2);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	system("pause");

}
