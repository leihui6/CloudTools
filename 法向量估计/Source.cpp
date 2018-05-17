#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
int
main(int argc, char** argv)
{
	//加载点云模型
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PCLPointCloud2 cloud_blob;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("pcd_cloud1_grid.pcd", *cloud) == -1){

		PCL_ERROR("Could not read file \n");
	}
	//* the data should be available in cloud

	// Normal estimation*
	//法向计算
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//为kdtree添加点云数据
	tree->setInputCloud(cloud);

	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	//点云法向计算时，需要搜索的近邻点大小
	n.setKSearch(20);
	//开始进行法向计算
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	//将点云数据与法向信息拼接
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	/*图形显示模块*/
	//显示设置
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	//设置背景色
	viewer->setBackgroundColor(0, 0, 0.7);

	//设置点云颜色，该处为单一颜色设置
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);

	//添加需要显示的点云数据
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");

	//设置点显示大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	//添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，５表示法向长度。
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 5, "normals");

	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	// Finish
	return (0);
}