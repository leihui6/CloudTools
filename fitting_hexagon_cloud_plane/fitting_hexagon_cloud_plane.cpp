#include"../CloudTools/CloudTools.cpp"

int main(int argc, char** argv)
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	string filename = "plane0border0";

	pcl::PCDReader read;
	if (0 != read.read(filename + ".pcd", *cloud)) {
		cerr << "File cannot be open!" << endl;
		return -1;
	}
	CloudTools cloud_tools;

	cloud_tools.filter_voxel_grid_downsample(cloud, cloud, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2<pcl::PointXYZ>(*cloud, *cloud_xyz);

	pcl::PointCloud<pcl::PointXYZ>::Ptr std_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ center_point;

	float radius;

	cloud_tools.find_center_point3d(cloud_xyz, center_point);
	cloud_tools.fitting_hexagon_cloud_plane(cloud_xyz, std_cloud, center_point, radius);
	
	pcl::io::savePCDFile("std_hexagon_cloud.pcd",*std_cloud);

	return 0;
}