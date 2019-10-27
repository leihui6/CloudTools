#include "../CloudTools/CloudTools.cpp"

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	std::string filename = "global_downsampled";

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename+".pcd", *cloud) == -1) //* load the file
	{
		cerr<<"Cannot open file"<<endl;
		return (-1);
	}

	CloudTools cloud_tools;

	cloud_tools.color_using_resolution(cloud, cloud_rgb);

	pcl::io::savePCDFile(filename+"_density.pcd", *cloud_rgb);

	return 0;
}
