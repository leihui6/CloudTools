#include "../CloudTools/CloudTools.cpp"

int main()
{
	CloudTools cloud_tools;
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	
	string filename = "connection_global";
	pcl::PCDReader reader;
	
	if (0 != reader.read(filename + ".pcd", *cloud)) {
		cerr << "File cannot be open!" << endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());

	cloud_tools.color_using_normal(cloud, cloud_rgb);

	pcl::io::savePCDFile(filename+"_colorbynormal.pcd",*cloud_rgb);

	return 0;
}
