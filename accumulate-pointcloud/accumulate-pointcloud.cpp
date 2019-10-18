
// based on pcl 1.9.1
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main(int argc, char** argv)
{
	const int SIZE = 5;

	pcl::PointCloud<pcl::PointXYZ> cloud[SIZE];

	for (int i = 0; i < SIZE; i++) {
		if (pcl::io::loadPCDFile<pcl::PointXYZ>("1-"+std::to_string(i+1)+".pcd", cloud[i]) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}
		std::cout << "Loaded "
			<< cloud[i].width * cloud[i].height
			<< " data points from test_pcd.pcd with the following fields: "
			<< std::endl;
	}

	pcl::PointCloud<pcl::PointXYZ> allinone;
	for (int i = 0; i < SIZE; i++) {
		allinone += cloud[i];
	}
	pcl::io::savePCDFile("view1_allinone.pcd",allinone);

	return (0);
}