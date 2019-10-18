#include<iostream>
#include<fstream>
#include<vector>

#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/kdtree/io.h>
#include<pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/io.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl/PCLHeader.h>
#include<pcl/PCLPointField.h>
using namespace std;

int main()
{
	// The original cloud pointer, Dont care about the format of file.
	pcl::PCLPointCloud2::Ptr cloud_ori(new pcl::PCLPointCloud2);
	// Only XYZ 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	// With normal vector
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
	// For show
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

	// It will add .pcd latter
	string filename = "global_shachepian";
	
	pcl::PCDReader reader;
	reader.read(filename + ".pcd", *cloud_ori);
	
	// the points cloud must be only xyz pointcloud if the fields.size() == 3
	// cout << cloud_ori->fields.size() << endl;

	// without normal vector
	// we need to calculate the normals and calculate the color
	if (cloud_ori->fields.size() == 3){
		pcl::fromPCLPointCloud2(*cloud_ori,*cloud_xyz);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud_xyz);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		ne.setRadiusSearch(0.8);
		ne.compute(*normals);
		pcl::concatenateFields(*cloud_xyz, *normals, *cloud_normal);
	}
	// with normal vector
	// the next step is color calculation
	else {
		pcl::fromPCLPointCloud2(*cloud_ori, *cloud_normal);
	}

	vector<float> 
		normal_x_vec,normal_y_vec,normal_z_vec;

	float min_nx = 999, min_ny = 999, min_nz = 999;
	float max_nx = -999, max_ny = -999, max_nz = -999;

	float curr_nx = 0, curr_ny = 0, curr_nz = 0;
	pcl::PointXYZRGB point;

	for (int i = 0; i < cloud_normal->points.size(); i++) {
		curr_nx = cloud_normal->points[i].normal_x;
		curr_ny = cloud_normal->points[i].normal_y;
		curr_nz = cloud_normal->points[i].normal_z;

		normal_x_vec.push_back(curr_nx);
		normal_y_vec.push_back(curr_ny);
		normal_z_vec.push_back(curr_nz);

		if (curr_nx > max_nx)
			max_nx = curr_nx;
		if (curr_ny > max_ny)
			max_ny = curr_ny;
		if (curr_nz > max_nz)
			max_nz = curr_nz;

		if (curr_nx < min_nx)
			min_nx = curr_nx;
		if (curr_ny < min_ny)
			min_ny = curr_ny;
		if (curr_nz < min_nz)
			min_nz = curr_nz;
		
		point.x = cloud_normal->points[i].x;
		point.y = cloud_normal->points[i].y;
		point.z = cloud_normal->points[i].z;
		cloud_rgb->push_back(point);
	}
	cout << "max_nx/ny/nz" << max_nx <<" "<< max_ny << " " << max_nz << endl;
	cout << "min_nx/ny/nz" << min_nx << " " << min_ny << " " << min_nz << endl;
	
	float
		total_nx = max_nx - min_nx,
		total_ny = max_ny - min_ny,
		total_nz = max_nz - min_nz;

	uint8_t r, g, b;
	uint32_t rgb;

	for (int i = 0; i < cloud_rgb->points.size(); i++) {
		r = (normal_x_vec[i] - min_nx) / total_nx * 255;
		g = (normal_y_vec[i] - min_ny) / total_ny * 255;
		b = (normal_z_vec[i] - min_nz) / total_nz * 255;

		rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		cloud_rgb->points[i].rgb = *reinterpret_cast<float*>(&rgb);
	}
	std::cout<<"Done"<<endl;

	pcl::io::savePCDFileBinary(filename +"_colorbynormals.pcd", *cloud_rgb);

	// visualization of point cloud
	pcl::visualization::CloudViewer viewer("cloud_rgb");
	viewer.showCloud(cloud_rgb);
	while (!viewer.wasStopped()) {}

	return 0;
}
