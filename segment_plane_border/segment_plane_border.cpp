#include"../CloudTools/CloudTools.cpp"

int main() {
	CloudTools cloud_tools;
	string filename = "plane0";

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	pcl::PCDReader read;
	if (0 != read.read(filename + ".pcd", *cloud)) {
		cerr << "File cannot be open!" << endl;
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_tools.filter_voxel_grid_downsample(cloud, cloud, 0.2);

	pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);
	
	

	vector<int> border_group;
	// 某个点如果360度上有一个度没有则为边界点
	cloud_tools.border_segment(border_group, cloud_xyz, 50);

	pcl::PointCloud<pcl::PointXYZ>::Ptr border_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_tools.index_vector_to_cloud<pcl::PointXYZ, pcl::PointXYZ>
		(border_group, *cloud_xyz, *border_cloud);

	pcl::io::savePCDFile(filename + "border_before_filter.pcd", *border_cloud);

	cloud_tools.filter_statistical_outlier_removal(border_cloud, border_cloud, 6, 1);

	pcl::io::savePCDFile(filename + "border_after_filter.pcd", *border_cloud);

	vector<vector<int>> border_cluster_index;
	cloud_tools.cloud_cluster(border_cluster_index, border_cloud, 25);
	cout << "检测边界的数量" << border_cluster_index.size() << endl;

	for (int i = 0; i < border_cluster_index.size();i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr single_border_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_tools.index_vector_to_cloud<pcl::PointXYZ, pcl::PointXYZ>
			(border_cluster_index[i], *border_cloud, *single_border_cloud);
		pcl::io::savePCDFile(filename + "border" + std::to_string(i) + ".pcd", *single_border_cloud);
	}

	return 0;
}