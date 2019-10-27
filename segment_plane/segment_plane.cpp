/*
描述:
	使用CloudTools(based on PCL 1.9.1)开发
	使用自生长方法分割点云平面
Input:
	任何形式点云(.pcd)
Output:
	分割完成的点云，保存格式为plane+序号.pcd
Time:
	2019年10月24日
Note:
	若使用点云自带的法向量需要调整角度
*/

#include "../CloudTools/CloudTools.cpp" 

int main() {
	default_random_engine generator;
	string filename = "global.pcd";
	CloudTools cloud_tools;

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	pcl::PCDReader read;
	if (0 != read.read(filename, *cloud)) {
		cerr<<"File cannot be open!"<<endl;
		return -1;
	}
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
	cloud_tools.filter_voxel_grid_downsample(cloud, cloud, 0.2);
	//cloud_tools.create_cloud_with_normal(cloud, cloud_normal, 0.4);

	// 计算法向量
	if (!cloud_tools.is_contained_normal(cloud)) 
	{
		cloud_tools.create_cloud_with_normal(cloud, cloud_normal,0.4);
	}
	else 
	{
		pcl::fromPCLPointCloud2(*cloud, *cloud_normal);
	}

	// 分割平面
	vector<vector<int>> total_plane_index;
	cloud_tools.plane_segment(cloud_normal, total_plane_index, 0.8, 15, 5000);
	cout<<"分割平面数:"<<total_plane_index.size()<<endl;

	// 保存各个平面并计算各个平面的重心
	vector<pcl::PointCloud<pcl::PointXYZ>> cloud_vec(total_plane_index.size());
	vector<pcl::PointXYZ> center_point_vec(total_plane_index.size());
	Eigen::Vector4f centroid;
	for (int i = 0; i < total_plane_index.size(); ++i)
	{
		cloud_tools.index_vector_to_cloud<pcl::PointNormal, pcl::PointXYZ>
			(total_plane_index[i], *cloud_normal, cloud_vec[i]);

		pcl::compute3DCentroid(cloud_vec[i], centroid);
		center_point_vec[i].x = centroid[0];
		center_point_vec[i].y = centroid[1];
		center_point_vec[i].z = centroid[2];
	}
	
	// 通过平面重心判断是否有重叠点云
	for (int i = 0; i < center_point_vec.size(); ++i)
	{
		if (cloud_vec[i].width == 0) 
		{
			continue;
		}
		for (int j = i + 1; j < center_point_vec.size(); ++j)
		{
			if (cloud_vec[j].width == 0)
			{
				continue;
			}
			double distance = pcl::geometry::distance(center_point_vec[i], center_point_vec[j]);
			//cout << "distanc:" << distance << endl;
			if (distance < 5 ) {
				cloud_vec[i] += cloud_vec[j];
				cloud_vec[j].clear();
			}
		}
	}

	// 保存有效点云
	int plane_count = 0;
	for (int i = 0; i < cloud_vec.size(); ++i)
	{
		if(cloud_vec[i].width != 0)
		{
			string str = "plane" + std::to_string(plane_count) + ".pcd";
			cloud_tools.save_cloud_with_random_color<pcl::PointXYZ>(str, cloud_vec[i], generator);
			plane_count = plane_count + 1;
		}
	}

	cout << "最终分割平面数:" << plane_count << endl;
	return 0;
}