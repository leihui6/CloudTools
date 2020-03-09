#pragma once
#include "CloudTools.h"

void Line_func::convert_to_vectorxf(Eigen::VectorXf& vx)
{
	vx.setZero();

	vx[0] = v[0];
	vx[1] = v[1];
	vx[2] = v[2];
	vx[3] = v[3];
	vx[4] = v[4];
	vx[5] = v[5];
}

void Line_func::convert_to_vector4f(Eigen::Vector4f & line_pt, Eigen::Vector4f & line_dir)
{
	line_pt[0] = v[0];
	line_pt[1] = v[1];
	line_pt[2] = v[2];
	line_pt[3] = 0;

	line_dir[0] = v[3];
	line_dir[1] = v[4];
	line_dir[2] = v[5];
	line_dir[3] = 0;
}

void Line_func::get_direction(Eigen::Vector3f & line_dir)
{
	line_dir[0] = v[3];
	line_dir[1] = v[4];
	line_dir[2] = v[5];
}

void Line_func::convert_to_cylinder_func(Cylinder_func & cylinder_func, float r)
{
	cylinder_func.set_value(v[0], v[1], v[2], v[3], v[4], v[5], r);

}

CloudTools::CloudTools() 
{
	m_plane_segment_angle_threshold = 15;
	m_default_search_radius = 0.8;
	m_border_search_radius = m_default_search_radius;
	m_plane_segment_search_radius = m_default_search_radius;
	// 边界聚类的搜索半径
	m_cloud_cluster_search_radius = m_default_search_radius;
	m_color_using_resolution_search_radius = m_default_search_radius;
}

CloudTools::~CloudTools() 
{

}

double CloudTools::vector_angle_360(pcl::PointXYZ& p1, pcl::PointXYZ& p2) 
{
	normalize(p1,p2);
	Eigen::Vector3d v1 = Eigen::Vector3d(p1.x, p1.y, p1.z);
	Eigen::Vector3d v2 = Eigen::Vector3d(p2.x, p2.y, p2.z);
	double radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
	if (v1.cross(v2).z() < 0) {
		radian_angle = 2 * M_PI - radian_angle;
	}
	return radian_angle * 180 / M_PI;
}


// 计算两个向量的夹角[0,180]
float CloudTools::vector_angle_180(pcl::PointXYZ& p1, pcl::PointXYZ& p2)
{
	normalize(p1, p2);
	float cos_angle_value = (p1.x * p2.x + p1.y * p2.y + p1.z * p2.z) /
		(sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z) * sqrt(p2.x * p2.x + p2.y * p2.y + p2.z * p2.z));

	if (cos_angle_value > 0)
		return radian2angle(acos(cos_angle_value));
	else
		return 180 - radian2angle(acos(abs(cos_angle_value)));
}

Eigen::Matrix4f CloudTools::rotation_matrix(const pcl::PointXYZ & p, Eigen::Vector3f & vv, const float t)
{
	vv.normalize();
	float u = vv[0];
	float v = vv[1];
	float w = vv[2];
	float a = p.x;
	float b = p.y;
	float c = p.z;

	Eigen::Matrix4f mat;
	mat << u * u + (v*v + w * w)*cos(t), u*v*(1 - cos(t)) - w * sin(t), u*w*(1 - cos(t)) + v * sin(t), (a*(v*v + w * w) - u * (b*v + c * w))*(1 - cos(t)) + (b*w - c * v)*sin(t),
		u*v*(1 - cos(t)) + w * sin(t), v*v + (u*u + w * w)*cos(t), v*w*(1 - cos(t)) - u * sin(t), (b*(u*u + w * w) - v * (a*u + c * w))*(1 - cos(t)) + (c*u - a * w)*sin(t),
		u*w*(1 - cos(t)) - v * sin(t), v*w*(1 - cos(t)) + u * sin(t), w*w + (u*u + v * v)*cos(t), (c*(u*u + v * v) - w * (a*u + b * v))*(1 - cos(t)) + (a*v - b * u)*sin(t),
		0, 0, 0, 1;
	return mat;
}

float CloudTools::vector_angle_180(Eigen::Vector3f& p1, Eigen::Vector3f& p2)
{
	p1.normalize();
	p2.normalize();
	float cos_angle_value = p1.dot(p2);
	//double cos_angle_value = (p1.x * p2.x + p1.y * p2.y + p1.z * p2.z) /
	//	(sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z) * sqrt(p2.x * p2.x + p2.y * p2.y + p2.z * p2.z));
	if (cos_angle_value > 0)
		return radian2angle(acos(cos_angle_value));
	else
		return 180 - radian2angle(acos(abs(cos_angle_value)));
}

void CloudTools::normalize(pcl::PointXYZ & p1, pcl::PointXYZ & p2)
{
	float p_t = sqrt(p1.x *p1.x + p1.y *p1.y + p1.z *p1.z);
	p1.x = p1.x / p_t;
	p1.y = p1.y / p_t;
	p1.z = p1.z / p_t;

	p_t = sqrt(p2.x *p2.x + p2.y *p2.y + p2.z *p2.z);
	p2.x = p2.x / p_t;
	p2.y = p2.y / p_t;
	p2.z = p2.z / p_t;
}

double CloudTools::radian2angle(double radian) {
	return 180 / M_PI * radian;
}
double CloudTools::angle2radian(double angle) {
	return M_PI * angle / 180;
}

// 统计滤波
void CloudTools::filter_statistical_outlier_removal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
	float mean_k, float std_thresh) {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(input_cloud);
	sor.setMeanK(mean_k);
	sor.setStddevMulThresh(std_thresh);
	sor.filter(*output_cloud);
}

void CloudTools::filter_voxel_grid_downsample(
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
	float leaf_size)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(input_cloud);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size); 
	sor.filter(*output_cloud);           
}

void CloudTools::filter_voxel_grid_downsample(
	pcl::PCLPointCloud2::Ptr input_cloud,
	pcl::PCLPointCloud2::Ptr output_cloud,
	float leaf_size)
{
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(input_cloud);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*output_cloud);
}

// 判断点云是否包含法向量
bool CloudTools::is_contained_normal(pcl::PCLPointCloud2::Ptr cloud) {
	int normal_count = 0;
	for (int i = 0; i != cloud->fields.size(); i++) {
		if (cloud->fields[i].name.find("normal_x") != string::npos ||
			cloud->fields[i].name.find("normal_y") != string::npos ||
			cloud->fields[i].name.find("normal_z") != string::npos) {
			normal_count++;
		}
	}
	return (normal_count == 3) ? true : false;
}

void CloudTools::create_cloud_with_normal(
	pcl::PCLPointCloud2::Ptr cloud,
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
	float search_radius) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud, *cloud_xyz);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_xyz);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(search_radius);
	ne.compute(*normal);
	pcl::concatenateFields(*cloud_xyz, *normal, *cloud_normal);
}

void CloudTools::create_cloud_normal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normal,
	float search_radius) 
{
	//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(search_radius);
	ne.compute(*normal);
}

void CloudTools::create_cloud_normal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<int>& process_index,
	pcl::PointCloud<pcl::Normal>::Ptr normal,
	float search_radius)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(search_radius);
	ne.setIndices(boost::make_shared<vector<int>>(process_index));
	ne.compute(*normal);
}

void CloudTools::create_cloud_with_normal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
	float search_radius) {
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(search_radius);
	ne.compute(*normal);
	pcl::concatenateFields(*cloud, *normal, *cloud_normal);
}



void CloudTools::segment_plane_DFS(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normal,
	list<vector<int>> &total_plane_index,
	vector<int> & rest_point_index,
	float search_radius,
	float angle_threshold,
	int group_number_threshold,
	float dis_same_plane,
	size_t requested_plane_size
	)
{
	m_plane_segment_search_radius = search_radius;

	m_plane_segment_angle_threshold = angle_threshold;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	vector<int> single_plane_index;
	vector<bool> visited(cloud->points.size(), false);

	for (int i = 0; i < visited.size(); ++i) 
	{
		if (visited[i] == false) 
		{
			single_plane_index.push_back(i);
			visited[i] = true;

			plane_segment_recursion(i, i, cloud, normal, kdtree, single_plane_index, visited);

			if (single_plane_index.size() > group_number_threshold)
			{
				total_plane_index.push_back(single_plane_index);
			}
			else
			{
				rest_point_index.insert(rest_point_index.end(), single_plane_index.begin(), single_plane_index.end());
			}
			single_plane_index.clear();
		}
	}
	m_segment_plane_count = 0;
	//Okey, Get all planes no care if contains the same planes
	//cout << "total_plane_index.size():" << total_plane_index.size() << endl;
	//cout << "center_point_vec.size()" << center_point_vec.size() << endl;

	combine_multi_points(cloud, total_plane_index, dis_same_plane);

	// 对分割的结果按照点集数量大小排序
	vector<int> group_size_vec;
	std::list<vector<int>>::iterator it1;
	for (it1 = total_plane_index.begin(); it1 != total_plane_index.end(); ++it1)
	{
		group_size_vec.push_back(it1->size());
	}
	vector<size_t> order_index;

	special_order(group_size_vec, order_index);

	//for (auto i : total_plane_index) cout <<i.size() <<" ";
	//cout << endl;

	list_order_by_vector_index(total_plane_index, order_index);

	//for (auto i : total_plane_index) cout << i.size() << " ";
	//cout << endl;

	// 当分割出来的平面个数 大于 需要的平面个数
	if (requested_plane_size > total_plane_index.size())
	{
		cerr << "[error] requested_plane_size > total_plane_index.size()" << endl;
		return;
	}
	else if (requested_plane_size < total_plane_index.size())
	{
		cerr << "[warning] requested_plane_size < total_plane_index.size()" << endl;
	}

	// plane_size < total_plane_index.size()
	size_t rest_n = total_plane_index.size() - requested_plane_size;
	for (size_t i = 0; i < rest_n; ++i)
	{
		total_plane_index.pop_back();
	}
	//for (auto i : total_plane_index) cout << i.size() << " ";
	//cout << endl;
}

void CloudTools::segment_plane_BFS(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normal,
	list<vector<int>> &total_plane_index,
	vector<int> & rest_point_index,
	float search_radius,
	float angle_threshold,
	int group_number_threshold,
	float dis_same_plane,
	size_t requested_plane_size)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	vector<bool> visited(cloud->points.size(), false);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (visited[i] == false)
		{
			vector<int> queue;

			queue.push_back(i);

			visited[i] = true;

			pcl::Normal& n_p1 = normal->points[i];

			// find the same points on plane
			for (size_t j = 0; j < queue.size(); j++)
			{
				pcl::PointXYZ &p = cloud->points[queue[j]];
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				int number = kdtree.radiusSearch(p, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

				if (number > 0)
				{
					for (size_t s = 1; s < pointIdxRadiusSearch.size(); ++s)
					{
						int tar_i = pointIdxRadiusSearch[s];

						pcl::Normal &n_p2 = normal->points[tar_i];

						if (visited[tar_i] == false && is_on_the_same_plane(n_p1, n_p2, angle_threshold))
						{
							queue.push_back(tar_i);

							visited[tar_i] = true;
						}
					}
				}
			}
			if (queue.size() > group_number_threshold)
			{
				total_plane_index.push_back(queue);
			}
			else
			{
				rest_point_index.insert(rest_point_index.end(), queue.begin(), queue.end());
			}
		}
	}

	combine_multi_points(cloud, total_plane_index, dis_same_plane);

	// 对分割的结果按照点集数量大小排序
	vector<int> group_size_vec;
	std::list<vector<int>>::iterator it1;
	for (it1 = total_plane_index.begin(); it1 != total_plane_index.end(); ++it1)
	{
		group_size_vec.push_back(it1->size());
	}
	vector<size_t> order_index;

	special_order(group_size_vec, order_index);

	//for (auto i : total_plane_index) cout <<i.size() <<" ";
	//cout << endl;

	list_order_by_vector_index(total_plane_index, order_index);

	//for (auto i : total_plane_index) cout << i.size() << " ";
	//cout << endl;

	// 当分割出来的平面个数 大于 需要的平面个数
	if (requested_plane_size > total_plane_index.size())
	{
		cerr << "[warning] requested_plane_size > total_plane_index.size()" << endl;
		return;
	}
	else if (requested_plane_size < total_plane_index.size())
	{
		cerr << "[warning] requested_plane_size < total_plane_index.size()" << endl;
	}

	// plane_size < total_plane_index.size()
	size_t rest_n = total_plane_index.size() - requested_plane_size;
	for (size_t i = 0; i < rest_n; ++i)
	{
		total_plane_index.pop_back();
	}
	//for (auto i : total_plane_index) cout << i.size() << " ";
	//cout << endl;
}

void CloudTools::combine_multi_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, list<vector<int>>& points, float distance)
{
	// save the center point of every plane.
	vector<pcl::PointXYZ> center_point_vec;

	std::list<vector<int>>::iterator it;
	for (it = points.begin(); it != points.end(); ++it)
	{
		pcl::PointXYZ center_point;

		find_center_point3d<pcl::PointXYZ>(*cloud, center_point, *it);

		center_point_vec.push_back(center_point);
	}

	//remove the same plane
	std::list<vector<int>>::iterator it1, it2;
	int i_1 = 0, i_2 = 0;
	for (it1 = points.begin();
		it1 != points.end();
		++it1, ++i_1)
	{
		double dis = 0.0;

		for (it2 = points.begin(); it2 != points.end();)
		{
			if (it1 == it2)
			{
				++it2;
				++i_2;
				continue;
			}

			dis = pcl::geometry::distance(
				center_point_vec[i_1],
				center_point_vec[i_2]);

			if (dis < distance)
			{
				it1->insert(it1->end(), it2->begin(), it2->end());
				it2 = points.erase(it2);
				center_point_vec.erase(center_point_vec.begin() + i_2);

				//cout << "total_plane_index.size()" << total_plane_index.size() << endl;
				//cout << "center_point_vec.size()" << center_point_vec.size() << endl;
				continue;
			}
			++it2;
			++i_2;
		}
		i_2 = 0;
	}
}

void CloudTools::color_using_normal(
	pcl::PCLPointCloud2::Ptr cloud, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb, 
	bool force_to_normal,
	float radius)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>());
	if (force_to_normal || !is_contained_normal(cloud))
	{
		create_cloud_with_normal(cloud, cloud_normal, radius);
	}
	else 
	{
		pcl::fromPCLPointCloud2(*cloud, *cloud_normal);
	}

	color_using_normal_com(cloud_normal, cloud_rgb);

	standardize_not_organized_cloud_header(*cloud_rgb);
}

void CloudTools::color_using_normal(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb, 
	float radius)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(radius);
	ne.compute(*normal);
	pcl::concatenateFields(*cloud, *normal, *cloud_normal);
	
	color_using_normal_com(cloud_normal, cloud_rgb);

	standardize_not_organized_cloud_header(*cloud_rgb);
}

void CloudTools::color_using_resolution(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
	float radius)
{
	vector<int> near_points(cloud->points.size(), 0);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	for (int i = 0; i < cloud->size(); i++) {
		if (kdtree.radiusSearch(cloud->points[i], 
			radius,
			pointIdxRadiusSearch, 
			pointRadiusSquaredDistance) > 0)
		{
			near_points[i] = pointIdxRadiusSearch.size();
		}
	}
	auto min_max = std::minmax_element(near_points.begin(), near_points.end());

	float 
		min = *min_max.first,
		max = *min_max.second;

	float total = max - min;

	pcl::PointXYZRGB point_rgb;

	for (int i = 0; i < cloud->size(); i++) {
		point_rgb.x = cloud->points[i].x;
		point_rgb.y = cloud->points[i].y;
		point_rgb.z = cloud->points[i].z;
		point_rgb.r = 255 * (near_points[i] - min) / total;
		point_rgb.g = 255 * (1 - (near_points[i] - min) / total);
		point_rgb.b = 0; //255-uint8_t((near_points[i] - min_near_number) / total_near_number * 255);

		cloud_rgb->push_back(point_rgb);
	}

	standardize_not_organized_cloud_header(*cloud_rgb);
}

bool CloudTools::is_border_point_on_plane(
	int index, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree)
{
	vector<bool> dimension(ceil(360 / m_border_min_angle), false);
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > & data = cloud->points;
	double max_angle = -999;
	if (kdtree.radiusSearch(
		data[index],
		m_border_search_radius,
		pointIdxRadiusSearch,
		pointRadiusSquaredDistance) > 0)
	{
		pcl::PointXYZ init_vector, compared_vector;
		

		for (int k = 1; k < pointIdxRadiusSearch.size(); k++)
		{
			init_vector = pcl::PointXYZ(
				data[pointIdxRadiusSearch[k]].x - data[index].x,
				data[pointIdxRadiusSearch[k]].y - data[index].y,
				data[pointIdxRadiusSearch[k]].z - data[index].z);

			for (size_t i = 1; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (i != k)
				{
					compared_vector = pcl::PointXYZ(
						data[pointIdxRadiusSearch[i]].x - data[index].x,
						data[pointIdxRadiusSearch[i]].y - data[index].y,
						data[pointIdxRadiusSearch[i]].z - data[index].z);
					double angle =
						vector_angle_360(init_vector, compared_vector);

					if (angle > max_angle)
					{
						max_angle = angle;
					}
				}
			}
		}
	}

	if (360 - max_angle > m_border_min_angle)
	{
		return true;
	}


		//	// Set init vector
		//	if (pointIdxRadiusSearch.size() > 0) 
		//	{
		//		init_vector = pcl::PointXYZ(
		//			data[pointIdxRadiusSearch[1]].x - data[index].x,
		//			data[pointIdxRadiusSearch[1]].y - data[index].y,
		//			data[pointIdxRadiusSearch[1]].z - data[index].z);
		//	}
		//	else
		//	{
		//		return true;
		//	}
		//	// Set compared vector
		//	for (size_t i = 2; i < pointIdxRadiusSearch.size(); ++i) 
		//	{
		//		compared_vector = pcl::PointXYZ(
		//			data[pointIdxRadiusSearch[i]].x - data[index].x,
		//			data[pointIdxRadiusSearch[i]].y - data[index].y,
		//			data[pointIdxRadiusSearch[i]].z - data[index].z);

		//		double angle = 
		//			vector_angle_360(init_vector, compared_vector);

		//		// cout << "now compared points:" << init_vector << " " << compared_vector << " angle:" << angle << endl;

		//		dimension[angle / m_border_min_angle] = true;
		//	}
		//}
		//for (int i = 0; i < dimension.size(); i++) 
		//{
		//	if (dimension[i] == false) 
		//	{
		//		return true;
		//	}
		//}
	

	return false;
}

void CloudTools::segment_plane_border(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<int> &border_index,
	float radius,
	float min_angle)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;

	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	normEst.setRadiusSearch(radius);
	normEst.compute(*normals);

	est.setInputCloud(cloud);
	est.setInputNormals(normals);
	est.setAngleThreshold(M_PI / (180 / min_angle));
	est.setSearchMethod(tree);
	est.setRadiusSearch(radius);
	est.compute(boundaries);

	for (size_t i = 0; i < cloud->size(); i++)
	{
		if (boundaries.points[i].boundary_point > 0)
		{
			border_index.push_back(i);
		}
	}
}

//void CloudTools::segment_plane_border(
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
//	pcl::PointCloud<pcl::Normal>::Ptr normal,
//	vector<int>& process_index,
//	vector<int>& border_index,
//	float radius,
//	float min_angle)
//{
//	pcl::PointCloud<pcl::Boundary> boundaries;
//	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//
//	est.setSearchMethod(tree);
//	est.setInputCloud(cloud);
//	est.setIndices(boost::make_shared<vector<int>>(process_index));
//	est.setInputNormals(normal);
//	est.setAngleThreshold(M_PI / (180 / min_angle));
//	est.setRadiusSearch(2.0);
//	est.compute(boundaries);
//
//	for (int i = 0; i < cloud->size(); i++)
//	{
//		uint8_t x = (boundaries.points[i].boundary_point);
//		int a = static_cast<int>(x);
//		if (a == 1)
//		{
//			border_index.push_back(i);
//		}
//	}
//}

void CloudTools::cloud_cluster(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<int> & process_index,
	vector<vector<int>>& total_cluster_index, 
	float radius, 
	int limited_number)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;

	IndicesPtr indices(new vector<int>());
	indices->assign(process_index.begin(), process_index.end());

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   
	ec.setClusterTolerance(radius);                    
	ec.setMinClusterSize(limited_number);                 
	ec.setMaxClusterSize(INT_MAX);               
	ec.setSearchMethod(tree);                   
	ec.setInputCloud(cloud);
	ec.setIndices(indices);
	ec.extract(cluster_indices);

	// total_cluster_index.assign(cluster_indices->begin(), cluster_indices.end());

	for (int i = 0; i < cluster_indices.size(); ++i)
	{
		total_cluster_index.push_back(vector<int>());
		total_cluster_index[i].swap(cluster_indices[i].indices);
	}
}

void CloudTools::cloud_cluster(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<vector<int>>& total_cluster_index,
	float radius, int limited_number)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(radius);
	ec.setMinClusterSize(limited_number);
	ec.setMaxClusterSize(INT_MAX);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	// total_cluster_index.assign(cluster_indices->begin(), cluster_indices.end());

	for (int i = 0; i < cluster_indices.size(); ++i)
	{
		total_cluster_index.push_back(vector<int>());
		total_cluster_index[i].swap(cluster_indices[i].indices);
	}
}

void CloudTools::find_intersection_of_line_with_two_plane(Eigen::Vector4f & plane_func1, Eigen::Vector4f & plane_func2, Line_func & line_func)
{
	float
		A1 = plane_func1[0],
		B1 = plane_func1[1],
		C1 = plane_func1[2],
		D1 = plane_func1[3],

		A2 = plane_func2[0],
		B2 = plane_func2[1],
		C2 = plane_func2[2],
		D2 = plane_func2[3];

	float m, n, l;
	m = B1 * C2 - B2 * C1;
	n = A2 * C1 - A1 * C2;
	l = A1 * B2 - A2 * B1;

	pcl::PointXYZ p;
	p.x = (B1 * D2 - B2 * D1) / (A1*B2 - A2 * B1);
	p.y = (A1 * D2 - A2 * D1) / (A2*B1 - A1 * B2);
	p.z = 0.0;

	// please visit to know the meaning of the var:
	// https://zh.wikipedia.org/wiki/%E7%A9%BA%E9%97%B4%E7%9B%B4%E7%BA%BF%E5%8F%8A%E5%85%B6%E6%96%B9%E7%A8%8B
	line_func.v[0] = p.x;
	line_func.v[1] = p.y;
	line_func.v[2] = p.z;
	line_func.v[3] = m;
	line_func.v[4] = n;
	line_func.v[5] = l;

	//cout
	//	<< "plane_func1 params:" << plane_func1
	//	<< endl
	//	<< "plane_func2 params:" << plane_func2
	//	<< endl;

	// cout << line_func << endl;
}

void CloudTools::find_intersection_of_point_with_two_line(Line_func & line_func1, Line_func & line_func2, pcl::PointXYZ & point)
{
	pcl::ModelCoefficients lina_a, lina_b;
	lina_a.values.assign(line_func1.v, line_func1.v + 6);
	lina_b.values.assign(line_func2.v, line_func2.v + 6);
	Eigen::Vector4f p_vf;
	pcl::lineWithLineIntersection(lina_a, lina_b, p_vf);
	convert_vector4f_to_pointxyz(p_vf, point);
	//cout << "s_point:" << s_point << endl;
}

void CloudTools::find_points_on_given_planes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<Eigen::Vector4f>& plane_func_vec, vector<vector<int>>& points_vec, float dis_limit)
{
	points_vec.resize(plane_func_vec.size());

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ &p = cloud->points[i];

		float dis = 0.0;

		for (size_t j = 0; j < plane_func_vec.size(); ++j)
		{
			dis = distance_point2plane(p, plane_func_vec[j]);

			if (dis < dis_limit)
			{
				points_vec[j].push_back(i);
			}
		}
	}
}

void CloudTools::find_points_on_given_cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int>& process_index, Cylinder_func & cylinder_func, vector<int>& points_vec, float threshold)
{
	points_vec.clear();

	Eigen::Vector4f line_pt, line_dir;

	cylinder_func.get_center_line(line_pt, line_dir);

	float dis = 0.0, r = cylinder_func.v[6];

	for (size_t i = 0; i < process_index.size(); ++i)
	{
		int tar_i = process_index[i];

		pcl::PointXYZ &p = cloud->points[tar_i];

		dis = pcl::sqrPointToLineDistance(p.getVector4fMap(), line_pt, line_dir);

		dis = sqrt(dis);

		if (abs(dis - r) < threshold)
		{
			points_vec.push_back(tar_i);
		}
	}
}

void CloudTools::find_intersection_of_point_with_three_plane(
	const Eigen::Vector4f &plane_a, const Eigen::Vector4f &plane_b, const Eigen::Vector4f &plane_c, pcl::PointXYZ &p)
{
	Eigen::Vector3f intersection_point;
	pcl::threePlanesIntersection(plane_a, plane_b, plane_c, intersection_point);
	p.x = intersection_point[0];
	p.y = intersection_point[1];
	p.z = intersection_point[2];
}

void CloudTools::remove_points_from_plane_func(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<Eigen::Vector4f>& func_vec, vector<int>& rest_index, float dis_limit)
{
	double dis = 0.0;

	for (int i = 0; i < cloud->points.size(); ++i)
	{
		bool is_on_plane = false;

		for (int j = 0; j < func_vec.size(); ++j)
		{
			dis = distance_point2plane(cloud->points[i], func_vec[j]);
			if (dis < dis_limit)
			{
				is_on_plane = true;
				break;
			}
		}
		if (!is_on_plane)
		{
			rest_index.push_back(i);
		}
	}
}

void CloudTools::remove_points_from_plane_func(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<int>& process_index,
	vector<Eigen::Vector4f>& func_vec, vector<int>& rest_index, float dis_limit)
{
	double dis = 0.0;

	for (int i = 0; i < process_index.size(); ++i)
	{
		bool is_on_plane = false;

		pcl::PointXYZ &p = cloud->points[process_index[i]];

		for (int j = 0; j < func_vec.size(); ++j)
		{
			dis = distance_point2plane(p, func_vec[j]);
			if (dis < dis_limit)
			{
				is_on_plane = true;
				break;
			}
		}
		if (!is_on_plane)
		{
			rest_index.push_back(process_index[i]);
		}
	}
}

void CloudTools::remove_points_from_cylinder_func(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	vector<int>& process_index, 
	vector<Cylinder_func>& func_vec, 
	vector<int>& rest_index, 
	float dis_limit)
{
	vector<Eigen::Vector4f> line_pt_vec, line_dir_vec;
	vector<float> r_vec;

	for (size_t i = 0; i < func_vec.size(); ++i)
	{
		Eigen::Vector4f line_pt, lint_dir;
		float r;

		func_vec[i].get_center_line(line_pt, lint_dir);

		r = func_vec[i].v[6];

		line_pt_vec.push_back(line_pt);
		line_dir_vec.push_back(lint_dir);
		r_vec.push_back(r);
	}

	for (size_t i = 0; i < process_index.size(); ++i)
	{
		bool is_on_cylinder = false;
		
		int p_i = process_index[i];
			
		pcl::PointXYZ &p = cloud->points[p_i];

		// 分别计算到各个圆柱面的距离
		for (size_t j = 0; j < func_vec.size(); ++j)
		{
			double dis = 0.0, r = r_vec[j];

			dis = pcl::sqrPointToLineDistance(p.getVector4fMap(), line_pt_vec[j], line_dir_vec[j]);

			dis = sqrt(dis);

			// 若在圆柱面内部，这个r一般大于实际圆柱面的半径
			if (dis - r < 0 || abs(dis - r) < dis_limit)
			{
				is_on_cylinder = true;
				break;
			}
		}

		if (!is_on_cylinder)
		{
			rest_index.push_back(p_i);
		}
	}

	//cout << "去除侧面圆柱点集数量=" << rest_index.size() << endl;

	// 用于测试，用于查看圆柱面去除是否正常
	//pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//index_vector_to_cloud<pcl::PointXYZ>(rest_index, *cloud, *save_cloud);
	//if (!save_cloud->empty())
	//{
	//	pcl::io::savePCDFileBinary("removed_cylinder_cloud.pcd", *save_cloud);
	//}
}


void CloudTools::remove_points_from_cylinder_func(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<Cylinder_func>& func_vec,
	vector<vector<int>> & oncylinder_index,
	vector<int>& rest_index,
	float dis_limit)
{
	vector<Eigen::Vector4f> line_pt_vec, line_dir_vec;
	vector<float> r_vec;

	oncylinder_index.resize(func_vec.size());

	for (size_t i = 0; i < func_vec.size(); ++i)
	{
		Eigen::Vector4f line_pt, lint_dir;
		float r;

		func_vec[i].get_center_line(line_pt, lint_dir);

		r = func_vec[i].v[6];

		line_pt_vec.push_back(line_pt);
		line_dir_vec.push_back(lint_dir);
		r_vec.push_back(r);
	}

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		bool is_on_cylinder = false;

		int p_i = i, cylinder_i = -1;

		pcl::PointXYZ &p = cloud->points[p_i];

		// 分别计算到各个圆柱面的距离
		for (size_t j = 0; j < func_vec.size(); ++j)
		{
			double dis = 0.0, r = r_vec[j];

			dis = pcl::sqrPointToLineDistance(p.getVector4fMap(), line_pt_vec[j], line_dir_vec[j]);

			dis = sqrt(dis);

			// 若在圆柱面内部，这个r一般大于实际圆柱面的半径
			if (dis - r < 0 || abs(dis - r) < dis_limit)
			{
				is_on_cylinder = true;
				cylinder_i = j;
				break;
			}
		}

		if (!is_on_cylinder)
		{
			rest_index.push_back(p_i);
		}
		else
		{
			oncylinder_index[cylinder_i].push_back(p_i);
		}
	}

	//cout << "去除侧面圆柱点集数量=" << rest_index.size() << endl;

	// 用于测试，用于查看圆柱面去除是否正常
	//pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//index_vector_to_cloud<pcl::PointXYZ>(rest_index, *cloud, *save_cloud);
	//if (!save_cloud->empty())
	//{
	//	pcl::io::savePCDFileBinary("removed_cylinder_cloud.pcd", *save_cloud);
	//}
}

void CloudTools::convert_vector4f_to_pointxyz(const Eigen::Vector4f & v, pcl::PointXYZ & p)
{
	p.x = v[0];
	p.y = v[1];
	p.z = v[2];
}

void CloudTools::special_order(vector<int> v, vector<size_t>& pos)
{
	pos.resize(v.size());
	vector<Order_container> oc_vec;
	for (size_t i = 0; i < v.size(); ++i)
	{
		oc_vec.push_back(Order_container{ v[i],i });
	}
	std::sort(oc_vec.begin(), oc_vec.end());

	for (size_t i = 0; i < v.size(); ++i)
	{
		pos[i] = oc_vec[i].i;
	}
}

bool CloudTools::is_parallel(const Line_func& l1, const Line_func& l2, double threshold)
{
	double
		r1 = l1.v[3] / l2.v[3],
		r2 = l1.v[4] / l2.v[4],
		r3 = l1.v[5] / l2.v[5];

#ifdef WP_DEBUG
	cout 
		<< "平行边判定误差:" << endl
		<< abs(r1 - r2) << " " << abs(r2 - r3) << " " << abs(r1 - r3) << endl;
#endif // WP_DEBUG
	
	if (abs(r1 - r2) < threshold && abs(r2 - r3) < threshold && abs(r1 - r3) < threshold)
	{
		return true;
	}
	return false;
}

bool CloudTools::is_parallel(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2, double threshold)
{
	double
		r1 = v1[0] / v2[0],
		r2 = v1[1] / v2[1],
		r3 = v1[2] / v2[2];

	if (v1[0] == 0 || v2[0] == 0)
	{
		r1 = 0;
	}
	if (v1[1] == 0 || v2[1] == 0)
	{
		r2 = 0;
	}
	if (v1[2] == 0 || v2[2] == 0)
	{
		r3 = 0;
	}

	//cout << r1 << " " << r2 << " " << r3 << endl;

	if (r1 == 0 && r2 == 0)
	{
		return r3 > 0;
	}
	else if(r1 == 0 && r3 == 0)
	{
		return r2 > 0;
	}
	else if (r2 == 0 && r3 == 0)
	{
		return r1 > 0;
	}
	else if (r1 == 0)
	{
		if (abs(r2 - r3) < threshold && abs(r3 - r2) < threshold)
		{
			return true;
		}
	}
	else if (r2 == 0)
	{
		if (abs(r1 - r3) < threshold && abs(r3 - r1) < threshold)
		{
			return true;
		}
	}
	else if (r3 == 0)
	{
		if (abs(r2 - r1) < threshold && abs(r1 - r2) < threshold)
		{
			return true;
		}
	}
	else
	{
		if (abs(r1 - r2) < threshold && abs(r2 - r3) < threshold && abs(r1 - r3) < threshold)
		{
			return true;
		}
	}

	return false;
}

bool CloudTools::is_vertical(const Line_func & l1, const Line_func & l2, float threshold)
{
	if (l1.v[4] * l2.v[4] + l1.v[5] * l2.v[5] + l1.v[6] * l2.v[6] < threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CloudTools::is_equal(double a, double b, double th)
{
	if (abs(a - b) < th)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CloudTools::is_on_plane(const pcl::PointXYZ & p, const Eigen::Vector4f & plane_func, double threshold)
{
	if (p.x * plane_func[0] + p.y * plane_func[1] + p.z * plane_func[2] + plane_func[3] < threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CloudTools::find_point_along_with_vector_within_dis(pcl::PointXYZ & p, Eigen::Vector3f & line_dir, pcl::PointXYZ &result_p, float d, bool is_ahead)
{
	if (d == 0)
	{
		result_p = p;
		return;
	}
	double m, n, l;
	m = line_dir[0];
	n = line_dir[1];
	l = line_dir[2];

	// M,N,L = x,y,l
	double
		M = sqrt((d*d*m*m) / (m*m + n * n + l * l)),
		N = sqrt((d*d*n*n) / (m*m + n * n + l * l)),
		L = sqrt((d*d*l*l) / (m*m + n * n + l * l));

	double x[2], y[2], z[2];

	x[0] = p.x + M;
	x[1] = p.x - M;
	y[0] = p.y + N;
	y[1] = p.y - N;
	z[0] = p.z + L;
	z[1] = p.z - L;

	//cout
	//	<< "p" << endl << p
	//	<< "line_dir" << endl << line_dir << endl;
	//cout
	//	<< "x:" << x[0] << " " << x[1] << endl
	//	<< "y:" << y[0] << " " << y[1] << endl
	//	<< "z:" << z[0] << " " << z[1] << endl;

	Eigen::Vector3f v;

	vector<pcl::PointXYZ> tmp_p;

	//! 筛选出两个平行于方向向量的点
	for (size_t i = 0; i < 2; i++)
	{
		for (size_t j = 0; j < 2; j++)
		{
			for (size_t k = 0; k < 2; k++)
			{
				v[0] = x[i] - p.x;
				v[1] = y[j] - p.y;
				v[2] = z[k] - p.z;
				if (is_parallel(line_dir, v))
				{
					pcl::PointXYZ p;
					p.x = x[i];
					p.y = y[j];
					p.z = z[k];
					tmp_p.push_back(p);
				}
			}
		}
	}

	// 一般会有两个结果
	if (M != 0 && N != 0 && L != 0 && tmp_p.size() != 2)
	{
		cerr << "[warning] point_along_with_vector_within_dis the possible point size != 2" << endl;
	}

	//! 根据于方向向量同向原则，选择前进或者后退的点
	for (size_t i = 0; i < tmp_p.size(); ++i)
	{
		pcl::PointXYZ & cp = tmp_p[i];
		v[0] = cp.x - p.x;
		v[1] = cp.y - p.y;
		v[2] = cp.z - p.z;

		//cout << "定点" << p << endl;
		//cout
		//	<< "可能的向量" << endl << tmp_dir << endl
		//	<< "对比向量" << endl << line_dir << endl;
		//cout <<"向量点积为:"<< tmp_dir.dot(line_dir) << endl;

		if(v.dot(line_dir) > 0 && is_ahead)
		{
			//cout << p << "->" << cp << " direction:" << endl << tmp_dir << endl;
			result_p = cp;
		}
		else if (!is_ahead)
		{
			result_p = cp;
		}
	}
}

void CloudTools::middle_point_between_two_point(pcl::PointXYZ & p1, pcl::PointXYZ & p2, pcl::PointXYZ & p)
{
	p.x = (p1.x + p2.x) / 2;
	p.y = (p1.y + p2.y) / 2;
	p.z = (p1.z + p2.z) / 2;
}

void CloudTools::convert_vector4f_to_vector3f(Eigen::Vector4f & v4, Eigen::Vector3f & v3)
{
	v3[0] = v4[0];
	v3[1] = v4[1];
	v3[2] = v4[2];
}

void CloudTools::convert_vector3f_to_pointxyz(Eigen::Vector3f & v, pcl::PointXYZ & p)
{
	p.x = v[0];
	p.y = v[1];
	p.z = v[2];
}

void CloudTools::find_fitting_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ & center_point, float & radius, float radius_step) {
	float
		total_distance = 0.0,
		last_total_distance = FLT_MAX;
	radius = 0.0;

	while (true)
	{
		for (int i = 0; i < cloud->points.size(); ++i)
		{
			total_distance = total_distance + (vector_distance(cloud->points[i], center_point) - radius);
		}
		total_distance = abs(total_distance);

		if (total_distance < last_total_distance)
		{
			last_total_distance = total_distance;
			radius += radius_step;
		}
		else
		{
			radius -= radius_step;
			break;
		}
		total_distance = 0;
	}
}

void CloudTools::find_fitting_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &process_index, pcl::PointXYZ & center_point, float & radius, float radius_step) {
	float
		total_distance = 0.0,
		last_total_distance = FLT_MAX;
	radius = 0.0;

	while (true)
	{
		int tar_i = 0;
		for (int i = 0; i < process_index.size(); ++i)
		{
			tar_i = process_index[i];

			total_distance = total_distance + (vector_distance(cloud->points[tar_i], center_point) - radius);
		}
		total_distance = abs(total_distance);

		if (total_distance < last_total_distance)
		{
			last_total_distance = total_distance;
			radius += radius_step;
		}
		else
		{
			radius -= radius_step;
			break;
		}
		total_distance = 0;
	}
}


void CloudTools::adjust_radius(DETECTED_TYPE type, float& fitting_radius) {
	switch (type)
	{
	case ADJUST_HEXAGON:
		fitting_radius = fitting_radius * (2 / 1.8);
		break;
	case ADJUST_CIRCLE:
		fitting_radius = fitting_radius * (1);
		break;
	case ADJUST_SQUARE:
		fitting_radius = fitting_radius * (sqrt(2) / 1.11421);
		break;
	case ADJUST_EQUILATERAL_TRIANGLE:
		fitting_radius = fitting_radius * (1.5);
		break;
	default:
		break;
	}
}

void CloudTools::fitting_hexagon_cloud_plane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr std_cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr vertex,
	pcl::PointXYZ & center_point,
	float & radius,
	bool create_fitting_cloud,
	float radius_step,
	float x_step,
	int maximum_iterations)
{
	// Find the fittness radius
	find_fitting_radius(cloud, center_point, radius);

	// Adjust the fittness radius, although it's unnecessary to do it.
	adjust_radius(ADJUST_HEXAGON, radius);

	if (!create_fitting_cloud)
	{
		// cout << "No fitting hexagon cloud created" << endl;
		// Do nothing about circle_cloud
		return;
	}

	create_cloud_hexagon_plane(radius, std_cloud, vertex, center_point, x_step);
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	cloud_transform_matrix(std_cloud, cloud, transformation_matrix, maximum_iterations);

	pcl::transformPointCloud(*std_cloud, *std_cloud, transformation_matrix);
	pcl::transformPointCloud(*vertex, *vertex, transformation_matrix);
}

void CloudTools::fitting_cylinder(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	Cylinder_func& cylinder_func,
	float leaf_size, float radius, 
	float cylinder_radius_min, float cylinder_radius_max)
{
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(leaf_size, leaf_size, leaf_size);
	sor.filter(*cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(radius);
	ne.setInputCloud(cloud);
	ne.compute(*normal);

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);        	//设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER); 	//设置分割模型为圆柱型
	seg.setMethodType(pcl::SAC_RANSAC);      	//设置采用RANSAC作为算法的参数估计方法
	seg.setNormalDistanceWeight(0.1);         	//设置表面法线权重系数
	seg.setMaxIterations(10000);              	//设置迭代的最大次数10000
	seg.setDistanceThreshold(0.1);           	//设置内点到模型的距离允许最大值
	seg.setRadiusLimits(cylinder_radius_min, cylinder_radius_max);              	//设置估计出的圆柱模型的半径范围
	seg.setInputCloud(cloud);
	seg.setInputNormals(normal);
	seg.segment(*inliers_cylinder, *coefficients_cylinder);

	for (int i = 0; i < coefficients_cylinder->values.size(); ++i)
	{
		cylinder_func.v[i] = coefficients_cylinder->values[i];
	}
}

void CloudTools::fitting_circle_cloud_plane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud, 
	pcl::PointXYZ & center_point, 
	float & radius, 
	bool create_fitting_cloud, 
	float radius_step, 
	float angle_step, 
	int maximum_iterations)
{
	// Find the fittness radius
	find_fitting_radius(cloud, center_point, radius, radius_step);

	// Adjust the fittness radius, althoug it's unnecessary to do it.
	adjust_radius(ADJUST_CIRCLE, radius);

	// If there is no need to draw point cloud on screen
	if (!create_fitting_cloud) {
		// cout << "No fitting circle cloud created" << endl;
		// Do nothing about circle_cloud
		return;
	}
	// 在指定圆心生成一个圆形点云数据，用于配准
	create_cloud_circle_plane(radius, circle_cloud, center_point, angle_step);

	// 通过旋转得到最终点云
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	cloud_transform_matrix(circle_cloud, cloud, transformation_matrix, maximum_iterations);
	pcl::transformPointCloud(*circle_cloud, *circle_cloud, transformation_matrix);
}

void CloudTools::fitting_circle_cloud_radius(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointXYZ & center_point,
	float & radius,
	float radius_step)
{
	// Find the fittness radius
	find_fitting_radius(cloud, center_point, radius, radius_step);
}

void CloudTools::fitting_circle_cloud_radius(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	std::vector<int> &process_index,
	pcl::PointXYZ & center_point,
	float & radius,
	float radius_step)
{
	// Find the fittness radius
	find_fitting_radius(cloud, process_index, center_point, radius, radius_step);
}

void CloudTools::fitting_cylinder_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<int>& process_index, Cylinder_func & cylinder_func, float threshold, float r_range, float r_step)
{
	float
		r_down = cylinder_func.v[6] - r_range,

		r_up = cylinder_func.v[6] + r_range;

	vector<size_t> on_cylinder_size_vec;

	for (float r = r_down; r < r_up; r += r_step)
	{
		vector<int> points_vec;

		cylinder_func.v[6] = r;

		find_points_on_given_cylinder(cloud, process_index, cylinder_func, points_vec, threshold);

		on_cylinder_size_vec.push_back(points_vec.size());
	}

	auto max_it = std::max_element(on_cylinder_size_vec.begin(), on_cylinder_size_vec.end());

	size_t max_index = std::distance(on_cylinder_size_vec.begin(), max_it);

	cylinder_func.v[6] = r_down + max_index * r_step;
}

double CloudTools::function_hexagon(float x, float r) {
	return (-sqrt(3) *x + sqrt(3) * r);
}

void CloudTools::color_using_normal_com(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
	float
		min_nx = FLT_MAX, min_ny = FLT_MAX, min_nz = FLT_MAX,
		max_nx = FLT_MIN, max_ny = FLT_MIN, max_nz = FLT_MIN,
		curr_nx = 0, curr_ny = 0, curr_nz = 0;

	pcl::PointXYZRGB point;

	for (int i = 0; i < cloud_normal->points.size(); i++)
	{
		curr_nx = cloud_normal->points[i].normal_x;
		curr_ny = cloud_normal->points[i].normal_y;
		curr_nz = cloud_normal->points[i].normal_z;

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

	float
		total_nx = max_nx - min_nx,
		total_ny = max_ny - min_ny,
		total_nz = max_nz - min_nz;

	for (int i = 0; i < cloud_rgb->points.size(); i++)
	{
		pcl::PointXYZRGB & p = cloud_rgb->points[i];
		pcl::PointNormal & cp = cloud_normal->points[i];

		p.r = 255 * (cp.normal_x - min_nx) / total_nx;
		p.g = 255 * (cp.normal_y - min_ny) / total_ny;
		p.b = 255 * (cp.normal_z - min_nz) / total_nz;
	}
}

void CloudTools::create_cloud_hexagon_plane(
	float radius, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr vertex,
	pcl::PointXYZ & center_point, 
	float x_step) {

	// 在[R/2,R]上画六边形的一条边
	// 通过旋转来得到完整的六边形
	float
		begin_x = radius / 2,
		end_x = radius;

	// 画出一条线
	pcl::PointCloud<pcl::PointXYZ>::Ptr point(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr line_on_hexagon_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 添加顶点
	point->points.push_back(pcl::PointXYZ(
		begin_x,
		function_hexagon(begin_x, radius),
		0));
	
	// 添加一条线
	for (float i = begin_x; i < end_x; i += x_step)
	{
		line_on_hexagon_cloud->points.push_back(
			pcl::PointXYZ(
				i,// + center_point.x,
				function_hexagon(i, radius),// + center_point.y,
				0//center_point.z
			)
		);
	}

	// 将这条线旋转6次，每次60度
	// 围绕z轴旋转
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	float theta = M_PI / 3;
	transform(0, 0) = cos(theta);
	transform(0, 1) = -sin(theta);
	transform(1, 0) = sin(theta);
	transform(1, 1) = cos(theta);

	// cout << transform <<endl;

	for (int i = 0; i < 6; ++i) {
		*cloud += *line_on_hexagon_cloud;
		pcl::transformPointCloud(*line_on_hexagon_cloud, *line_on_hexagon_cloud, transform);

		*vertex += *point;
		pcl::transformPointCloud(*point, *point, transform);
	}
	cloud->height = 1;
	cloud->width = cloud->points.size();

	// 调整到指定中心点(平移坐标系)
	for (int i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x += center_point.x;
		cloud->points[i].y += center_point.y;
		cloud->points[i].z += center_point.z;
	}
	// 调整顶点坐标到指定中心点
	for (int i = 0; i < vertex->points.size(); ++i) {
		vertex->points[i].x += center_point.x;
		vertex->points[i].y += center_point.y;
		vertex->points[i].z += center_point.z;
	}

}

void CloudTools::cloud_cluster_recursion(
	int point_index,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
	vector<int> & cluster_index,
	vector<bool> & visited) {

	vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > & data = cloud->points;
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	if (kdtree.radiusSearchT(data[point_index],
		m_cloud_cluster_search_radius, 
		pointIdxRadiusSearch, 
		pointRadiusSquaredDistance) > 0) 
	{
		for (size_t i = 1; i < pointIdxRadiusSearch.size(); ++i) 
		{
			if (visited[pointIdxRadiusSearch[i]] == false) 
			{
				visited[pointIdxRadiusSearch[i]] = true;
				cluster_index.push_back(pointIdxRadiusSearch[i]);
				cloud_cluster_recursion(pointIdxRadiusSearch[i], cloud, kdtree, cluster_index, visited);
			}
		}
	}
}

void CloudTools::cloud_transform_matrix(
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
	Eigen::Matrix4f & transformation_matrix, 
	int maximumIterations)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(source_cloud);
	icp.setInputTarget(target_cloud);
	// Set the max correspondence distance to 2cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(5);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations(maximumIterations);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon(1e-10);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon(0.01);
	pcl::PointCloud<pcl::PointXYZ> final;
	icp.align(final);
	transformation_matrix = icp.getFinalTransformation();
}

void CloudTools::create_cloud_circle_plane(
	float radius,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointXYZ & center_point,
	float angle_step)
{
	float x, y;
	for (float angle = 0; angle < 360; angle += angle_step)
	{
		x = radius * cos(M_PI / (180 / angle)) + center_point.x;
		y = radius * sin(M_PI / (180 / angle)) + center_point.y;
		cloud->points.push_back(pcl::PointXYZ(x, y, center_point.z));
	}
	cloud->height = 1;
	cloud->width = cloud->points.size();
}

double CloudTools::vector_distance(pcl::PointXYZ & p1, pcl::PointXYZ & p2)
{
	return sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y) + (p2.z - p1.z)*(p2.z - p1.z));
}

void CloudTools::plane_segment_recursion(
	int original_index, int compare_index,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::Normal>::Ptr normal,
	pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
	vector<int>& single_plane_index,
	vector<bool>& visited)
{

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// cout << original_index <<"<->"<< compare_index <<endl;
	// cout << single_plane_index.size()<<endl;
	if (kdtree.radiusSearchT(cloud->points[compare_index], m_plane_segment_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 1; i < pointIdxRadiusSearch.size(); ++i)
		{
			if (visited[pointIdxRadiusSearch[i]] == false)
			{
				compare_index = pointIdxRadiusSearch[i];
				if (is_on_the_same_plane(normal->points[original_index], normal->points[compare_index]))
				{
					single_plane_index.push_back(compare_index);
					// Marked as visited one.
					visited[compare_index] = true;
					plane_segment_recursion(original_index, compare_index, cloud, normal, kdtree, single_plane_index, visited);
					
					if (cloud->points.size() == ++m_segment_plane_count)
					{
						return;
					}
				}
				else
				{
					// Not on the same plane.
				}
			}
			else
			{
				// Already visited.
			}
		}
	}
	else
	{
		// No search result
	}
}

bool CloudTools::is_on_the_same_plane(const pcl::Normal & p1, const pcl::Normal& p2) {
	//if (p1.normal_x == 0.0)  p1.normal_x = 0.000001;
	//if (p1.normal_y == 0.0)  p1.normal_y = 0.000001;
	//if (p1.normal_z == 0.0)  p1.normal_z = 0.000001;
	//if (p2.normal_x == 0.0)  p2.normal_x = 0.000001;
	//if (p2.normal_y == 0.0)  p2.normal_y = 0.000001;
	//if (p2.normal_z == 0.0)  p2.normal_z = 0.000001;

	pcl::PointXYZ p1_xyz = pcl::PointXYZ(p1.normal_x, p1.normal_y, p1.normal_z);

	pcl::PointXYZ p2_xyz = pcl::PointXYZ(p2.normal_x, p2.normal_y, p2.normal_z);

	double angle = vector_angle_180(p1_xyz, p2_xyz);

	return angle < m_plane_segment_angle_threshold ? true : false;
}

bool CloudTools::is_on_the_same_plane(const pcl::Normal & p1, const pcl::Normal& p2, float angle_threshold) {
	//if (p1.normal_x == 0.0)  p1.normal_x = 0.000001;
	//if (p1.normal_y == 0.0)  p1.normal_y = 0.000001;
	//if (p1.normal_z == 0.0)  p1.normal_z = 0.000001;
	//if (p2.normal_x == 0.0)  p2.normal_x = 0.000001;
	//if (p2.normal_y == 0.0)  p2.normal_y = 0.000001;
	//if (p2.normal_z == 0.0)  p2.normal_z = 0.000001;

	pcl::PointXYZ p1_xyz = pcl::PointXYZ(p1.normal_x, p1.normal_y, p1.normal_z);

	pcl::PointXYZ p2_xyz = pcl::PointXYZ(p2.normal_x, p2.normal_y, p2.normal_z);

	double angle = vector_angle_180(p1_xyz, p2_xyz);

	return angle < angle_threshold ? true : false;
}

void CloudTools::list_order_by_vector_index(list<vector<int>>& total_plane_index, vector<size_t>& order_index)
{
	list<vector<int>> new_total_plane_index;

	for (size_t i = 0; i < order_index.size(); ++i)
	{
		size_t list_i = 0;
		list<vector<int>>::iterator	it;
		for (it = total_plane_index.begin(); it != total_plane_index.end(); ++it)
		{
			if (order_index[i] == list_i)
			{
				new_total_plane_index.push_back(*it);
				break;
			}
			++list_i;
		}
	}
	total_plane_index = new_total_plane_index;
}

void CloudTools::convert_plane_func(vector<float>& vec, Eigen::Vector4f & plane_func)
{
	double 
		x0, y0, z0,
		A, B, C;

	x0 = vec[0];
	y0 = vec[1];
	z0 = vec[2];

	A = vec[3];
	B = vec[4];
	C = vec[5];

	// ABCD
	plane_func[0] = A;
	plane_func[1] = B;
	plane_func[2] = C;
	plane_func[3] = 0 - A * x0 - B * y0 - C * z0;
}

void CloudTools::find_point_vetical_to_plane(pcl::PointXYZ & p, Eigen::Vector4f & plane_func, pcl::PointXYZ &p_result)
{
	float A, B, C, D;
	A = plane_func[0];
	B = plane_func[1];
	C = plane_func[2];
	D = plane_func[3];

	float x0, y0, z0;
	x0 = p.x;
	y0 = p.y;
	z0 = p.z;
	
	float t = (A * x0 + B * y0 + C * z0 + D) / (A * A + B * B + C * C);

	p_result.x = x0 - A * t;
	p_result.y = y0 - B * t;
	p_result.z = z0 - C * t;
}

void CloudTools::find_semi_circle_on_border(
	pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, 
	float radius, 
	std::vector<pcl::PointXYZ> & circle_center_point_vec,
	float deviation_threshold)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	
	float semi_dis = sqrt(2) * radius;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	pcl::PointCloud<pcl::PointXYZ>::Ptr circle_center_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXYZ co_axis(1, 0, 0);

	float tmp_dis = 0.0;

	//ofstream ofile("average_distance.txt", std::ios::out);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ & p = cloud->points[i];

		if (kdtree.radiusSearch(p, semi_dis, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0)
		{
			continue;
		}

		pcl::PointXYZ p1, middle_point, v1, circle_center_point;

		Eigen::Vector3f vertical;

		p1 = cloud->points[pointIdxRadiusSearch.back()];

		middle_point_between_two_point(p, p1, middle_point);
		
		v1.x = p.x - p1.x;
		v1.y = p.y - p1.y;
		v1.z = p.z - p1.z;

		vertical = v1.getVector3fMap().cross(co_axis.getVector3fMap());

		find_point_along_with_vector_within_dis(middle_point, vertical, circle_center_point, sqrt(radius*radius - (radius * radius) / 2));

		deviation_distance_point_to_points(circle_center_point, cloud, pointIdxRadiusSearch, tmp_dis);

		if (tmp_dis < deviation_threshold)
		{
			circle_center_cloud->points.push_back(circle_center_point);
		}

		//ofile << tmp_dis << " ";
	}
	//ofile << endl;
	//ofile.close();

	if (circle_center_cloud->empty())
	{
		return;
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr test(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::savePCDFileBinary("semi_circle.pcd", *circle_center_cloud);

	std::vector<std::vector<int>> total_cluster;
	cloud_cluster(circle_center_cloud, total_cluster, 1.0, 0);

	cout << "there are " << total_cluster.size() << " semi-circle." << endl;

	for (auto &i: total_cluster)
	{
		pcl::PointXYZ circle_center;
		find_center_point3d(*circle_center_cloud, circle_center, i);
		circle_center_point_vec.push_back(circle_center);
	}

	//circle_center_cloud->height = 1;
	//circle_center_cloud->width = test_cloud->points.size();
	//pcl::io::savePCDFileBinary("test.pcd", *circle_center_cloud);
	/*
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(test_cloud);
	while (!viewer.wasStopped()) {}
	*/

}

void CloudTools::deviation_distance_point_to_points(pcl::PointXYZ & p, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &process_index, float& average_distance)
{
	float total_dis = 0.0, dis = 0.0;
	std::vector<float> dis_vec;

	for (auto i: process_index)
	{
		dis = pcl::geometry::distance(p, cloud->points[i]);
		dis_vec.push_back(dis);
		total_dis += dis;
	}
	average_distance = total_dis / process_index.size();

	total_dis = 0.0;
	for (auto i : dis_vec)
	{
		total_dis = total_dis + (i - average_distance) * (i - average_distance);
	}
	average_distance = sqrt(total_dis / process_index.size());
}

void CloudTools::max_box_deltaXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &delta_x, float &delta_y, float &delta_z)
{
	Eigen::Vector4f minpt, maxpt;

	pcl::getMinMax3D(*cloud, minpt, maxpt);

	delta_x = maxpt[0] - minpt[0];
	delta_y = maxpt[1] - minpt[1];
	delta_z = maxpt[2] - minpt[2];
}

void CloudTools::projected_plane_size(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float & width, float &height)
{
	// projected to YZ plane, so x is 0
	Eigen::Vector4f min_pt, max_pt;

	pcl::getMinMax3D(*cloud, min_pt, max_pt);

	std::vector<pcl::PointXYZ> corner_point(4);

	corner_point[0] = pcl::PointXYZ(0, min_pt[1], min_pt[2]);
	corner_point[1] = pcl::PointXYZ(0, min_pt[1], max_pt[2]);
	corner_point[2] = pcl::PointXYZ(0, max_pt[1], min_pt[2]);
	corner_point[3] = pcl::PointXYZ(0, max_pt[1], max_pt[2]);

	float d0, d1, d2;
	d0 = max_pt[0] - min_pt[0];
	d1 = max_pt[1] - min_pt[1];
	d2 = max_pt[2] - min_pt[2];

	if (d0 == 0)
	{
		width = d1;
		height = d2;
	}
	else if (d1 == 0)
	{
		width = d0;
		height = d2;
	}
	else
	{
		width = d0;
		height = d1;
	}


	//std::vector<pcl::PointXYZ> ordered_corner_point;
	//ordered_corner_point.push_back(corner_point[0]);
	//corner_point.erase(corner_point.begin());

	//size_t j = 0;
	//while (!corner_point.empty())
	//{
	//	std::vector<pcl::PointXYZ>::iterator it = corner_point.end();
	//	float distance = FLT_MAX;

	//	for (std::vector<pcl::PointXYZ>::iterator i = corner_point.begin(); i != corner_point.end(); i++)
	//	{
	//		float d = 0.0;
	//		d = pcl::geometry::distance(ordered_corner_point.back(), *i);
	//		if (d < distance)
	//		{
	//			distance = d;
	//			it = i;
	//		}
	//	}
	//	if (it != corner_point.end())
	//	{
	//		ordered_corner_point.push_back(*it);
	//		corner_point.erase(it);
	//	}
	//}

	//// all done, we got ordered corner point set.
	//cout
	//	<< "L1=" << pcl::geometry::distance(ordered_corner_point[0], ordered_corner_point[1]) 
	//	<< " L2=" << pcl::geometry::distance(ordered_corner_point[1], ordered_corner_point[2])
	//	<< " L3=" << pcl::geometry::distance(ordered_corner_point[2], ordered_corner_point[3])
	//	<< " L4=" << pcl::geometry::distance(ordered_corner_point[3], ordered_corner_point[0])
	//	<< endl;

	//vector<Line_func>line_func_vec(4);

	//for (size_t i = 0; i < ordered_corner_point.size(); ++i)
	//{
	//	pcl::PointXYZ &p = ordered_corner_point[i];
	//	line_func_vec[i].v[0] = p.x;
	//	line_func_vec[i].v[1] = p.y;
	//	line_func_vec[i].v[2] = p.z;

	//	pcl::PointXYZ &p2 = ordered_corner_point[(i + 1) % ordered_corner_point.size()];
	//	line_func_vec[i].v[3] = p2.x - p.x;
	//	line_func_vec[i].v[4] = p2.y - p.y;
	//	line_func_vec[i].v[5] = p2.z - p.z;
	//}

	//// line_func[0] <=> line_func[2]
	//// line_func[1] <=> line_func[3]
	//vector<vector<int>> index_vec;
	//collect_points_on_lines(cloud, line_func_vec, index_vec,3.0);
	//
	//// test for line's point cloud
	////for (size_t i = 0; i < index_vec.size(); ++i)
	////{
	////	pcl::PointCloud<pcl::PointXYZ>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	////	index_vector_to_cloud(index_vec[i], *cloud, *save_cloud);
	////	pcl::io::savePCDFileBinary("line_cloud" + std::to_string(i) + ".pcd", *save_cloud);
	////}

	//float dis_0_2 = 0.0, dis_1_3 = 0.0;

	//distance_between_two_lines(cloud, index_vec[0], index_vec[2], 0.1, dis_0_2);

	//distance_between_two_lines(cloud, index_vec[1], index_vec[3], 0.1, dis_1_3);
	//cout
	//	<< "dis_0_2=" << dis_0_2
	//	<< "dis_1_3=" << dis_1_3
	//	<< endl;

}

void CloudTools::collect_points_on_lines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<Line_func> & line_func_vec, vector<vector<int>> & index_vec, float threshold)
{
	index_vec.resize(line_func_vec.size());

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ &p = cloud->points[i];

		size_t line_j = -1;

		for (size_t j = 0; j < line_func_vec.size(); ++j)
		{
			Eigen::Vector4f line_pt, line_dir;
			line_func_vec[j].convert_to_vector4f(line_pt, line_dir);
			float d = pcl::sqrPointToLineDistance(p.getVector4fMap(), line_pt, line_dir);

			d = sqrt(d);
			if (d < threshold)
			{
				line_j = j;

				index_vec[line_j].push_back(i);
			}
		}
	}
}


double CloudTools::distance_point2plane(pcl::PointXYZ &p, Eigen::Vector4f & plane_func)
{
	// without judgement
	return abs(plane_func[0] * p.x + plane_func[1] * p.y + plane_func[2] * p.z + plane_func[3]) /
		sqrt(pow(plane_func[0], 2) + pow(plane_func[1], 2) + pow(plane_func[2], 2));
}

double CloudTools::distance_point2plane_signed(pcl::PointXYZ &p, Eigen::Vector4f & plane_func)
{
	// without judgement
	return (plane_func[0] * p.x + plane_func[1] * p.y + plane_func[2] * p.z + plane_func[3]) /
		sqrt(pow(plane_func[0], 2) + pow(plane_func[1], 2) + pow(plane_func[2], 2));
}

double CloudTools::distance_between_two_plane(Eigen::Vector4f & plane_func1, Eigen::Vector4f & plane_func2)
{
	double
		A = plane_func1[0],
		B = plane_func1[1],
		C = plane_func1[2],

		D1 = plane_func1[3],
		D2= plane_func2[3];

	return abs(D1 - D2) / sqrt(A*A + B * B + C * C);
}

void CloudTools::distance_between_two_plane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<int>& plane_points_index_1,
	vector<int>& plane_points_index_2,
	float ratio, float & distance)
{
	if (ratio < 0 || ratio > 1)
	{
		cerr << "[error] ratio must be 0 to 1" << endl;
		return;
	}

	vector<float>dis_vec;

	float dis = 0.0;

	for (size_t i = 0; i < plane_points_index_1.size(); ++i)
	{
		int tar_i = plane_points_index_1[i];

		pcl::PointXYZ &p_i = cloud->points[tar_i];

		float dis = 0.0, min_dis = FLT_MAX;

		for (size_t j = 0; j < plane_points_index_2.size(); ++j)
		{
			int tar_j = plane_points_index_2[j];

			pcl::PointXYZ &p_j = cloud->points[tar_j];

			dis = pcl::geometry::distance(p_i, p_j);

			if (dis < min_dis)
			{
				min_dis = dis;
			}
		}
		dis_vec.push_back(min_dis);
	}

	std::sort(dis_vec.begin(), dis_vec.end());

	size_t b = ratio * dis_vec.size();

	float total_dis = 0.0;

	for (size_t i = 0; i < b; ++i)
	{
		total_dis += dis_vec[i];
	}
	distance = total_dis / b;
}

void CloudTools::distance_between_two_lines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<int>& plane_points_index_1,
	vector<int>& plane_points_index_2,
	float ratio, float &distance)
{
	if (ratio < 0 || ratio > 1)
	{
		cerr << "[error] ratio must be 0 to 1" << endl;
		return;
	}

	Line_func line_1;
	find_line_function(cloud, plane_points_index_1, line_1, 0.1);
	Eigen::Vector3f dire_ptr;
	line_1.get_direction(dire_ptr);

	vector<float>dis_vec;

	float dis = 0.0;

	for (size_t i = 0; i < plane_points_index_1.size(); ++i)
	{
		int tar_i = plane_points_index_1[i];

		pcl::PointXYZ &p_i = cloud->points[tar_i];

		float dis = 0.0, min_dis = FLT_MAX;

		size_t min_dis_j = 0;
		for (size_t j = 0; j < plane_points_index_2.size(); ++j)
		{
			int tar_j = plane_points_index_2[j];

			pcl::PointXYZ &p_j = cloud->points[tar_j];

			dis = pcl::geometry::distance(p_i, p_j);

			if (dis < min_dis)
			{
				min_dis_j = tar_j;

				min_dis = dis;
			}
		}

		pcl::PointXYZ p1;
		p1.x = cloud->points[min_dis_j].x - p_i.x;
		p1.y = cloud->points[min_dis_j].y - p_i.y;
		p1.z = cloud->points[min_dis_j].z - p_i.z;

		//if ()
		//{

		//}

		dis_vec.push_back(min_dis);
	}

	std::sort(dis_vec.begin(), dis_vec.end());

	size_t b = ratio * dis_vec.size();

	float total_dis = 0.0;

	for (size_t i = 0; i < b; ++i)
	{
		total_dis += dis_vec[i];
	}
	distance = total_dis / b;
}

void CloudTools::find_plane_function(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f & plane_func, float distance_threshold)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	plane_func = Eigen::Vector4f(
		coefficients->values[0],
		coefficients->values[1],
		coefficients->values[2],
		coefficients->values[3]
	);
}

void CloudTools::find_plane_function(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int>& process_index, Eigen::Vector4f & plane_func, float distance_threshold)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	coefficients->values.resize(4);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setInputCloud(cloud);
	seg.setIndices(boost::make_shared<vector<int>>(process_index));

	seg.segment(*inliers, *coefficients);

	plane_func = Eigen::Vector4f(
		coefficients->values[0],
		coefficients->values[1],
		coefficients->values[2],
		coefficients->values[3]
	);
}


void CloudTools::find_circle3d_function(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &process_index, vector<float> & circle3d_func, float min_radius, float max_radius, float distance_threshold)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);
	seg.setMethodType(pcl::SAC_LMEDS);
	seg.setDistanceThreshold(distance_threshold);
	seg.setMaxIterations(10000);
	seg.setRadiusLimits(min_radius, max_radius);
	seg.setInputCloud(cloud);
	seg.setIndices(boost::make_shared<vector<int>>(process_index));
	seg.segment(*inliers, *coefficients);

	circle3d_func.resize(7);
	circle3d_func.swap(coefficients->values);
}

void CloudTools::find_line_function(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int> &process_index, Line_func & lin_func, float distance_threshold)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	//seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_LMEDS);
	seg.setDistanceThreshold(distance_threshold);
	seg.setMaxIterations(1000);
	seg.setInputCloud(cloud);
	seg.setIndices(boost::make_shared<vector<int>>(process_index));
	seg.segment(*inliers, *coefficients);

	lin_func.v[0] = coefficients->values[0];
	lin_func.v[1] = coefficients->values[1];
	lin_func.v[2] = coefficients->values[2];
	lin_func.v[3] = coefficients->values[3];
	lin_func.v[4] = coefficients->values[4];
	lin_func.v[5] = coefficients->values[5];
}

void CloudTools::find_points_on_plane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	vector<int>& process_index,
	Eigen::Vector4f & plane_func,
	vector<int> & index,
	float distance_threshold)
{
	for (int i = 0; i < process_index.size(); ++i)
	{
		if (distance_point2plane(cloud->points[process_index[i]], plane_func) < distance_threshold)
		{
			index.push_back(process_index[i]);
		}
	}
}

void CloudTools::find_points_on_plane(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	Eigen::Vector4f & plane_func,
	vector<int> & index,
	float distance_threshold)
{
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (distance_point2plane(cloud->points[i], plane_func) < distance_threshold)
		{
			index.push_back(i);
		}
	}
}

void Cylinder_func::get_center_line(Eigen::Vector4f & line_pt, Eigen::Vector4f & line_dir)
{
	line_pt[0] = v[0];
	line_pt[1] = v[1];
	line_pt[2] = v[2];
	line_pt[3] = 0;

	line_dir[0] = v[3];
	line_dir[1] = v[4];
	line_dir[2] = v[5];
	line_dir[3] = 0;
}

void Cylinder_func::set_value(float line_pt_x, float line_pt_y, float line_pt_z, float normal_x, float normal_y, float normal_z, float r)
{
	v[0] = line_pt_x;
	v[1] = line_pt_y;
	v[2] = line_pt_z;
	v[3] = normal_x;
	v[4] = normal_y;
	v[5] = normal_z;
	v[6] = r;
}
