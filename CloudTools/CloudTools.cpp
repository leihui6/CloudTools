#pragma once
#include "CloudTools.h"

CloudTools::CloudTools() {

	m_default_search_radius = 0.8;
	m_border_search_radius = m_default_search_radius;
	m_plane_segment_search_radius = m_default_search_radius;
	// 边界聚类的搜索半径
	m_cloud_cluster_search_radius = m_default_search_radius;
	m_color_using_resolution_search_radius = m_default_search_radius;

}

double CloudTools::vector_angle_360(pcl::PointXYZ& p1, pcl::PointXYZ& p2) {
	Eigen::Vector3d v1 = Eigen::Vector3d(p1.x, p1.y, p1.z);
	Eigen::Vector3d v2 = Eigen::Vector3d(p2.x, p2.y, p2.z);
	double radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
	if (v1.cross(v2).z() < 0) {
		radian_angle = 2 * M_PI - radian_angle;
	}
	return radian_angle * 180 / M_PI;
}


// 计算两个向量的夹角[0,180]
double CloudTools::vector_angle_180(pcl::PointXYZ& p1, pcl::PointXYZ& p2) {
	double cos_angle_value = (p1.x * p2.x + p1.y * p2.y + p1.z * p2.z) /
		(sqrt(p1.x * p1.x + p1.y * p1.y + p1.z * p1.z) * sqrt(p2.x * p2.x + p2.y * p2.y + p2.z * p2.z));
	if (cos_angle_value > 0)
		return radian2angle(acos(cos_angle_value));
	else
		return 180 - radian2angle(acos(abs(cos_angle_value)));
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

void CloudTools::plane_segment(
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
	vector<vector<int>> &total_plane_index,
	float search_radius,
	float angle_threshold,
	int group_number_threshold
	)
{
	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud_normal);
	vector<int> single_plane_index;
	vector<bool> visited(cloud_normal->points.size(), false);
	m_plane_segment_search_radius = search_radius;
	m_plane_segment_angle_threshold = angle_threshold;

	for (int i = 0; i < visited.size(); ++i) {
		if (visited[i] == false) {
			single_plane_index.push_back(i);
			visited[i] = true;

			plane_segment_recursion(i, i, cloud_normal, kdtree, single_plane_index, visited);

			if (single_plane_index.size() > group_number_threshold) {
				total_plane_index.push_back(single_plane_index);
			}
			single_plane_index.clear();
		}
	}
}

void CloudTools::color_using_normal(pcl::PCLPointCloud2::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>());
	if (!is_contained_normal(cloud)) 
	{
		create_cloud_with_normal(cloud, cloud_normal);
	}
	else 
	{
		pcl::fromPCLPointCloud2(*cloud, *cloud_normal);
	}
	//vector<float>normal_x_vec, normal_y_vec, normal_z_vec;
	float min_nx = 999, min_ny = 999, min_nz = 999;
	float max_nx = -999, max_ny = -999, max_nz = -999;
	float curr_nx = 0, curr_ny = 0, curr_nz = 0;

	pcl::PointXYZRGB point;

	for (int i = 0; i < cloud_normal->points.size(); i++) 
	{
		curr_nx = cloud_normal->points[i].normal_x;
		curr_ny = cloud_normal->points[i].normal_y;
		curr_nz = cloud_normal->points[i].normal_z;

		//normal_x_vec.push_back(curr_nx);
		//normal_y_vec.push_back(curr_ny);
		//normal_z_vec.push_back(curr_nz);

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
	//cout << "max_nx/ny/nz" << max_nx << " " << max_ny << " " << max_nz << endl;
	//cout << "min_nx/ny/nz" << min_nx << " " << min_ny << " " << min_nz << endl;

	float
		total_nx = max_nx - min_nx,
		total_ny = max_ny - min_ny,
		total_nz = max_nz - min_nz;

	uint8_t r, g, b;
	uint32_t rgb;

	for (int i = 0; i < cloud_rgb->points.size(); i++) 
	{
		r = (cloud_normal->points[i].normal_x - min_nx) / total_nx * 255;
		g = (cloud_normal->points[i].normal_y - min_ny) / total_ny * 255;
		b = (cloud_normal->points[i].normal_z - min_nz) / total_nz * 255;
		rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		cloud_rgb->points[i].rgb = *reinterpret_cast<float*>(&rgb);
	}
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
	float min_near_number = 999, max_near_number = -999;
	float total_near_number;
	for (int i = 0; i < near_points.size(); i++) {
		if (near_points[i] > max_near_number)
			max_near_number = near_points[i];
		if (near_points[i] < min_near_number)
			min_near_number = near_points[i];
	}
	total_near_number = max_near_number - min_near_number;
	pcl::PointXYZRGB point_rgb;
	uint8_t r, g, b;
	for (int i = 0; i < cloud->size(); i++) {
		point_rgb.x = cloud->points[i].x;
		point_rgb.y = cloud->points[i].y;
		point_rgb.z = cloud->points[i].z;
		r = uint8_t((near_points[i] - min_near_number) / total_near_number * 255);
		g = 255 - uint8_t((near_points[i] - min_near_number) / total_near_number * 255);
		b = 0;//255-uint8_t((near_points[i] - min_near_number) / total_near_number * 255);

		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		point_rgb.rgb = *reinterpret_cast<float*>(&rgb);

		cloud_rgb->push_back(point_rgb);
	}

	standardize_not_organized_cloud_header(*cloud_rgb);
}

bool CloudTools::is_border_point_on_plane(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree)
{
	vector<bool> dimension(int(360 / m_border_min_angle), false);
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > & data = cloud->points;

	if (kdtree.radiusSearch(
		data[index],
		m_border_search_radius, 
		pointIdxRadiusSearch, 
		pointRadiusSquaredDistance) > 0)
	{
		pcl::PointXYZ init_vector, compared_vector;

		// Set init vector
		if (pointIdxRadiusSearch.size() > 0) 
		{
			init_vector = pcl::PointXYZ(
				data[pointIdxRadiusSearch[1]].x - data[index].x,
				data[pointIdxRadiusSearch[1]].y - data[index].y,
				data[pointIdxRadiusSearch[1]].z - data[index].z);
		}
		else
		{
			return true;
		}
		// Set compared vector
		for (size_t i = 2; i < pointIdxRadiusSearch.size(); ++i) 
		{
			compared_vector = pcl::PointXYZ(
				data[pointIdxRadiusSearch[i]].x - data[index].x,
				data[pointIdxRadiusSearch[i]].y - data[index].y,
				data[pointIdxRadiusSearch[i]].z - data[index].z);

			double angle = 
				vector_angle_360(init_vector, compared_vector);

			// cout << "now compared points:" << init_vector << " " << compared_vector << " angle:" << angle << endl;

			dimension[angle / m_border_min_angle] = true;
		}
	}
	for (int i = 0; i < dimension.size(); i++) 
	{
		if (dimension[i] == false) 
		{
			return true;
		}
	}
	return false;
}

void CloudTools::border_segment(vector<int> &border_index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_angle) {
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	m_border_min_angle = min_angle;
	for (int i = 0; i < cloud->points.size(); ++i) 
	{
		if (is_border_point_on_plane(i, cloud, kdtree))
		{
			border_index.push_back(i);
		}
	}
}

void CloudTools::cloud_cluster(vector<vector<int>>& total_cluster_index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int limited_number)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	vector<bool> visited;
	visited.resize(cloud->size(), false);
	vector<int> single_cluster_index;

	for (int i = 0; i < cloud->size(); i++) {
		if (visited[i] == false) {
			visited[i] = true;
			cloud_cluster_recursion(i, cloud, kdtree, single_cluster_index, visited);
			//cout << "single_cluster_index:" << single_cluster_index.size() << endl;
			if (single_cluster_index.size() > limited_number)
			{
				total_cluster_index.push_back(single_cluster_index);
			}
			single_cluster_index.clear();
		}
	}
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

	create_cloud_hexagon_plane(radius, std_cloud, center_point, x_step);
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	cloud_transform_matrix(std_cloud, cloud, transformation_matrix, maximum_iterations);
	pcl::transformPointCloud(*std_cloud, *std_cloud, transformation_matrix);
}

void CloudTools::find_center_point3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ & center_point) {
	Eigen::Vector4f centroid_eigen;
	pcl::compute3DCentroid(*cloud, centroid_eigen);
	center_point.x = centroid_eigen[0];
	center_point.y = centroid_eigen[1];
	center_point.z = centroid_eigen[2];
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

double CloudTools::function_hexagon(float x, float r) {
	return (-sqrt(3) *x + sqrt(3) * r);
}

void CloudTools::create_cloud_hexagon_plane(
	float radius, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	pcl::PointXYZ & center_point, 
	float x_step) {

	// 在[R/2,R]上画六边形的一条边
	// 通过旋转来得到完整的六边形
	float
		begin_x = radius / 2,
		end_x = radius;

	// 画出一条线
	pcl::PointXYZ point;
	pcl::PointCloud<pcl::PointXYZ>::Ptr line_on_hexagon_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (float i = begin_x; i < end_x; i += x_step)
	{
		point = pcl::PointXYZ(
			i,// + center_point.x,
			function_hexagon(i, radius),// + center_point.y,
			0//center_point.z
		);
		line_on_hexagon_cloud->points.push_back(point);
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
	}
	cloud->height = 1;
	cloud->width = cloud->points.size();

	// 调整到指定中心点(平移坐标系)
	for (int i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x += center_point.x;
		cloud->points[i].y += center_point.y;
		cloud->points[i].z += center_point.z;
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

	if (kdtree.radiusSearch(data[point_index],
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
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
	pcl::KdTreeFLANN<pcl::PointNormal> &kdtree,
	vector<int>& single_plane_index,
	vector<bool>& visited) {

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// cout << original_index <<"<->"<< compare_index <<endl;
	// cout << single_plane_index.size()<<endl;
	if (kdtree.radiusSearch(cloud->points[compare_index], m_plane_segment_search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 1; i < pointIdxRadiusSearch.size(); ++i)
		{
			if (visited[pointIdxRadiusSearch[i]] == false)
			{
				compare_index = pointIdxRadiusSearch[i];
				if (is_on_the_same_plane(cloud->points[original_index], cloud->points[compare_index]))
				{
					single_plane_index.push_back(compare_index);
					// Marked as visited one.
					visited[compare_index] = true;
					plane_segment_recursion(original_index, compare_index, cloud, kdtree, single_plane_index, visited);
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

bool CloudTools::is_on_the_same_plane(pcl::PointNormal & p1, pcl::PointNormal& p2) {
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

