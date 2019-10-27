#pragma once
#include "Header.h"

enum DETECTED_TYPE {
	ADJUST_HEXAGON,
	ADJUST_CIRCLE,
	ADJUST_SQUARE,
	ADJUST_EQUILATERAL_TRIANGLE,
};

class CloudTools
{
public:
	CloudTools();

public:
	// 存储 索引下的点云
	template<typename INPUT_T, typename OUTPUT_T>
	void index_vector_to_cloud(
		vector<int>& indexVec,
		pcl::PointCloud<INPUT_T> & input_cloud,
		pcl::PointCloud<OUTPUT_T>& output_cloud) {
		output_cloud.height = 1;
		output_cloud.width = indexVec.size();
		for (size_t i = 0; i != indexVec.size(); i++) {
			output_cloud.points.push_back(
				OUTPUT_T(input_cloud.points[indexVec[i]].x,
					input_cloud.points[indexVec[i]].y,
					input_cloud.points[indexVec[i]].z));
		}
	}

	// 计算两个向量的夹角[0,360]
	double vector_angle_360(pcl::PointXYZ& p1, pcl::PointXYZ& p2);

	// 计算两个向量的夹角[0,180]
	double vector_angle_180(pcl::PointXYZ& p1, pcl::PointXYZ& p2);

	// 弧度与角度之间的转换
	double radian2angle(double radian);
	double angle2radian(double angle);


	// 统计滤波
	void filter_statistical_outlier_removal(
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud,
		float mean_k, float std_thresh);

	// 体素滤波
	void filter_voxel_grid_downsample(
		pcl::PCLPointCloud2::Ptr  input_cloud,
		pcl::PCLPointCloud2::Ptr  output_cloud,
		float leaf_size);

	// 判断点云是否包含法向量
	bool is_contained_normal(pcl::PCLPointCloud2::Ptr cloud);


	// 计算点云的法向
	void create_cloud_with_normal(
		pcl::PCLPointCloud2::Ptr cloud,
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals,
		float searchRadius = 0.8);

	// 平面分割
	void plane_segment(
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
		vector<vector<int>> &total_plane_group,
		float search_radius,
		float angle_threshold,
		int group_number_threshold);

	template<typename T>
	void save_cloud_with_random_color(const string filename, pcl::PointCloud<T> &cloud_without_rgb, std::default_random_engine &generator) {
		std::uniform_int_distribution<int> random(0, 255);
		uint8_t r, g, b;
		r = random(generator);
		g = random(generator);
		b = random(generator);
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointXYZRGB point_rgb;
		for (int i = 0; i < cloud_without_rgb.points.size(); i++) {
			point_rgb.x = cloud_without_rgb.points[i].x;
			point_rgb.y = cloud_without_rgb.points[i].y;
			point_rgb.z = cloud_without_rgb.points[i].z;
			point_rgb.rgb = *reinterpret_cast<float*>(&rgb);
			cloud_with_rgb->push_back(point_rgb);
		}

		standardize_not_organized_cloud_header<pcl::PointXYZRGB>(*cloud_with_rgb);
		pcl::io::savePCDFile(filename, *cloud_with_rgb);
	}

	// 根据法向量显示颜色
	void color_using_normal(pcl::PCLPointCloud2::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb);
	
	// 根据点云分辨率给点云上色
	// 红色->绿色: 点云密度高->低
	void color_using_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb, float radius = 1.0);

	// 分割边界
	void border_segment(vector<int>& border_index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_angle);

	// 点云聚类(用于提取出独立边界)
	void cloud_cluster(vector<vector<int>> & total_cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int limited_number);

	/*
	* 拟合圆形，得到半径以及圆心
	* 可以选择性获得圆形点云
	* @center_point: 点云重心点，需要提前算好
	* @radius: 拟合出的最佳半径(对于圆形可直接使用，但是其它则需要调整)
	* @create_fitting_cloud: 用于判断是否需要用于绘制的标准点云
	* @radius_step: 拟合半径时的递增步长
	* @angle_step: 生成用于绘制的标准点云的角度步长(note:不是所有标准图形称此名)
	* @maximum_iterations: 用于icp的迭代次数, 次数大小决定最终标准图形拟合效果
	*/	
	void fitting_circle_cloud_plane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud,
		pcl::PointXYZ & center_point,
		float & radius,
		bool create_fitting_cloud = true,
		float radius_step = 0.05,	// for finding fitting_radius
		float angle_step = 1.0,		// for drawing std circle cloud
		int maximum_iterations = 50 // for geting fittness matrix
	);
	

	/*
	* 拟合六边形，得到边长
	* 可以选择性获得六边形点云
	* @center_point: 点云重心点，需要提前算好
	* @radius: 拟合出的最佳半径(对于圆形可直接使用，但是其它则需要调整)
	* @create_fitting_cloud: 用于判断是否需要用于绘制的标准点云
	* @radius_step: 拟合半径时的递增步长
	* @x_step: 生成用于绘制的标准点云的x方向步长(note:不是所有标准图形称此名)
	* @maximum_iterations: 用于icp的迭代次数, 次数大小决定最终标准图形拟合效果
	*/
	void fitting_hexagon_cloud_plane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr std_cloud,
		pcl::PointXYZ & center_point,
		float & radius,
		bool create_fitting_cloud = true,
		float radius_step = 0.05,	// for finding fitting_radius
		float x_step = 0.01,		// for drawing std hexagon cloud
		int maximum_iterations = 50 // for geting fittness matrix
	);

	// 找点云的重心坐标
	void find_center_point3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointXYZ & center_point);
private:

	// common
	float m_default_search_radius;
	void find_fitting_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ & center_point,float & radius,float radius_step = 0.01);
	void adjust_radius(DETECTED_TYPE type, float &fitting_radius);
	double vector_distance(pcl::PointXYZ & p1, pcl::PointXYZ &p2);
	void cloud_transform_matrix(
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
		Eigen::Matrix4f &transformation_matrix,
		int maximumIterations = 50);
	template <typename T>
	void standardize_not_organized_cloud_header(pcl::PointCloud<T> & cloud) {
		cloud.width = cloud.points.size();
		cloud.height = 1;
	}
	// end for common

	// plane_segment
	void plane_segment_recursion(
		int original_index, int compare_index,
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
		pcl::KdTreeFLANN<pcl::PointNormal> &kdtree, 
		vector<int>& single_group_index,
		vector<bool>& visited);
	bool is_on_the_same_plane(pcl::PointNormal & p1, pcl::PointNormal& p2);
	float m_plane_segment_search_radius;
	float m_plane_segment_angle_threshold;
	// end for plane_segment

	// border_segment
	bool is_border_point_on_plane(int index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);
	float m_border_search_radius;
	float m_border_min_angle;
	// end for segment


	// cluster
	void cloud_cluster_recursion(int point_index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, vector<int> & cluster, vector<bool> & visited);
	float m_cloud_cluster_search_radius;
	// end for cluster


	// color_using resolution
	float m_color_using_resolution_search_radius;
	// end for color_using_resolution

	// fitting_circle_cloud_plane
	void create_cloud_circle_plane(
		float radius, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		pcl::PointXYZ & center_point,
		float angle_step = 1);
	// end for fitting_circle_cloud_plane

	//fitting hexagon cloud
	void create_cloud_hexagon_plane(
		float radius, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		pcl::PointXYZ & center_point,
		float x_step = 0.01);
	double function_hexagon(float x, float r);
	//end for fitting hexagon cloud
};