#pragma once
#ifndef CLOUDTOOLS_H

#include "Header.h"

enum DETECTED_TYPE {
	ADJUST_HEXAGON,
	ADJUST_CIRCLE,
	ADJUST_SQUARE,
	ADJUST_EQUILATERAL_TRIANGLE,
};

//! 使用点向式表示直线
struct Line_func 
{
	//! 使用两点式表示直线方程
	/*!
	v[0] 直线上一点.x
	v[1] 直线上一点.y
	v[2] 直线上一点.z
	v[3] 方向向量.m
	v[4] 方向向量.n
	v[5] 方向向量.l
	*/
	float v[6];
	friend ostream & operator << (ostream & os,const Line_func & lf)
	{
		cout
			<< "(x0,y0,z0)->" << "(" << lf.v[0] << "," << lf.v[1] << "," << lf.v[2] << ")"
			<< endl
			<< "(n,m,l)->" << "(" << lf.v[3] << "," << lf.v[4] << "," << lf.v[5] << ")";
		return os;
	}

	void convert_to_vectorxf(Eigen::VectorXf& vx);

	void convert_to_vector4f(Eigen::Vector4f &line_pt, Eigen::Vector4f &line_dir);

	void get_direction(Eigen::Vector3f &line_dir);
};

struct Cylinder_func {
	//! 柱面方程
	/*!
	v[0]-v[5] 为柱面中轴直线的参数
	v[6] 为柱面的半径

	v[0] 直线上一点.x
	v[1] 直线上一点.y
	v[2] 直线上一点.z
	v[3] 方向向量.m
	v[4] 方向向量.n
	v[5] 方向向量.l
	v[6] 半径

	*/
	float v[7];

	friend ostream & operator << (ostream & os, const Cylinder_func & cf)
	{
		cout
			<< "(x0,y0,z0)->" << "(" << cf.v[0] << "," << cf.v[1] << "," << cf.v[2] << ")"
			<< endl
			<< "(n,m,l)->" << "(" << cf.v[3] << "," << cf.v[4] << "," << cf.v[5] << ")"
			<< endl
			<< "r->" << cf.v[6];
		return os;
	}
};

struct Order_container {

	//! 排序依据
	int v;
	//! 原来的索引，不参与排序
	size_t i;

	// 升序(v<o.v则为降序)
	bool operator < (const Order_container &o) const
	{
		return v > o.v;
	}
};

struct Couple_distance {
	// 索引
	size_t i1;
	size_t i2;
	// 索引对应的距离
	double distance;
};

class CloudTools
{
public:
	CloudTools();

public:
	//!存储索引的点云
	
	/*!
	\param[in] indexVec 在input_cloud下的索引
	\param[in] input_cloud 点云
	\param[out] output_cloud 索引所指定的点云
	*/
	template<typename INPUT_T,typename OUTPUT_T>
	void index_vector_to_cloud(vector<int>& indexVec, pcl::PointCloud<INPUT_T> & input_cloud, pcl::PointCloud<OUTPUT_T>& output_cloud)
	{
		output_cloud.clear();

		for (size_t i = 0; i != indexVec.size(); i++)
		{
			output_cloud.points.push_back(
				OUTPUT_T(
					input_cloud.points[indexVec[i]].x,
					input_cloud.points[indexVec[i]].y,
					input_cloud.points[indexVec[i]].z));
		}
		standardize_not_organized_cloud_header<OUTPUT_T>(output_cloud);
	}

	//! 计算两个向量的夹角[0,360]
	double vector_angle_360(pcl::PointXYZ& p1, pcl::PointXYZ& p2);

	

	//! 统计滤波
	void filter_statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float mean_k, float std_thresh);

	//! 体素滤波
	void filter_voxel_grid_downsample(pcl::PCLPointCloud2::Ptr  input_cloud, pcl::PCLPointCloud2::Ptr  output_cloud, float leaf_size);
	void filter_voxel_grid_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, float leaf_size);
	
	//! 判断点云是否包含法向量
	bool is_contained_normal(pcl::PCLPointCloud2::Ptr cloud);

	//! 计算点云的法向
	void create_cloud_with_normal(pcl::PCLPointCloud2::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, float searchRadius = 0.8);
	void create_cloud_with_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal, float search_radius);
	
	//! 根据法向分布聚类，平面分割
	/*!
	\param[in] cloud_normal 包含法线的点云数据
	\param total_plane_group 包含索引的聚类结果
	\param ordered_group_index 按照聚类结果点集数量大小排序，保存的是聚类结果的索引, 默认降序排序
	\param rest_point_index 剩余的点集索引
	\param search_radius 聚类时搜索点集的半径
	\param angle_threshold 判断是否同一类的角度限制
	\param group_number_threshold 对聚类的数量限制
	\param dis_same_plane 判断同一片点云的距离
	\param plane_size 指定需要的平面数量(选择方法为从大到小), 0代表不选择(即选择全部的分割结果)
	*/
	void segment_plane(
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
		list<vector<int>> &total_plane_group,
		vector<int> & rest_point_index,
		float search_radius,
		float angle_threshold,
		int group_number_threshold,
		float dis_same_plane = 2.0,
		size_t plane_size = 7);

	
	//! 将点云赋予指定颜色并保存
	/*!
	\param[in] filename 保存的文件名称
	\param[in] cloud_without_rgb: 需要保存的点云，一般是没有颜色的
	\param[in] r,g,b: [0-255]
	
	*/
	template<typename T>
	void save_cloud_with_color(const string filename, pcl::PointCloud<T> &cloud_without_rgb,
		unsigned int r, unsigned int g, unsigned int b)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointXYZRGB point_rgb;
		for (int i = 0; i < cloud_without_rgb.points.size(); i++) {
			point_rgb.x = cloud_without_rgb.points[i].x;
			point_rgb.y = cloud_without_rgb.points[i].y;
			point_rgb.z = cloud_without_rgb.points[i].z;
			point_rgb.r = r;
			point_rgb.g = g;
			point_rgb.b = b;
			cloud_with_rgb->push_back(point_rgb);
		}

		standardize_not_organized_cloud_header<pcl::PointXYZRGB>(*cloud_with_rgb);
		pcl::io::savePCDFile(filename, *cloud_with_rgb);
	}

	//! 根据法向量显示颜色,用于区分平面效果不错
	/*!
	根据法向量显示颜色,用于区分平面效果不错
	PCLPointCloud2格式，可以强制计算法向量，也可以使用自带的法向量信息

	\param[in] cloud 需要计算的点云
	\param[out] 输出xyzrgb格式，可以直接用于显示
	\param[in] 是否强制计算法向量，由于PCLPointCloud2格式读取可能自带法向量信息，所以这里可以选择是否强制再次计算法向量

	*/
	void color_using_normal(
		pcl::PCLPointCloud2::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
		bool force_to_normal,
		float radius = 0.8);

	/*!
	* 根据法向量显示颜色
	* @cloud:
	*	需要计算法向量的点云，目前强制为xyz点云
	* @cloud_rgb:
	*	输出xyzrgb格式，可以直接用于显示
	* @radius:
	*	计算法向量时使用
	*/
	void color_using_normal(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
		float radius = 0.8);

	/*
	* 根据点云分辨率给点云上色
	* 显示的颜色特征：
	*	红色->绿色: 点云密度高->低
	*/
	void color_using_resolution(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb, 
		float radius = 1.0);

	/* 分割边界
	* @border_index: 边界分割的结果，以索引形式保存
	* @cloud: 需要分割的点云
	* @radius: 法向计算半径，同时也是判断是否为边界的半径
	* @min_angle: 角度判断阈值
	*/
	void segment_plane_border(
		vector<int>& border_index, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		float radius, 
		float min_angle = 60);

	/* 
	* 根据kdtree距离的点云聚类(可用于提取出独立边界)
	* @ total_cluster: 聚类结果
	* @ cloud: 所需聚类点云
	* @ radius: kdtree搜索半径
	* @ limited_number: 判定为一有效聚类阈值
	*/
	void cloud_cluster( 
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		vector<int> & process_index,
		vector<vector<int>> & total_cluster, float radius, int limited_number);

	/*
	* 拟合圆形，得到半径以及圆心
	* 可以选择性获得圆形点云
	* @center_point: 点云重心点，输入
	* @radius: 拟合出的最佳半径(对于圆形可直接使用，但是其它则需要调整) ，输出
	* @create_fitting_cloud: 用于判断是否需要用于绘制的标准点云 ，输入
	* @radius_step: 拟合半径时的递增步长， 
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
	* @cloud: 用户输入
	* @std_cloud: 标准六边形，输出
	* @vertex_vec: 标准六边形的顶点数据，输出
	* @center_point: 点云重心点，输入
	* @radius: 拟合出的最佳半径(对于圆形可直接使用，但是其它则需要调整) ，输出
	* @create_fitting_cloud: 用于判断是否需要用于绘制的标准点云
	* @radius_step: 拟合半径时的递增步长
	* @x_step: 生成用于绘制的标准点云的x方向步长(note:不是所有标准图形称此名)
	* @maximum_iterations: 用于icp的迭代次数, 次数大小决定最终标准图形拟合效果
	*/
	void fitting_hexagon_cloud_plane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr std_cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr vertex_vec,
		pcl::PointXYZ & center_point,
		float & radius,
		bool create_fitting_cloud = true,
		float radius_step = 0.05,	// for finding fitting_radius
		float x_step = 0.01,		// for drawing std hexagon cloud
		int maximum_iterations = 50 // for geting fittness matrix
	);

	//! 拟合圆柱
	/*!
		输入点云，拟合其中的圆柱方程

	\param[in] cloud 需要判断的点云
	\param[in] coefficients_cylinder 圆柱方程
	\param[in] leaf_size 网格降采样的大小
	\param[in] radius 计算法向的kdtree搜索半径
	\param[in] cylinder_radius_min 拟合的圆柱最小半径
	\param[in] cylinder_radius_max 拟合的圆柱最大半径

	*/
	void fitting_cylinder(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		Cylinder_func& cylinder_func,
		float leaf_size = 0.2,
		float radius = 0.8,
		float cylinder_radius_min = 5.0,
		float cylinder_radius_max = 10.0
	);

	/*
	* 找点云的重心坐标
	* @input 可以计算点云索引部分
	* note:
	* @index 可选
	*/
	template<typename T=pcl::PointXYZ>
	void find_center_point3d(pcl::PointCloud<T> cloud, pcl::PointXYZ & center_point)
	{
		Eigen::Vector4f centroid_eigen;
		pcl::compute3DCentroid<T>(cloud, centroid_eigen);
		center_point.x = centroid_eigen[0];
		center_point.y = centroid_eigen[1];
		center_point.z = centroid_eigen[2];
	}

	template<typename T>
	void find_center_point3d(pcl::PointCloud<T> cloud, pcl::PointXYZ & center_point, vector<int>& index)
	{
		Eigen::Vector4f centroid_eigen;
		pcl::compute3DCentroid(cloud, index, centroid_eigen);
		center_point.x = centroid_eigen[0];
		center_point.y = centroid_eigen[1];
		center_point.z = centroid_eigen[2];
	}

	//! 计算平面方程
	/*!
	使用pcl方法计算平面方程
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	
	\param[in] cloud 需要拟合的点云
	\param[out] plane_func 拟合的平面方程
	\param[in] distance_threshold 控制拟合阈值
	*/
	void find_plane_function(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f & plane_func, float distance_threshold = 0.001);
	
	//! 查找在某个平面上某一高度范围内的点集
	/*!
	
	\param[in] cloud 平面所在的点云
	\param[in] plane_func 平面方程
	\param[out] index 在平面上的点的索引
	\param[in] distance_threshold 在这个范围内的点将被视为在平面上

	*/
	void find_points_on_plane(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		vector<int>& process_index,
		Eigen::Vector4f & plane_func,
		vector<int> & index,
		float distance_threshold
	);

	//! 点到平面距离，并取绝对值
	double distance_point2plane(pcl::PointXYZ &p, Eigen::Vector4f & plane_func);

	//! 点到平面距离，有符号
	double distance_point2plane_signed(pcl::PointXYZ &p, Eigen::Vector4f & plane_func);

	//! 平面到平面距离
	double distance_between_two_plane(Eigen::Vector4f & plane_func1, Eigen::Vector4f & plane_func2);

	template <typename T>
	void standardize_not_organized_cloud_header(pcl::PointCloud<T> & cloud) {
		cloud.width = cloud.points.size();
		cloud.height = 1;
	}

	//! 计算空间直线方程
	/*!
	计算空间直线方程，需要可以输入两各平面方程
	\param[in] plane_func1 平面方程1
	\param[in] plane_func2 平面方程2
	\param[out] line_func 两平面方程定义下的直线方程

	*/
	void find_intersection_of_line_with_two_plane(Eigen::Vector4f& plane_func1, Eigen::Vector4f& plane_func2, Line_func& line_func);

	//! 计算三个平面的交点
	/*!
	\param[in] plane_a 平面方程a
	\param[in] plane_b 平面方程b
	\param[in] plane_c 平面方程c
	\param[out] intersection_point 交点
	*/
	void find_intersection_of_point_with_three_plane(const Eigen::Vector4f &plane_a, const Eigen::Vector4f &plane_b, const Eigen::Vector4f &plane_c, pcl::PointXYZ &p);


	//! 计算两直线交点
	/*!
	输入两直线方程，计算交点
	对于空间的直线，可能不会存在真正意义上的交点(大多数)，需要设置某一个阈值

	\param[in] line_func1 直线方程1
	\param[in] line_func2 直线方程2
	\param[out] point 两直线交点

	*/
	void find_intersection_of_point_with_two_line(Line_func& line_func1, Line_func& line_func2,pcl::PointXYZ &point);

	
	//! 去除点云中某平面上的点集
	/*!
	指定几个平面方程，删除在指定平面方程上的点集

	\param[in] cloud 点云
	\param[in] process_index 可指定需要处理的点云索引
	\param[in] func_vec 指定要删除点云所在的平面方程
	\param[out] removed_index 删除平面点集后剩下的点云索引，即这些点集都不在所指定平面上
	\param[in] dis_limit
	*/
	void remove_points_from_plane_func(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		vector<int> & process_index,
		vector<Eigen::Vector4f>& func_vec, 
		vector<int> & rest_index,
		float dis_limit
	);

	//! 将以Eigen::Vector4f表示的点转换为 pointxyz格式
	void convert_vector4f_to_pointxyz(const Eigen::Vector4f &v, pcl::PointXYZ &p);

	//! 排序并获取索引
	void special_order(vector<int> v, vector<size_t>& pos);

	//! 判断两直线是否平行
	bool is_parallel(const Line_func& l1, const Line_func& l2, double threshold = 9e-2);
	
	//! 判断两方向向量是否平行
	bool is_parallel(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, double threshold = 9e-2);
	
	//! 判断两直线是否垂直
	bool is_vertical(const Line_func& l1, const Line_func& l2, double threshold = 9e-3);

	//! 判断两浮点数是否相等
	bool is_equal(double a, double b, double th = 9e-3);

	//! 判断一个点是否在平面上
	bool is_on_plane(const pcl::PointXYZ &p, const Eigen::Vector4f & plane_func, double threshold = 9e-2);

	//! 计算P0沿着向量移动D距离后的新坐标
	/*!
	\param[in] p 方向向量上的定点
	\prarm[in] p 所在直线的方向向量
	\prarm[out] result_p 移动后的坐标点
	\param[in] d 在方向向量上移动的距离
	\param[in] is_ahead true选择沿着dir前进的点; false则反之
	*/
	void find_point_along_with_vector_within_dis(pcl::PointXYZ &p, Eigen::Vector3f & line_dir, pcl::PointXYZ &result_p, double d, bool is_ahead = true);

	//! 计算给定两点的中间点坐标
	void middle_point_between_two_point(pcl::PointXYZ &p1, pcl::PointXYZ &p2, pcl::PointXYZ &p);

	//! 将Vector4f转换到Vector3f,忽略掉最后一个值
	void convert_vector4f_to_vector3f(Eigen::Vector4f& v4, Eigen::Vector3f& v3);

	//! 将vector3f转换到PointXYZ类型
	void convert_vector3f_to_pointxyz(Eigen::Vector3f & v, pcl::PointXYZ &p);

	//! 计算两个向量的夹角[0,180]
	double vector_angle_180(pcl::PointXYZ& p1, pcl::PointXYZ& p2);

	//! 弧度与角度之间的转换
	double radian2angle(double radian);

	//! 角度与弧度之间的转换
	double angle2radian(double angle);

	//! 判断两点是否在平面上
	bool is_on_the_same_plane(const pcl::PointNormal & p1, const pcl::PointNormal& p2);

private:
	float m_plane_segment_angle_threshold;

	float m_plane_segment_search_distance;

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
	
	// end for common

	// plane_segment
	void plane_segment_recursion(
		int original_index, int compare_index,
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
		pcl::KdTreeFLANN<pcl::PointNormal> &kdtree, 
		vector<int>& single_group_index,
		vector<bool>& visited);
	
	float m_plane_segment_search_radius;
	
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr vertex,
		pcl::PointXYZ & center_point,
		float x_step = 0.01);
	double function_hexagon(float x, float r);
	//end for fitting hexagon cloud


	// colour using normals
	void color_using_normal_com(
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb);
};

#endif // CLOUDTOOLS_H
