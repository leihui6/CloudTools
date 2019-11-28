#pragma once
#pragma warning(disable:4996)

#include "../../CloudTools/CloudTools.h"

typedef pcl::PointXYZ pointT;

//! 缺陷区域的信息
struct Defect_info {
	//! 投影到底面平面的面积
	double area_size;

	double 
		max_height, //!< 距离底面平面的最大高度
		mean_height;//!< 距离底面平面的平均高度

	//! 可以用于绘制立方体(高度为0的平面)
	/*!
	vex_vec的四个值可以构成一个平面矩形\n
	以下是用于绘制在空间中的矩形四个定点\n
	vex_vec[0]\n
	vex_vec[1]\n
	vex_vec[2]\n
	vex_vec[3]\n
	*/
	vector<pointT> vex_vec;
};

//! 工件类，负责分析工件数据
/*!
工业零件 目前即刹车片\n
一个工业零件主要检测7个面，\n
包括一个上底面以及六个侧面，其中六个侧面的按顺序存储
*/
class WorkPiece
{
public:
	WorkPiece();

	~WorkPiece();

	//! 设置顶面的方程
	/*!
	设置需要输入含有六个点的参数\n
	此函数的参数可来自3D数据采集设备\n
	六个点分别是:\n
	[0][1][2]:平面所过定点\n
	[3][4][5]:平面法向
	*/
	void set_top_plane_func(vector<float> &top_plane_func);

	//! 加载原始点云数据
	/*!
	输入原始点云，开始计算相关信息 \n
	注意原始点云会被网格化处理

	\param[in] cloud 原始点云数据

	*/
	void load_original_cloud(pcl::PointCloud<pointT>::Ptr cloud);

	//! 计算工件平面方程
	/*!
	计算平面方程，会计算整体工件的平面，并进行排序，归类等功能
	*/
	void self_calc_plane_func();

	//! 将原始点云去除圆柱面
	/*!
	\param[in] threshold_dis 用于判断是否在柱面方程上
	*/
	void self_remove_cylinder_cloud(double threshold_dis = 1.0);

	//! 检测底面缺陷部分
	void detect_defect_part_on_bottom_plane();

	//! 检测底面距离最长边
	void detect_max_dis_bottom_line();

	//! 获取底面最长边(以两点表示直线段)，以及对应的距离
	/*!
	边的形式以线段，而线段以一对点的形式给出，所以一组最长边，用四个点表示 \n
	point_vec中的元素每四个一组  \n
	point_vec.[0]-[1] 是一条边  \n
	point_vec.[2]-[3] 是另一条边 \n

	\param[out] point_vec 点集对，每四个一组
	\param[out] dis_vec 距离对，每四个point_vec的元素对应一个
	*/
	void get_max_dis_bottom_line_segment(vector<pointT>& point_vec, vector<double> &dis_vec);

	//! 计算底面距离边某一段距离的内点
	/*!
	\param[in] dis 需要移动的距离
	*/
	void detect_bottom_inner_point_along_with_line(double dis);

	//! 获取底面距离边某一段距离的内点
	/*!
	\param[out] inner_points 移动之后的点集
	\param[out] dis_vec 需要移动的距离集合，与点一一对应
	*/
	void get_bottom_inner_point_along_with_line(vector<pointT>& inner_points, vector<double> &dis_vec);

	//! 获取底面缺陷检测结果
	/*!
	\param[out] bottom_defect 缺陷点集，索引的是原始点云
	\param[out] defect_info_vec 缺点部分的信息
	*/
	void get_bottom_defect(vector<vector<int>>& bottom_defect, vector<Defect_info>&defect_info_vec);

	//! 获取底面到顶面的高度
	void get_distance_between_top_and_bottom(double & dis);

	//! 获取底面的顶点
	void get_bottom_vertex(vector<pointT>& bottom_vertex) const;

	//! 获取工件的底面点云数据
	void get_bottom_cloud(pcl::PointCloud<pointT>::Ptr bottom_cloud);

	//! 获取原始点云数据
	void get_original_cloud(pcl::PointCloud<pointT>::Ptr original_cloud);

	//! 获取没有圆柱面的原始点云数据
	/*!
	注意这里获取的是索引
	*/
	void get_no_cylinder_original_cloud(vector<int>& no_cylinder_original_cloud_index);

	//! 获取工件的底面平面方程
	void get_bottom_plane_func(Eigen::Vector4f& bottom_plane_func);

	//! 获取侧面点云集合
	void get_beside_cloud_vec(vector<pcl::PointCloud<pointT>::Ptr>& beside_cloud_vec);

	//! 获取侧面平面方程集合
	void get_beside_plane_func_vec(vector<Eigen::Vector4f>& beside_plane_func);

	//! 获取点云中心坐标
	void get_cloud_center_point(pointT & p);
private:

	//! 加载包括侧面以及底面的全部平面点云数据以及对应方程
	/*!
	点集数量最大的为底面，其余的为侧面\n
	其中侧面会依据相邻最近距离进行排序\n
	只有在确定侧面以及顶面点云顺序后才会计算平面方程。\n
	因此，平面方程的顺序和点集的顺序是一样的

	\param[in] 全部点云数据集合

	*/
	void load_all_plane(vector<pcl::PointCloud<pointT>::Ptr> &cloud_vec);

	//! 加载侧面平面点云
	/*!
	输入侧面的点云\n
	会依据距离排序，即相邻的平面一定是相互最近的\n
	排序依据是侧面点云的重心坐标

	\param[in] 侧面点云数据集合

	*/
	void load_beside_plane(vector<pcl::PointCloud<pointT>::Ptr> &cloud_vec);

	//! 加载底面平面点云
	/*!
	输入底面的点云

	\param[in] 底面点云数据
	\param[in] 底面平面方程

	*/
	void load_bottom_plane(pcl::PointCloud<pointT>::Ptr bottom_cloud);

	//! 计算底面和侧面相交的直线方程
	/*!
	计算每个侧面方程和底面的交线
	*/
	void self_calc_bottom_intersection_of_line_func();

	//! 计算底面交线的交点
	/*!
	根据三个平面可以确定一个点的原则\n
	利用相邻两个侧面和底面计算出三平面的交点\n
	同时，这个交点也是底面的顶点
	*/
	void self_calc_bottom_intersection_of_point();

	//! 计算圆柱方程
	/*!
	为了圆柱面拟合准确度，会将其他的平面的数据全部去除，理想情况为仅剩下圆柱面的点云\n
	所以计算前需要计算各个平面的方程，利用方程将其他点集去除，\n
	拟合剩余点云

	\param[in] threshold_dis 用于判断点集是否在侧面方程上

	*/
	void self_calc_cylinder_func(double threshold_dis = 0.8);

	//! 清楚上次所有计算数据
	void clear();

private:
	//! 数据处理工具
	CloudTools m_cloud_tools;

	//! 原始点云数据
	pcl::PointCloud<pointT>::Ptr m_original_cloud;

	//! 底面点云的中心坐标
	pointT m_bottom_center_point;
	
	//! 点云重心坐标
	pointT m_cloud_center_point;

	//! 在原始点云数据的基础上去除了圆柱面
	vector<int> m_original_cloud_without_cylinder_index;

	//! 底面点云数据
	pcl::PointCloud<pointT>::Ptr m_bottom_cloud;

	//! 底面方程
	Eigen::Vector4f m_bottom_plane_func;

	//! 顶面点云数据
	//pcl::PointCloud<pointT>::Ptr m_top_cloud;

	//! 顶面方程\n
	/*!
	[0] [1] [2] 表示定点\n
	[3] [4] [5] 平面法线法向
	*/
	vector<float> m_top_plane_func;

	//!使用ABCD来表示平main方程
	Eigen::Vector4f m_top_plane_func_ABCD;

	//! 侧面点云集合
	vector<pcl::PointCloud<pointT>::Ptr> m_beside_cloud_vec;
	//! 底面方程集合
	vector<Eigen::Vector4f> m_beside_plane_func_vec;

	//! 底面和各个侧面的交线
	vector<Line_func> m_bottom_intersection_of_line_vec;

	//! 底面交线的交点(底面的顶点)
	/*!
	[0]-[1] 是line[0]的两端点\n
	[1]-[2] 是line[1]的两端点
	*/
	vector<pointT> m_bottom_intersection_of_point_vec;

	//! (中间与侧面等高)圆柱的方程
	/*!
	为了内存管理，使用vector存储
	*/
	vector<Cylinder_func> m_cylinder_func_vec;

	//! 底面缺料独立部分，索引相对于去除柱面后的原始点云
	vector<vector<int>> m_bottom_defect_index;
	
	//! 底部缺料信息，以矩形表示
	vector<Defect_info> m_bottom_defect_info_vec;

	//! 底面距离最长边
	/*! 
	索引于 底面和各个侧面的交线\n
	Couple_distance.i1 索引1\n
	Couple_distance.i2 索引2\n
	Couple_distance.distance 两直线间的距离
	*/
	vector<Couple_distance> m_max_dis_bottom_line_vec;

	//! 向内移动一定距离的点
	vector<pointT> m_bottom_innner_points;

	//! 向内移动一定距离的点到顶面的距离
	vector<double> m_bottom_inner_distance;
};

