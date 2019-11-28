#include "WorkPiece.h"

WorkPiece::WorkPiece()
{

}

WorkPiece::~WorkPiece()
{

}

void WorkPiece::set_top_plane_func(vector<float>& top_plane_func)
{
	m_top_plane_func = top_plane_func;

	// ABC
	m_top_plane_func_ABCD[0] = top_plane_func[3];
	m_top_plane_func_ABCD[1] = top_plane_func[4];
	m_top_plane_func_ABCD[2] = top_plane_func[5];

	double x0, y0, z0;
	double A, B, C;

	x0 = top_plane_func[0];
	y0 = top_plane_func[1];
	z0 = top_plane_func[2];

	A = top_plane_func[3];
	B = top_plane_func[4];
	C = top_plane_func[5];

	// D
	m_top_plane_func_ABCD[3] = 0 - A * x0 - B * y0 - C * z0;
}

void WorkPiece::load_original_cloud(pcl::PointCloud<pointT>::Ptr cloud)
{
	m_original_cloud = cloud;

	// 计算各个平面的方程
	self_calc_plane_func();
	// 计算圆柱面方程
	self_calc_cylinder_func();
	// 去除圆柱面
	self_remove_cylinder_cloud();
	// 计算各平面交线
	self_calc_bottom_intersection_of_line_func();
	// 计算底面的顶点
	self_calc_bottom_intersection_of_point();
}

void WorkPiece::load_all_plane(vector<pcl::PointCloud<pointT>::Ptr>& cloud_vec)
{
	if (cloud_vec.empty())
	{
		cerr << "[error] no cloud_vec data." << endl;
		return;
	}

	size_t max_point_size = 0;
	int max_index = 0;
	for (int i = 0; i < cloud_vec.size(); ++i)
	{
		if (cloud_vec[i]->points.size() > max_point_size)
		{
			max_point_size = cloud_vec[i]->points.size();
			max_index = i;
		}
	}

	//! 无序侧面点云
	vector<pcl::PointCloud<pointT>::Ptr> bottom_cloud_vec;

	vector<Eigen::Vector4f> bottom_plane_func_vec;

	for (int i = 0; i < cloud_vec.size(); ++i)
	{
		if (i == max_index)
		{
			load_bottom_plane(cloud_vec[i]);

			m_cloud_tools.find_center_point3d(*cloud_vec[i], m_bottom_center_point);
		}
		else
		{
			bottom_cloud_vec.push_back(cloud_vec[i]);
		}
	}
	load_beside_plane(bottom_cloud_vec);

	//test
	//pcl::io::savePCDFileBinary("bottom_cloud.pcd", *m_bottom_cloud);
	//for (size_t i = 0; i < m_beside_cloud_vec.size(); ++i)
	//{
	//	pcl::io::savePCDFileBinary("beside_cloud" + to_string(i) + ".pcd", *m_beside_cloud_vec[i]);
	//}
}

void WorkPiece::load_beside_plane(vector<pcl::PointCloud<pointT>::Ptr>& cloud_vec)
{
	if (cloud_vec.empty())
	{
		cerr << "[error] no cloud_vec data." << endl;
		return;
	}
	vector<pointT> center_point_vec;
	center_point_vec.resize(cloud_vec.size());

	//! 计算各侧面的重心
	for (int i = 0; i < cloud_vec.size();++i)
	{
		m_cloud_tools.find_center_point3d(*cloud_vec[i], center_point_vec[i]);
	}

	float dis = 0.0, min_dis = FLT_MAX, min_index = 0;
	vector<int> cloud_index_vec;
	cloud_index_vec.resize(cloud_vec.size());

	for (int i = 0; i < cloud_index_vec.size(); ++i)
	{
		cloud_index_vec[i] = i;
	}

	//! 得到依次顺序
	for (int i = 0; i < center_point_vec.size() - 1; ++i)
	{
		for (int j = i + 1; j < center_point_vec.size(); ++j)
		{
			dis = pcl::geometry::distance(center_point_vec[i], center_point_vec[j]);
			if (dis < min_dis)
			{
				min_dis = dis;
				min_index = j;
			}
		}
		std::swap(center_point_vec[i + 1], center_point_vec[min_index]);

		std::swap(cloud_index_vec[i + 1], cloud_index_vec[min_index]);

		min_dis = FLT_MAX;
		min_index = i;
	}

	//! 按照顺序加载
	for (int i=0;i< cloud_index_vec.size();++i)
	{
		m_beside_cloud_vec.push_back(cloud_vec[cloud_index_vec[i]]);

		Eigen::Vector4f plane_func;

		m_cloud_tools.find_plane_function(cloud_vec[cloud_index_vec[i]], plane_func, 10e-5);

		m_beside_plane_func_vec.push_back(plane_func);
	}

	//for (int i=0;i< m_beside_cloud_vec.size();++i)
	//{
	//	pcl::io::savePCDFileASCII("test_plane_order" + std::to_string(i) + ".pcd", *m_beside_cloud_vec[i]);
	//}
}

void WorkPiece::load_bottom_plane(pcl::PointCloud<pointT>::Ptr bottom_cloud)
{
	m_bottom_cloud = bottom_cloud;
	m_cloud_tools.find_plane_function(bottom_cloud, m_bottom_plane_func,10e-4);
}

void WorkPiece::get_bottom_cloud(pcl::PointCloud<pointT>::Ptr bottom_cloud)
{
	bottom_cloud = m_bottom_cloud;
}

void WorkPiece::get_original_cloud(pcl::PointCloud<pointT>::Ptr original_cloud)
{
	original_cloud = m_original_cloud;
}

void WorkPiece::get_no_cylinder_original_cloud(vector<int>& no_cylinder_original_cloud_index)
{
	no_cylinder_original_cloud_index = m_original_cloud_without_cylinder_index;
}

void WorkPiece::get_bottom_plane_func(Eigen::Vector4f& bottom_plane_func)
{
	bottom_plane_func = m_bottom_plane_func;
}

void WorkPiece::get_beside_cloud_vec(vector<pcl::PointCloud<pointT>::Ptr>& beside_cloud_vec)
{
	beside_cloud_vec = m_beside_cloud_vec;
}

void WorkPiece::get_beside_plane_func_vec(vector<Eigen::Vector4f>& beside_plane_func)
{
	beside_plane_func = m_beside_plane_func_vec;
}

void WorkPiece::get_cloud_center_point(pointT & p)
{
	p = m_cloud_center_point;
}

void WorkPiece::self_calc_bottom_intersection_of_line_func()
{
	//! 需要 底面数据/侧面数据
	if (!m_bottom_cloud)
	{
		cerr << "[error] no bottom cloud data." << endl;
		return;
	}
	else if (m_beside_cloud_vec.empty())
	{
		cerr << "[error] no beside cloud data." << endl;
		return;
	}

	const int beside_plane_size = m_beside_plane_func_vec.size();

	for (int i = 0; i < beside_plane_size; ++i)
	{
		Line_func line_func;
		m_cloud_tools.find_intersection_of_line_with_two_plane(
			m_bottom_plane_func, m_beside_plane_func_vec[i], line_func);

		m_bottom_intersection_of_line_vec.push_back(line_func);
	}
}

void WorkPiece::self_calc_bottom_intersection_of_point()
{
	//! 需要底面，以及侧面数据
	if (!m_bottom_cloud)
	{
		cerr<<"[error] no bottom cloud data."<<endl;
		return;
	}
	if (m_beside_plane_func_vec.empty())
	{
		cerr << "[error] no beside plane function data." << endl;
		return;
	}

	pointT p;
	size_t n = m_beside_plane_func_vec.size();

	for (size_t i = 0; i < n; ++i)
	{
		if (i == 0)
		{
			m_cloud_tools.find_intersection_of_point_with_three_plane(
				m_beside_plane_func_vec[i],
				m_beside_plane_func_vec.back(),
				m_bottom_plane_func,
				p);
		}
		else
		{
			m_cloud_tools.find_intersection_of_point_with_three_plane(
				m_beside_plane_func_vec[i],
				m_beside_plane_func_vec[i - 1],
				m_bottom_plane_func,
				p);
		}
		
		m_bottom_intersection_of_point_vec.push_back(p);
	}

	//! 获取顶点点云数据
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//for (int i=0;i< m_bottom_intersection_of_point_vec.size();++i)
	//{
	//	cloud->points.push_back(m_bottom_intersection_of_point_vec[i]);
	//}
	//cloud->height = 1;
	//cloud->width = cloud->points.size();
	//pcl::io::savePCDFileBinary("vex.pcd",*cloud);
}

void WorkPiece::self_calc_cylinder_func(double threshold_dis)
{
	//! 需要原始数据/底面数据/侧面数据
	if (!m_original_cloud)
	{
		cerr<<"[error] no original cloud data."<<endl;
		return;
	} 
	else if (!m_bottom_cloud)
	{
		cerr<<"[error] no bottom cloud data."<<endl;
		return;
	}
	else if (m_beside_cloud_vec.empty())
	{
		cerr<<"[error] no beside cloud data."<<endl;
		return;
	}

	double 
		dis = 0.0;

	size_t 
		beside_plane_size = m_beside_plane_func_vec.size();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr 
		contain_cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i < m_original_cloud->points.size(); ++i)
	{
		pointT &p = m_original_cloud->points[i];

		bool is_plane_point = false;

		// 判断是否在侧面的平面上
		for (int j = 0; j < beside_plane_size; ++j)
		{
			dis = m_cloud_tools.distance_point2plane(p, m_beside_plane_func_vec[j]);
			if (dis < threshold_dis)
			{
				is_plane_point = true;
				break;
			}
		}

		// 判断是否在底面上
		dis = m_cloud_tools.distance_point2plane(p, m_bottom_plane_func);
		if (dis < threshold_dis)
		{
			is_plane_point = true;
		}

		if (!is_plane_point)
		{
			contain_cylinder_cloud->points.push_back(p);
		}
	}

	//m_cloud_tools.standardize_not_organized_cloud_header<pointT>(*contain_cylinder_cloud);
	//pcl::io::savePCDFileBinary("contain_cylinder_cloud.pcd", *contain_cylinder_cloud);

	Cylinder_func cylinder_func;
	
	m_cloud_tools.fitting_cylinder(contain_cylinder_cloud, cylinder_func);

	m_cylinder_func_vec.push_back(cylinder_func);

	//cout << cylinder_func << endl;

}

void WorkPiece::clear()
{
	if (m_original_cloud)
	{
		m_original_cloud->clear();
	}

	if (m_bottom_cloud)
	{
		m_bottom_cloud->clear();
	}

	m_original_cloud_without_cylinder_index.clear();

	m_top_plane_func.clear();

	m_beside_cloud_vec.clear();

	m_beside_plane_func_vec.clear();

	m_bottom_intersection_of_line_vec.clear();

	m_bottom_intersection_of_point_vec.clear();

	m_cylinder_func_vec.clear();

	m_bottom_defect_index.clear();

	m_bottom_defect_info_vec.clear();

	m_max_dis_bottom_line_vec.clear();

	m_bottom_innner_points.clear();

	m_bottom_inner_distance.clear();

}

void WorkPiece::self_calc_plane_func()
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointNormal>);

	// 点云降采样
	m_cloud_tools.filter_voxel_grid_downsample(m_original_cloud, m_original_cloud, 0.2);
	//cout << "降采样后点云大小:" << m_original_cloud->width << endl;

	// 加载中心点云
	m_cloud_tools.find_center_point3d(*m_original_cloud, m_cloud_center_point);

	// 强制计算法向量
	m_cloud_tools.create_cloud_with_normal(m_original_cloud, cloud_normal, 0.8);

	// 分割平面
	list<vector<int>> total_plane_index;
	vector<int> rest_point_index;
	m_cloud_tools.segment_plane(cloud_normal, total_plane_index, rest_point_index, 1.0, 15, 3000, 5.0);
	cout << "分割平面数:" << total_plane_index.size() << endl;

	// 生成对应点云数据
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_cloud_vec;
	plane_cloud_vec.resize(total_plane_index.size());

	std::list<vector<int>>::iterator it;
	size_t plane_count = 0;
	for (it = total_plane_index.begin(); it != total_plane_index.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr one_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		m_cloud_tools.index_vector_to_cloud<pcl::PointXYZ, pcl::PointXYZ>
			(*it, *m_original_cloud, *one_plane_cloud);
		plane_cloud_vec[plane_count++] = one_plane_cloud;
	}
	
	load_all_plane(plane_cloud_vec);
}

void WorkPiece::self_remove_cylinder_cloud(double threshold_dis)
{
	if (m_cylinder_func_vec.empty())
	{
		cerr<<"[error] no cylinder func data."<<endl;
		return;
	}

	double dis = 0.0, r = m_cylinder_func_vec[0].v[6];

	Eigen::Vector4f pt, line_pt, line_dir;

	for (int i = 0; i < m_original_cloud->points.size(); ++i)
	{
		pointT &p = m_original_cloud->points[i];

		pt[0] = p.x;
		pt[1] = p.y;
		pt[2] = p.z;

		line_pt[0] = m_cylinder_func_vec[0].v[0];
		line_pt[1] = m_cylinder_func_vec[0].v[1];
		line_pt[2] = m_cylinder_func_vec[0].v[2];

		line_dir[0] = m_cylinder_func_vec[0].v[3];
		line_dir[1] = m_cylinder_func_vec[0].v[4];
		line_dir[2] = m_cylinder_func_vec[0].v[5];

		dis = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);

		dis = sqrt(dis);

		if (abs(dis - r) > threshold_dis)
		{
			m_original_cloud_without_cylinder_index.push_back(i);
		}
	}

	// test
	CloudTools cloud_tools;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	m_cloud_tools.index_vector_to_cloud<pointT>(m_original_cloud_without_cylinder_index, *m_original_cloud, *cloud);
	pcl::io::savePCDFileBinary("removed_cylinder_cloud.pcd", *cloud);
}

void WorkPiece::detect_defect_part_on_bottom_plane()
{
	float height = 5.0f;

	vector<int> within_height_index, empty_index;

	// 选取平面上某一高度的点云
	m_cloud_tools.find_points_on_plane(m_original_cloud, m_original_cloud_without_cylinder_index,
		m_bottom_plane_func, within_height_index, height);

	vector<int> rest_index;

	// 去除侧面的点云，留下底面的点云数据
	m_cloud_tools.remove_points_from_plane_func(
		m_original_cloud, within_height_index,
		m_beside_plane_func_vec, rest_index, 0.3);

	vector<int> defect_part_index;
	
	// 仅分析在平面之下的数据
	double 
		A, B, flag;
	A = m_bottom_plane_func[0];
	B = m_bottom_plane_func[1];

	if (A * B < 0)
	{
		flag = 1;
	}
	else
	{
		flag = -1;
	}

	double dis = 0.0, threshold_dis = 0.15;

	for (int i = 0; i < rest_index.size(); ++i)
	{
		int index = rest_index[i];

		pointT & p = m_original_cloud->points[index];

		dis = m_cloud_tools.distance_point2plane_signed(p, m_bottom_plane_func);

		if (dis * flag < 0)
		{
			if (abs(dis) > threshold_dis)
			{
				defect_part_index.push_back(index);
			}
		}
	}

	// 通过聚类过滤部分点集
	m_cloud_tools.cloud_cluster(m_original_cloud, defect_part_index, m_bottom_defect_index, 0.4, 10);

	// 将缺陷点集投影到底面上，并计算缺料信息
	size_t n = m_bottom_defect_index.size();
	m_bottom_defect_info_vec.resize(n);

	pcl::ProjectInliers<pointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);

	coefficients->values[0] = m_bottom_plane_func[0];
	coefficients->values[1] = m_bottom_plane_func[1];
	coefficients->values[2] = m_bottom_plane_func[2];
	coefficients->values[3] = m_bottom_plane_func[3];
	proj.setModelCoefficients(coefficients);

	pcl::PointCloud<pointT>::Ptr cloud(new pcl::PointCloud<pointT>);
	pcl::PointCloud<pointT>::Ptr cloud_projected(new pcl::PointCloud<pointT>);

	for (size_t i = 0; i < n; ++i)
	{
		// 投影到底面平面
		m_cloud_tools.index_vector_to_cloud(m_bottom_defect_index[i], *m_original_cloud, *cloud);

		proj.setInputCloud(cloud);

		proj.filter(*cloud_projected);

		// 计算投影后的最小包围盒
		pcl::MomentOfInertiaEstimation <pointT> feature_extractor;
		feature_extractor.setInputCloud(cloud_projected);
		feature_extractor.compute();

		pointT 
			min_point_OBB,
			max_point_OBB,
			pos_OBB;
		Eigen::Matrix3f rota_matrix_OBB;

		feature_extractor.getOBB(min_point_OBB, max_point_OBB, pos_OBB, rota_matrix_OBB);

		Eigen::Vector3f 
			position(pos_OBB.x, pos_OBB.y, pos_OBB.z),
			min_p(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z),
			max_p(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
		
		//cout << min_p << max_p << endl;

		// 计算面积
		double h, w, l;
		h = abs(max_p[0] - min_p[0]) < 0.09 ?  1: max_p[0] - min_p[0];
		w = abs(max_p[1] - min_p[1]) < 0.09 ?  1: max_p[1] - min_p[1];
		l = abs(max_p[2] - min_p[2]) < 0.09 ?  1: max_p[2] - min_p[2];
		m_bottom_defect_info_vec[i].area_size = h * w * l;

		// 保存顶点以便于绘制
		//Eigen::Vector3f p1(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
		Eigen::Vector3f p2(min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
		Eigen::Vector3f p3(max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
		//Eigen::Vector3f p4(max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
		//Eigen::Vector3f p5(min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
		Eigen::Vector3f p6(min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
		Eigen::Vector3f p7(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
		//Eigen::Vector3f p8(max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

		//p1 = rota_matrix_OBB * p1 + position;
		p2 = rota_matrix_OBB * p2 + position;
		p3 = rota_matrix_OBB * p3 + position;
		//p4 = rota_matrix_OBB * p4 + position;
		//p5 = rota_matrix_OBB * p5 + position;
		p6 = rota_matrix_OBB * p6 + position;
		p7 = rota_matrix_OBB * p7 + position;
		//p8 = rota_matrix_OBB * p8 + position;

		//pointT pt1(p1(0), p1(1), p1(2));
		pointT pt2(p2(0), p2(1), p2(2));
		pointT pt3(p3(0), p3(1), p3(2));
		//pointT pt4(p4(0), p4(1), p4(2));
		//pointT pt5(p5(0), p5(1), p5(2));
		pointT pt6(p6(0), p6(1), p6(2));
		pointT pt7(p7(0), p7(1), p7(2));
		//pointT pt8(p8(0), p8(1), p8(2));

		//m_bottom_defect_info_vec[i].vex_vec.push_back(pt1);
		m_bottom_defect_info_vec[i].vex_vec.push_back(pt2);
		m_bottom_defect_info_vec[i].vex_vec.push_back(pt3);
		//m_bottom_defect_info_vec[i].vex_vec.push_back(pt4);
		//m_bottom_defect_info_vec[i].vex_vec.push_back(pt5);
		m_bottom_defect_info_vec[i].vex_vec.push_back(pt7);
		m_bottom_defect_info_vec[i].vex_vec.push_back(pt6);
		//m_bottom_defect_info_vec[i].vex_vec.push_back(pt8);

		// 缺料区域的平均高度
		vector<double> dis_vec;
		double dis = 0.0, total_dis = 0.0;
		for (size_t j = 0; j < cloud->points.size(); ++j)
		{
			dis = m_cloud_tools.distance_point2plane(cloud->points[j], m_bottom_plane_func);
			dis_vec.push_back(dis);
			total_dis += dis;
		}
		auto max_dis = std::max_element(dis_vec.begin(), dis_vec.end());
		m_bottom_defect_info_vec[i].max_height = *max_dis;
		m_bottom_defect_info_vec[i].mean_height = double(total_dis / dis_vec.size());
	}
}

void WorkPiece::detect_max_dis_bottom_line()
{
	if (m_bottom_intersection_of_line_vec.empty())
	{
		cerr<<"[error] no bottom inetersection of line data."<<endl;
		return;
	}
	if (m_bottom_intersection_of_point_vec.empty())
	{
		cerr << "[error] no bottom inetersection of point data." << endl;
		return;
	}

	// distance of line
	const size_t line_size = m_bottom_intersection_of_line_vec.size();

	Couple_distance couple_distance;
	Eigen::Vector4f line_a_pt, line_a_dir;
	Eigen::Vector4f line_b_pt, line_b_dir;

	// 点到直线距离
	double d1 = 0.0, d2 = 0.0, d3 = 0.0, d4 = 0.0;
	//double d = 0.0;

	for (size_t i = 0; i < line_size / 2; ++i)
	{
		Line_func & l_a = m_bottom_intersection_of_line_vec[i];

		l_a.convert_to_vector4f(line_a_pt, line_a_dir);

		// 根据规律，
		// 0->3, 1->4, 2->5
		size_t j = i + 3;

		//for (size_t j = i + 1; j < line_size; ++j)
		//{
		Line_func & l_b = m_bottom_intersection_of_line_vec[j];
		//cout << i << " l_a:" << l_a << " l_b" << l_b << endl;
		//cout << m_cloud_tools.distance_between_two_plane(m_beside_plane_func_vec[i], m_beside_plane_func_vec[j]) << endl;

		// 判断两直线是否平行
		if (m_cloud_tools.is_parallel(l_a, l_b, 0.09))
		{
			//cout << i << "->" << j << endl;
			
			//cout << i << " l_a:" << l_a << " l_b" << l_b << endl;

			// i->j
			d1 = m_cloud_tools.distance_point2plane(m_bottom_intersection_of_point_vec[i], m_beside_plane_func_vec[j]);

			d2 = m_cloud_tools.distance_point2plane(m_bottom_intersection_of_point_vec[i + 1], m_beside_plane_func_vec[j]);

			// j->i
			d3 = m_cloud_tools.distance_point2plane(m_bottom_intersection_of_point_vec[j], m_beside_plane_func_vec[i]);
			
			d4 = m_cloud_tools.distance_point2plane(
				m_bottom_intersection_of_point_vec[(j + 1) == line_size ? 0 : j + 1],
				m_beside_plane_func_vec[i]);

			// d = m_cloud_tools.distance_between_two_plane(m_beside_plane_func_vec[i], m_beside_plane_func_vec[j]);
			// cout << m_cloud_tools.distance_between_two_plane(m_beside_plane_func_vec[i], m_beside_plane_func_vec[j]) << endl;

			//l_b.convert_to_vector4f(line_b_pt, line_b_dir);

			//d1 = pcl::sqrPointToLineDistance(line_a_pt, line_b_pt, line_b_dir);
			//d1 = sqrt(d1);
			//cout << d1 << endl;

			//d2 = pcl::sqrPointToLineDistance(line_b_pt, line_a_pt, line_a_dir);
			//d2 = sqrt(d2);
			//cout << d2 << endl;

			//cout << (d1 + d2) / 2 << endl;

			couple_distance.i1 = i;
			couple_distance.i2 = j;
			couple_distance.distance = (d1 + d2 + d3 + d4) / 4;
			//couple_distance.distance = d;// (d1 + d2) / 2;
			m_max_dis_bottom_line_vec.push_back(couple_distance);
		}
		//}
	}
}

void WorkPiece::get_max_dis_bottom_line_segment(vector<pointT>& point_vec, vector<double>& dis_vec)
{
	size_t vec_n = m_bottom_intersection_of_point_vec.size();

	// 最大边的对数
	size_t line_n = m_max_dis_bottom_line_vec.size();

	for (size_t i = 0; i < line_n; ++i)
	{
		Couple_distance& cd = m_max_dis_bottom_line_vec[i];
		cout << cd.i1 << "->" << cd.i2 << ":" << cd.distance << endl;

		// 第一条边
		point_vec.push_back(m_bottom_intersection_of_point_vec[cd.i1]);

		// 需要注意顶点不要越界
		if (cd.i1 != vec_n - 1)
		{
			point_vec.push_back(m_bottom_intersection_of_point_vec[cd.i1 + 1]);
		}
		else
		{
			point_vec.push_back(m_bottom_intersection_of_point_vec[0]);
		}

		// 第二条边
		point_vec.push_back(m_bottom_intersection_of_point_vec[cd.i2]);

		// 需要注意顶点不要越界
		if (cd.i2 != vec_n - 1)
		{
			point_vec.push_back(m_bottom_intersection_of_point_vec[cd.i2 + 1]);
		}
		else
		{
			point_vec.push_back(m_bottom_intersection_of_point_vec[0]);
		}

		// 两条边的距离
		dis_vec.push_back(cd.distance);
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//for (int i = 0; i < point_vec.size(); ++i)
	//{
	//	cloud->points.push_back(point_vec[i]);
	//}
	//cloud->height = 1;
	//cloud->width = cloud->points.size();
	//pcl::io::savePCDFileBinary("max_line.pcd",*cloud);
}

void WorkPiece::detect_bottom_inner_point_along_with_line(double dis)
{
	if (!m_bottom_cloud)
	{
		cerr<<"[error] no bottom cloud data."<<endl;
		return;
	}

	size_t n = m_bottom_intersection_of_line_vec.size();

	m_bottom_innner_points.resize(n);

	// 找到理论点
	for (size_t i = 0; i != n; ++i)
	{
		pointT pb, pe, mid_p;

		if (i != n - 1)
		{
			pb = m_bottom_intersection_of_point_vec[i];
			pe = m_bottom_intersection_of_point_vec[i + 1];
		}
		else
		{
			pb = m_bottom_intersection_of_point_vec[i];
			pe = m_bottom_intersection_of_point_vec[0];
		}

		m_cloud_tools.middle_point_between_two_point(pb, pe, mid_p);

		Eigen::Vector3f
			inner_dir, // 指向底面中心的方向向量

			dir_flag; // 用于判断点的前后
		
		dir_flag[0] = m_bottom_center_point.x - mid_p.x;
		dir_flag[1] = m_bottom_center_point.y - mid_p.y;
		dir_flag[2] = m_bottom_center_point.z - mid_p.z;

		// 获取侧面法向，作为mid_p直线底面中心方向向量
		m_cloud_tools.convert_vector4f_to_vector3f(m_beside_plane_func_vec[i], inner_dir);
		
		// 修正方向，使其和dir_flag同向
		if (dir_flag.dot(inner_dir) < 0)
		{
			inner_dir *= -1;
		}

		pointT inner_point;

		// 计算底面向中心垂直方向前进一段距离的坐标点
		m_cloud_tools.find_point_along_with_vector_within_dis(mid_p, inner_dir, inner_point, dis, true);

		m_bottom_innner_points[i] = inner_point;
	}

	// 利用理论点查找实际值，使用kdtree查找点云中的最近值
	pcl::KdTreeFLANN<pointT> kdtree;
	kdtree.setInputCloud(m_bottom_cloud);
	int K = 5;
	std::vector<int> search_index(K);
	std::vector<float> distance_index(K);
	for (size_t i = 0; i < m_bottom_innner_points.size(); ++i)
	{
		pointT &p = m_bottom_innner_points[i];
		if (kdtree.nearestKSearch(p, K, search_index, distance_index) > 0)
		{
			p = m_bottom_cloud->points[search_index[0]];
			//cout<< distance_index.size()<<endl;
		}
		search_index.clear();
		distance_index.clear();
	}

	// 计算点到顶顶面的距离
	m_bottom_inner_distance.resize(m_bottom_innner_points.size(), 0);

	for (size_t i = 0; i < m_bottom_innner_points.size(); ++i)
	{
		dis = m_cloud_tools.distance_point2plane(m_bottom_innner_points[i], m_top_plane_func_ABCD);

		m_bottom_inner_distance[i] = dis;
	}

	//test 验证保存inner点集
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//for (int i = 0; i < m_bottom_innner_points.size(); ++i)
	//{
	//	cloud->points.push_back(m_bottom_innner_points[i]);
	//}
	//cloud->height = 1;
	//cloud->width = cloud->points.size();
	//pcl::io::savePCDFileBinary("inner_points.pcd", *cloud);
}

void WorkPiece::get_bottom_inner_point_along_with_line(vector<pointT>& inner_points, vector<double> & dis_vec)
{
	inner_points = m_bottom_innner_points;

	dis_vec = m_bottom_inner_distance;
}

void WorkPiece::get_bottom_defect(vector<vector<int>>& bottom_defect, vector<Defect_info>&defect_info_vec)
{
	bottom_defect = m_bottom_defect_index;
	defect_info_vec = m_bottom_defect_info_vec;
}

void WorkPiece::get_distance_between_top_and_bottom(double & dis)
{
	if (!m_bottom_cloud)
	{
		cerr<<"[error] no bottom cloud data."<<endl;
		return;
	}
	if (m_top_plane_func.empty())
	{
		cerr << "[error] no top plane func data." << endl;
		return;
	}
	pointT p;
	p.x = m_top_plane_func[0];
	p.y = m_top_plane_func[1];
	p.z = m_top_plane_func[2];
	dis = m_cloud_tools.distance_point2plane(p, m_bottom_plane_func);
}

void WorkPiece::get_bottom_vertex(vector<pointT>& bottom_vertex) const
{
	bottom_vertex = m_bottom_intersection_of_point_vec;
}
