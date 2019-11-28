/*!
* 根据点云高度变化显示点云表面异常情况
* 计算数据源为：
*	整体点云数据
*	各平面数据
*	
* 计算过程如下：
*	计算各平面方程
*	剔除非目标点云数据
*	对目标点云进行分析着色
*
*/
#include "../../CloudTools/CloudTools.h"

#include "WorkPiece.h"

const string folder_name = "六边形_有缺角2\\";
//const string folder_name = ".\\";

const string cloud_name = "六边形_有缺角2";
//const string cloud_name = "completed_cloud0";

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

int line_count = 1;

int col_count = 1;

int font_size = 14;
int line_add = 14;

void show_max_dis_line(vector<pointT>& point_vec, vector<double>& dis_vec);

void show_defect(vector<vector<int>> &bottom_defect, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud, vector<Defect_info> &defect_info_vec);

void show_inner_point(vector<pcl::PointXYZ> &point_vec, vector<double> &inner_distance_vec);

void show_height(double height);

void show_bottom_vertex(vector<pcl::PointXYZ> & vex_points);

int main(/*int argc, char** argv*/)
{
	viewer->setBackgroundColor(0, 0, 1);
	viewer->addCoordinateSystem(1.0, "first");

	/**************加载整体点云**************/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(folder_name + cloud_name + ".pcd", *cloud);
	cout << "loaded cloud_original size:" << cloud->points.size() << endl;

	WorkPiece work_piece;

	// 保存处理过程数据
	work_piece.set_is_save_process_data(true);

	// 装载点云数据
	work_piece.load_original_cloud(cloud);

	// 计算底面和顶面的高度
	double distance;
	work_piece.get_distance_between_top_and_bottom(distance);
	show_height(distance);
	cout << "底面到顶面的高度:" << distance << endl;

	// 检测底面缺料情况
	work_piece.detect_defect_part_on_bottom_plane();
	vector<vector<int>> bottom_defect;
	vector<Defect_info> defect_info_vec;
	work_piece.get_bottom_defect(bottom_defect, defect_info_vec);
	cout << "缺料/角部分数量:" << bottom_defect.size() << endl;
	show_defect(bottom_defect, cloud, defect_info_vec);

	// 获取最大边长
	work_piece.detect_max_dis_bottom_line();
	vector<pointT> point_vec; 
	vector<double> dis_vec; 
	work_piece.get_max_dis_bottom_line_segment(point_vec, dis_vec);
	for (auto d : dis_vec) cout << "dis:" << d << endl;
	show_max_dis_line(point_vec, dis_vec);
	
	// 获取顶点数据
	vector<pointT> vex_points;
	work_piece.get_bottom_vertex(vex_points);
	show_bottom_vertex(vex_points);

	// 获取向内移动3mm的点
	work_piece.detect_bottom_inner_point_along_with_line(3.0);
	vector<pcl::PointXYZ> inner_point_vec;
	vector<double> inner_distance_vec;
	work_piece.get_bottom_inner_point_along_with_line(inner_point_vec, inner_distance_vec);
	for (auto D : inner_distance_vec) cout << "inner D:" << D << endl;
	show_inner_point(inner_point_vec, inner_distance_vec);

	string str = "cloud";
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 255, 0);
	viewer->addPointCloud(cloud, rgb, str);

	pointT p;
	work_piece.get_cloud_center_point(p);
	viewer->addCoordinateSystem(5, p.x, p.y, p.z);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	return 0;
}

void show_max_dis_line(vector<pointT>& point_vec, vector<double>& dis_vec)
{
	size_t j = 0;
	for (size_t i = 0; i < point_vec.size(); i += 4)
	{
		viewer->addLine<pcl::PointXYZ>(point_vec[i], point_vec[i + 1], 255, 0, 0, "line" + to_string(i));
		viewer->addLine<pcl::PointXYZ>(point_vec[i + 2], point_vec[i + 3], 255, 255, 255, "line" + to_string(i + 1));
		
		string show_dis = "edge:" + to_string(j) + " distance:" + std::to_string(dis_vec[j]);
		line_count += line_add;
		viewer->addText(show_dis, col_count, line_count, font_size, 1, 1, 1);
		j++;
	}
}

void show_defect(
	vector<vector<int>> &bottom_defect,
	pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud,
	vector<Defect_info> &defect_info_vec)
{
	string t = "defect size:" + to_string(bottom_defect.size()) + "(red zone)";
	line_count += line_add;
	viewer->addText(t, col_count, line_count, font_size, 1, 1, 1);

	for (size_t i = 0; i < defect_info_vec.size(); ++i)
	{
		t = "defect:" + to_string(i) + " size:" + to_string(defect_info_vec[i].area_size) + "; max/mean height:" + to_string(defect_info_vec[i].max_height) + "/" + to_string(defect_info_vec[i].mean_height);
		line_count += line_add;
		viewer->addText(t, col_count, line_count, font_size, 1, 1, 1);
	}
	CloudTools cloud_tools;
	// 保存缺料缺角部分点云数据
	for (int i = 0; i < bottom_defect.size(); ++i)
	{
		string str = "defect" + to_string(i);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_tools.index_vector_to_cloud < pcl::PointXYZ, pcl::PointXYZ>
			(bottom_defect[i], *original_cloud, *cloud);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud,255,0,0);
		viewer->addPointCloud(cloud, rgb, str);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 5, str);
	}
}

void show_inner_point(vector<pcl::PointXYZ>& point_vec, vector<double> &inner_distance_vec)
{
	string t = "inner point:" + to_string(point_vec.size());
	line_count += line_add;
	viewer->addText(t, col_count, line_count, font_size, 1, 1, 1);

	for (size_t i = 0; i < inner_distance_vec.size(); ++i)
	{
		t = to_string(i) + " inner height(to top plane):" + to_string(inner_distance_vec[i]);
		line_count += line_add;
		viewer->addText(t, col_count, line_count, font_size, 1, 1, 1);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < point_vec.size(); ++i)
	{
		cloud->points.push_back(point_vec[i]);
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 255, 0, 0);
	string str = "inner_point";
	viewer->addPointCloud(cloud, rgb, str);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 10, str);
}

void show_height(double height)
{
	string t = "height:" + to_string(height);
	line_count += line_add;
	viewer->addText(t, col_count, line_count, font_size, 1, 1, 1);
}

void show_bottom_vertex(vector<pcl::PointXYZ>& vex_points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < vex_points.size(); ++i)
	{
		cloud->points.push_back(vex_points[i]);
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 255, 0, 0);
	string str = "vex_point";
	viewer->addPointCloud(cloud, rgb, str);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 10, str);
}
