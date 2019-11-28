/* 
* 三维点云获取与转台控制
* 使用串口通信 和 VST3D三维点云 SDK
*/
#include <boost/thread/thread.hpp>

#include "../../CloudTools/CloudTools.h"
#include "../../examples/work_piece/WorkPiece.h"
#include <CameraComm1.h>
#include <SerialComm.h>
#include <VST3D.h>

// 等待转台运动间隔
const float WAIT_MSECOND = 500;

// 发送消息间隔
const float SEND_GAP_MSECOND = 1000;

// 出错再次扫描间隔(希望不要用到)
const float SCAN_GAP_MSECOND = 1000;

//! 用于改变字体的高度
int line_count = 1;
int col_count = 1;

//! 字体大小
int font_size = 12;
int line_add = 12;

//! -1:暂停 0:2D相机  1:开始
/*!
	输入a-> 0
	输入b-> 1
*/
int obtain_point_cloud_signal = -1;

// 当前处理的点云数量
int cloud_count = 0;

// 编号/积料检测摄像头
CameraComm1 camera_comm1;

// 串口通信(控制转台)
SerialComm serial_comm;

// 3d 扫描相机
VST3D vst3d;

// TODO:裂痕检测相机

// 3D 数据处理
WorkPiece work_piece;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("缺料检测"));

//! 负责初始化各个模块，其包括 1. 串口通信模块 2. vst3d相机模块
bool init_env(vector<float> &);

//! 转换vst格式点云为pcd格式
void convert_vst2pcd(VST3D_PT ** pPointClouds, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

//! 获取点云线程
void obtain_point_cloud(std::vector<string> &);

//! 从文件中读取发送指令
void read_command_from_file(const string& filename,vector<string> &command_vec);

//! 添加原始点云数据
void add_original_cloud(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud);

//! 添加缺料信息
void add_defect_info(pcl::visualization::PCLVisualizer::Ptr viewer, vector<vector<int>> &bottom_defect, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud, vector<Defect_info> &defect_info_vec);

//! 添加最长边信息
void add_max_dis_line(pcl::visualization::PCLVisualizer::Ptr viewer, vector<pointT>& point_vec, vector<double>& dis_vec);

//! 添加向内点
void add_inner_point(pcl::visualization::PCLVisualizer::Ptr viewer, vector<pointT>& point_vec, vector<double>& inner_distance_vec);

//! 添加高度信息
void add_height(pcl::visualization::PCLVisualizer::Ptr viewer, double height);

//! 添加2d相机检测信息
void add_work_piece_info(pcl::visualization::PCLVisualizer::Ptr viewer, const string &str);

//! 2D相机线程
void obtain_2D_camera();

//! pcl下的键盘监控
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);

int main()
{
	vector<float> top_plane_func;
	if (0 != init_env(top_plane_func))
	{
		cerr<<"初始化失败"<<endl;
		system("pause");
		return -1;
	}
	work_piece.set_top_plane_func(top_plane_func);

	vector<string> command_vec;
	read_command_from_file("command.txt", command_vec);
	if (command_vec.empty())
	{
		cerr<<"指令集读取失败"<<endl;
		return -1;
	}

	viewer->setBackgroundColor(0, 0, 1);

	viewer->addCoordinateSystem(1.0, "first");
	
	// 启动键盘控制，获取相机和转台操作指令
	viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	// 启动控制转台+获取点云进程
	boost::thread th_obtain_2D_camera(&obtain_2D_camera);
	//th_obtain_2D_camera.join();

	// 启动控制转台+获取点云进程
	boost::thread th_obtain_point_cloud(&obtain_point_cloud, command_vec);
	//th_obtain_point_cloud.join();
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	    //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	//viewer->spin();
	return 0;
}

void init_table()
{
	string init[2] =
	{
		"YH 10 3",
		"XH 10 3"
	};
	int serial_status;

	cout << "[转台]" << "转台初始化" << endl;

	serial_status = serial_comm.send(init[0] + "\r\n");

	Sleep(5000);

	serial_status = serial_comm.send(init[1] + "\r\n");

	Sleep(1000);

	cout << "[转台]" << "转台初始化完毕" << endl;
}

bool init_env(vector<float> & top_plane_func)
{
	// 串口通信模块
	try
	{
		int serial_status = 0;
		serial_status = serial_comm.open_port("COM5");

		if (!serial_status)
		{
			throw string("Open port failed" );
		}

		serial_status = serial_comm.setup_DCB(9600);
		if (!serial_status)
		{
			throw string("Setup DCB failed");
		}

		serial_status = serial_comm.setup_timeout(0, 0, 0, 0, 0);
		if (!serial_status)
		{
			throw string("Setup timeout failed");
		}
		serial_comm.flush_buffer();
	}
	catch (const string &e)
	{
		cout << "[串口] 串口通信初始化失败" << endl;
		cerr << e << endl;
		return -1;
	}

	cout << "[串口]" << "串口通信初始化 OK!" << endl;

	// vst3d相机模块
	try
	{
		int vst3d_status = 0;

		vst3d_status = vst3d.init("C:\\Program Files\\VST\\VisenTOP Studio\\VisenTOP Studio.exe");
		if (VST3D_RESULT_OK != vst3d_status)
		{
			throw string("Could not Start Scanner software");
		}
		Sleep(3000);

		vst3d_status = vst3d.connect_align();
		if (VST3D_RESULT_OK != vst3d_status)
		{
			throw string("Could not connect to Scanner(connect_align)");
		}

		Sleep(3000);

		vst3d_status = vst3d.del_background(top_plane_func);
		if (VST3D_RESULT_OK != vst3d_status)
		{
			throw string("Could not del_background");
		}
	}
	catch (const string& e)
	{
		cout << "[3d] 3D 相机初始化失败" << endl;
		cerr << e << endl;
		return -2;
	}

	cout << "[3D]" << "3D 相机初始化 OK!" << endl;

	//! 初始化转台
	{
		init_table();
	}

	//! TCP/IP 通信
	{
		char ip[32] = "192.168.1.111";
		int port = 8081;
		camera_comm1.init(ip, port);
	}
	return 0;
}

void convert_vst2pcd(
	VST3D_PT ** pPointClouds, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	int totalnum;
	vst3d.get_num_points(totalnum);

	// 遍历单次采集到的所有点云，得到点云坐标等信息，通过索引方式[0 1 2 ....]
	VST3D_PT * pt = nullptr;

	pcl::PointXYZ normal;
	
	for (int i = 0; i < totalnum; i++)
	{
		vst3d.get_each_point_by_index(i, &pt);
		normal.x = pt->x;
		normal.y = pt->y;
		normal.z = pt->z;
		//normal.normal_x = pt->nx;
		//normal.normal_y = pt->ny;
		//normal.normal_z = pt->nz;
		//cr = (float)pt->cr / 255;
		//cg = (float)pt->cg / 255;
		//cb = (float)pt->cb / 255;
		cloud->points.push_back(normal);
	}

	cloud->height = 1;
	cloud->width = cloud->points.size();
}

//! 获取点云数据并且处理线程
void obtain_point_cloud(std::vector<string> &command_vec)
{
	cout << "[3D] 已启动获取点云线程" << endl;

	while (true)
	{
		// 开始获取点云
		if (obtain_point_cloud_signal == 1)
		{
			cout << "[3D]" << "收到指令，开始获取点云数据" << endl;
			cout << "[3D]" << "等待机械手臂运动" << endl;

			//Sleep(10000);
			Sleep(1000);

			string curr_command;

			int cloud_size = 0;

			int serial_status, vst3d_status;

			for (int i = 0; i < command_vec.size(); ++i)
			{
				//cout << "[" << i << "]" << "----------begin----------" << endl;

				curr_command = command_vec[i];

				cout << "[3D]当前指令:" << curr_command << endl;

				// 发送指令
				serial_status = serial_comm.send(curr_command+ "\r\n");

				while (!serial_status)
				{
					cerr << "数据发送失败，等待 " << SEND_GAP_MSECOND / 1000 << " s" << endl;

					Sleep(SEND_GAP_MSECOND);

					serial_status = serial_comm.send(curr_command);
				}

				cout << "[3D]" << curr_command << "数据发送成功" << endl;

				// 等待转台转动
				cout << "[3D]" << "等待转台转动，" << "等待 " << WAIT_MSECOND/1000 << "s" << endl;

				Sleep(WAIT_MSECOND);

				// 获取点云数据
				cout << "[3D]" << "开始扫描" << endl;

				vst3d_status = VST3D_Scan(); // 开始单次扫描

				while (vst3d_status != VST3D_RESULT_OK)
				{
					cerr << "扫描错误" << "等待"
						<< SCAN_GAP_MSECOND / 1000 << "s " << "重新开始扫描" << endl;
					// TODO
					// 扫描出错处理????
					Sleep(SCAN_GAP_MSECOND);

					vst3d_status = VST3D_Scan(); // 开始单次扫描
				}

				cout << "[3D]" << "当前扫描成功，开始获取点云数据" << endl;

				VST3D_PT * pPointClouds = nullptr;

				vst3d_status = vst3d.get_point_cloud(cloud_size, &pPointClouds);

				if (vst3d_status == VST3D_RESULT_OK)
				{
					// TODO:
					// 转换为pcd格式???
					//cout << "点云大小: " << cloud_size << endl;

					convert_vst2pcd(&pPointClouds, cloud);

					*cloud += *cloud;

					//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud, 255, 255, 255);
					//viewer->updatePointCloud(cloud, cloud_in_color_h, "cloud");
					//viewer->updatePointCloud(cloud, "cloud");
				}
				// TODO:
				// 获取错误情况处理
				else
				{
					cerr << "获取点云错误 error code:" << vst3d_status << endl;
					// do nothing
				}
				//cout << "[" << i << "]" << "----------over----------" << endl;
			}

			cout << "[3D]" << "所有指令已经发送" << endl;
			init_table();

			cout << "[3D]" << "点云保存完成" << endl;
			pcl::io::savePCDFileBinary(
				"completed_cloud" + std::to_string(cloud_count) + ".pcd", 
				*cloud);
			
			cout << "[3D]" << "点云保存完成" << endl;

			work_piece.load_original_cloud(cloud);
			
			// 计算底面到顶面的高度
			double distance;
			work_piece.get_distance_between_top_and_bottom(distance);
			add_height(viewer, distance);
			cout << "底面到顶面的高度:" << distance << endl;

			// 检测底面缺料情况
			work_piece.detect_defect_part_on_bottom_plane();
			vector<vector<int>> bottom_defect;
			vector<Defect_info> defect_info_vec;
			work_piece.get_bottom_defect(bottom_defect, defect_info_vec);
			cout << "缺料/角部分数量:" << bottom_defect.size() << endl;
			add_defect_info(viewer, bottom_defect, cloud, defect_info_vec);

			// 获取最大边长
			work_piece.detect_max_dis_bottom_line();
			vector<pointT> point_vec;
			vector<double> dis_vec;
			work_piece.get_max_dis_bottom_line_segment(point_vec, dis_vec);
			add_max_dis_line(viewer, point_vec, dis_vec);
			cout << "最大边长数量:" << dis_vec.size() << endl;

			// 获取向内移动3mm的点
			work_piece.detect_bottom_inner_point_along_with_line(3.0);
			vector<pcl::PointXYZ> inner_point_vec;
			vector<double> inner_distance_vec;
			work_piece.get_bottom_inner_point_along_with_line(inner_point_vec, inner_distance_vec);
			for (auto D : inner_distance_vec) cout << "inner D:" << D << endl;
			add_inner_point(viewer, inner_point_vec, inner_distance_vec);

			// 添加原始点云
			add_original_cloud(viewer, cloud);

			// 获取点云重心点，设置坐标系
			pointT p;
			work_piece.get_cloud_center_point(p);
			viewer->addCoordinateSystem(5, p.x, p.y, p.z);

			vst3d.clear();

			//work_piece.clear();
			// system("cls");
			obtain_point_cloud_signal = -1;
		}
		
		// 暂停
		//if (obtain_point_cloud_signal == -1)
		//{
			// TODO
		//}
	}
}

//! 2D相机通信
void obtain_2D_camera();

void read_command_from_file(const string & filename, vector<string> &command_vec)
{
	ifstream ifile(filename);

	if (!ifile.is_open())
	{
		cerr <<"Open file failed"<< endl;
		return;
	}

	string str;

	while (getline(ifile, str))
	{
		command_vec.push_back(str);
	}

	ifile.close();
}

void add_original_cloud(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud)
{
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(original_cloud, 0, 255, 0);
	string str = "original_cloud";
	viewer->addPointCloud(original_cloud, rgb, str);
}

void add_defect_info(pcl::visualization::PCLVisualizer::Ptr viewer, vector<vector<int>>& bottom_defect, pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud, vector<Defect_info>& defect_info_vec)
{
	string t = "defect size:" + to_string(bottom_defect.size()) + "(red zone)";
	line_count += line_add;
	viewer->addText(t, col_count, line_count, font_size, 1, 1, 1);

	for (size_t i = 0; i < defect_info_vec.size(); ++i)
	{
		t = "defect:" + to_string(i) + " size:" + to_string(defect_info_vec[i].area_size) + " max/mean height:" + to_string(defect_info_vec[i].max_height) + "/" + to_string(defect_info_vec[i].mean_height);
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
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 255, 0, 0);
		viewer->addPointCloud(cloud, rgb, str);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 5, str);
	}
}

void add_max_dis_line(pcl::visualization::PCLVisualizer::Ptr viewer, vector<pointT>& point_vec, vector<double>& dis_vec)
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

void add_inner_point(pcl::visualization::PCLVisualizer::Ptr viewer, vector<pcl::PointXYZ>& point_vec, vector<double>& inner_distance_vec)
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
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_POINTS, 8, str);
}

void add_height(pcl::visualization::PCLVisualizer::Ptr viewer, double height)
{
	string t = "height:" + to_string(height);
	line_count += line_add;
	viewer->addText(t, col_count, line_count, font_size, 1, 1, 1);
}

void add_work_piece_info(pcl::visualization::PCLVisualizer::Ptr viewer, const string & str)
{
	line_count += line_add;
	viewer->addText(str, col_count, line_count, font_size, 1, 1, 1);
}

void obtain_2D_camera()
{
	
	string id, result;
	while (true)
	{
		if (obtain_point_cloud_signal == 0)
		{
			// 发送111指令
			camera_comm1.send("111");
			Sleep(500);
			// 接受111结果
			camera_comm1.receive(id);

			Sleep(5000);

			// 发送222指令
			camera_comm1.send("222");
			Sleep(500);
			// 接受222结果
			camera_comm1.receive(result);

			const string t = id + ":" + result;
			add_work_piece_info(viewer, t);

			// 暂停等待
			obtain_point_cloud_signal = -1;
		}
	}
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent & event, void * nothing)
{
	string input = event.getKeySym();
	if (input == "a")
	{
		cout << "[2D] 2D检测开始" << endl;
		obtain_point_cloud_signal = 0;
	}
	else if (input == "b")
	{
		cout << "[3D] 3D检测开始" << endl;
		obtain_point_cloud_signal = 1;
	}
}
