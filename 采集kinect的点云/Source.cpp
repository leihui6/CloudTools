#pragma once
#include "Globaler.h"

//2016-12-07 更新日志:
//	将存储的矩阵修改为了存储到前一帧的变换矩阵,点云格式不变

// 2016-12-29 更新日志
// 取消实时变换的功能,将功能转为单一的图像获取并保存.
// 保存情况为:
// 1).按下V启动保存
// 2).自定义的保存pcd或txt格式的数据

int main(int argc, char **argv)
{
	// 初始化三个场景的位置
	preProcess();

	// 创建窗口程序
	createGLUTThread(argc,argv);

	// 创建kinect的线程,并且是可分离的.
	createKinectThread();

	// 创建调用display以重绘的线程
	createShowThread();

	// 合并线程并销毁线程参数
	destroyPthread();
	
	return 0;
}

