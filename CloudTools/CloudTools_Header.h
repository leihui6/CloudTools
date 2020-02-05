//========================================================
/**
*  @file      CloudTools_Header.h
*
*  项目描述:	三类刹车片的表面缺陷以及基本几何尺寸的检测
*  文件描述:	核心点云计算工具库头文件
*  适用平台:	Windows10
*  作    者:	LeiHui Li@  Wang_RiWei@
*  项 目 组:	智能计算研发一组
*  公    司:	天津微深联创科技有限公司
*
//========================================================
*/
#pragma once

#include <direct.h>
#include <io.h>

#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <ctime>
#include <list>
#include <fstream>
using namespace std;


#include <Eigen/Dense> 

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/intersections.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
using namespace pcl;


//#include <pthread.h>
