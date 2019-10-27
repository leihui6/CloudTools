#pragma once

#include<iostream>
#include<vector>
#include<string>
#include<random>
#include<ctime>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense> 

#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace pcl;