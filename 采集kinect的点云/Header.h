#pragma once

#include <pthread.h>
#include <iostream>
#include <vector>
#include <fstream>
using namespace std;

/*OSG相关头文件*/
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgUtil/DelaunayTriangulator>
#include <osgWidget/WindowManager>
#include <osgWidget/ViewerEventHandlers>
#include <osgWidget/Box>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Vec3>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/PolygonStipple>
#include <osg/io_utils>
#include <osg/Math>
#include <osg/StateSet>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Texture2D>
#include <osg/Config>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#if defined(_MSC_VER) && defined(OSG_DISABLE_MSVC_WARNINGS)
// disable warning "glutCreateMenu_ATEXIT_HACK' : unreferenced local function has been removed"
#pragma warning( disable : 4505 )
#endif

#  include <GL/glut.h>
#define NOMINMAX
#include <Windows.h>

// pcl相关
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>



