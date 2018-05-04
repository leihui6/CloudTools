#pragma once
#include "Header.h"

#include <Kinect.h>
#include <opencv2\opencv.hpp>
using namespace cv;


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}


class CMotionRecognition
{

public:
	CMotionRecognition();

	~CMotionRecognition();

	// 初始化Kinect
	HRESULT InitializeDefaultSensor();

	HRESULT update();

	// 获取osg格式的点云
	osg::ref_ptr<osg::Node> CalPointCloud(bool _GetPclPointCloud);

	// 保存数据
	int SaveCloudPoints(bool _saveAsPCD, bool _saveAsTXT, std::string _filename, std::string _dir);

	osg::ref_ptr<osg::Node> GetPointCloud() const;
private:

	int nDepthWidth;

	int nDepthHeight;

	int nColorWidth;

	int nColorHeight;

	cv::Mat depthImg;

	cv::Mat colorImg;

	//CC3DPoint	* pC3dPoint;

	UINT16		* pDepthBuffer;

	RGBQUAD		* m_pColorRGBX;

	osg::ref_ptr<osg::Node>   model;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelPointcloud;

	IKinectSensor		* m_pKinectSensor;

	IColorFrameReader	* m_pColorFrameReader;

	IDepthFrameReader	* m_pDepthFrameReader;

private:

	// 将Kinect数据转换为Mat图像 & 保存到 m_pColorRGBX
	void ConvertMat_color(const RGBQUAD* pBuffer, int nWidth, int nHeight);
	
	// 将Kinect数据转换为Mat图像 & 保存到 pDepthBuffer
	void ConvertMat_depth(const UINT16* pBuffer, USHORT nMinDepth, USHORT nMaxDepth);

	// 获取图像数据
	HRESULT GetColorImage();

	// 获取深度数据
	HRESULT GetDepthImage();

	HRESULT updateIsOk;

	// 计算得出点云
	osg::ref_ptr<osg::Node> AssembleAsPointCloud(bool _GetPclPointCloud);

	
};

