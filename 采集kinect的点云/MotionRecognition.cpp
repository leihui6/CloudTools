#pragma once
#include "MotionRecognition.h"

CMotionRecognition::CMotionRecognition()
	:
	pDepthBuffer(NULL),
	m_pColorRGBX(NULL),
	//pC3dPoint(NULL),
	m_pKinectSensor(NULL),
	m_pColorFrameReader(NULL),
	m_pDepthFrameReader(NULL),

	depthImg(),
	colorImg(),
	model(),
	modelPointcloud(new pcl::PointCloud<pcl::PointXYZRGB>()),

	nColorHeight(1080),
	nColorWidth(1920),
	nDepthHeight(424),
	nDepthWidth(512),

	updateIsOk(E_FAIL)

{
	pDepthBuffer = new UINT16 [nDepthWidth * nDepthHeight]();

	m_pColorRGBX = new RGBQUAD[nColorWidth * nColorHeight]();

}

/// Destructor
CMotionRecognition::~CMotionRecognition()
{
	if (pDepthBuffer)
	{
		delete[] pDepthBuffer;
		pDepthBuffer = NULL;
	}

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	//if (pC3dPoint){
	//	delete[] pC3dPoint;
	//	pC3dPoint = NULL;
	//}

	SafeRelease(m_pColorFrameReader);

	SafeRelease(m_pDepthFrameReader);

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
}

/// Initializes the default Kinect sensor
HRESULT CMotionRecognition::InitializeDefaultSensor()
{
	HRESULT hr = GetDefaultKinectSensor(&m_pKinectSensor);

	//找到kinect设备
	if (m_pKinectSensor && SUCCEEDED(hr))
	{
		IDepthFrameSource * pDepthFrameSource = NULL;
		IColorFrameSource * pColorFrameSource = NULL;
		//打开kinect
		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr)){
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}
		if (SUCCEEDED(hr))
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		if (SUCCEEDED(hr))
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);

		SafeRelease(pDepthFrameSource);
		SafeRelease(pColorFrameSource);
	}
	else
	{
		std::cout << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}

	return hr;
}

osg::ref_ptr<osg::Node> CMotionRecognition::CalPointCloud(bool _GetPclPointCloud){

	// 只有当update成功后才会执行,
	// 在执行此函数前需要执行成功的执行update()
	if (SUCCEEDED(updateIsOk))
	{
		return AssembleAsPointCloud(_GetPclPointCloud);
	}
	else
	{
		return osg::ref_ptr<osg::Node>();
	}
}

HRESULT CMotionRecognition::update(){

	// 使用imshow前请检查是否将图像赋值

	// 获取深度数据
	HRESULT hr = GetDepthImage();

	// 获取颜色数据
	if (SUCCEEDED(hr))
	{
		hr = GetColorImage();
	}

	updateIsOk = hr;

	return hr;
}

HRESULT CMotionRecognition::GetColorImage(){

	if (!m_pColorFrameReader)
	{
		return E_FAIL;
	}

	IColorFrame* pColorFrame = NULL;

	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD *pBuffer = NULL;

		hr = pColorFrame->get_RelativeTime(&nTime);

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
			}
			else if (m_pColorRGBX)
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = nColorWidth * nColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		if (SUCCEEDED(hr))
		{
			ConvertMat_color(pBuffer, nColorWidth, nColorHeight);
		}

		SafeRelease(pFrameDescription);

	}

	SafeRelease(pColorFrame);

	return hr;

}

HRESULT CMotionRecognition::GetDepthImage(){

	if (!m_pDepthFrameReader)
	{
		return E_FAIL;
	}

	IDepthFrame	* pDepthFrame = nullptr;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr)){
		IFrameDescription * pFrameDescription = nullptr;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT16 *pBuffer = NULL;
		UINT nBufferSize = 0;

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))
		{
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;
			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if (SUCCEEDED(hr))
		{
			ConvertMat_depth(pBuffer, nDepthMinReliableDistance, nDepthMaxDistance);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);

	return hr;
}

void CMotionRecognition::ConvertMat_depth(const UINT16* _pBuffer, USHORT nMinDepth, USHORT nMaxDepth)
{
	//cv::Mat img(nDepthHeight, nDepthWidth, CV_8UC3);
	//uchar* p_mat = img.data;

	const UINT16
		* pBuffer   = _pBuffer,
		* pBufferEnd = _pBuffer + (nDepthWidth * nDepthHeight);

	UINT16 * pDepthBufferTmp	= pDepthBuffer;

	while (pBuffer < pBufferEnd)
	{
		*pDepthBufferTmp = *pBuffer;

		//USHORT depth = *pBuffer;

		//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

		//*p_mat = intensity;
		//++p_mat;
		//*p_mat = intensity;
		//++p_mat;
		//*p_mat = intensity;
		//++p_mat;

		++pDepthBufferTmp;
		++pBuffer;
	}

	//Mat test(nDepthHeight, nDepthWidth, CV_16UC1, pDepthBuffer);

	//cv::imshow("test0",test);
	//waitKey(50);

	//cv::medianBlur(test, test,7);

	//cv::imshow("test1",test);
	//waitKey(50);

	//return img;
}

void CMotionRecognition::ConvertMat_color(const RGBQUAD* _pBuffer, int nWidth, int nHeight)
{
	//cv::Mat img(nHeight, nWidth, CV_8UC3);
	//uchar* p_mat = img.data;

	const RGBQUAD
		* pBuffer	 = _pBuffer,
		* pBufferEnd = pBuffer + (nWidth * nHeight);

	RGBQUAD * pBufferTmp = m_pColorRGBX;

	while (pBuffer < pBufferEnd)
	{
		*pBufferTmp = *pBuffer;
		//*p_mat = pBuffer->rgbBlue;
		//++p_mat;
		//*p_mat = pBuffer->rgbGreen;
		//++p_mat;
		//*p_mat = pBuffer->rgbRed;
		//++p_mat;

		++pBufferTmp;
		++pBuffer;
	}

	//img.copyTo(colorImg);

	//return img;
}

osg::ref_ptr<osg::Node> CMotionRecognition::AssembleAsPointCloud(bool _GetPclPointCloud)
{
	if (!m_pKinectSensor)
	{
		return osg::ref_ptr<osg::Node>();
	}

	// 一帧空间坐标
	osg::ref_ptr<osg::Vec3Array> coodrVec = new osg::Vec3Array();
	// 一帧颜色值
	osg::ref_ptr<osg::Vec4Array> colorVec	= new osg::Vec4Array();

	ICoordinateMapper * m_pCoordinateMapper = nullptr;

	HRESULT hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

	// 当前点的颜色值/坐标
	osg::Vec4f pointColor;
	osg::Vec3f pointCoodr;
	pcl::PointXYZRGB pclPoint;
	int pointCalCount = 0;

	if (_GetPclPointCloud)
	{
		modelPointcloud->clear();
		modelPointcloud->width = static_cast<uint32_t>(nDepthWidth);
		modelPointcloud->height = static_cast<uint32_t>(nDepthHeight);
		modelPointcloud->is_dense = false;
	}
	for (size_t y = 0; y != nDepthHeight; y++)
	{
		for (size_t x = 0; x != nDepthWidth; x++)
		{ 
			DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
			UINT16 currDepth = pDepthBuffer[y * nDepthWidth + x];

			// Coordinate Mapping Depth to Color Space
			ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
			m_pCoordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, currDepth, &colorSpacePoint);
			int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f)),
				colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
			if ((0 <= colorX) && (colorX < nColorWidth) && (0 <= colorY) && (colorY < nColorHeight))
			{
				RGBQUAD color = m_pColorRGBX[colorY * nColorWidth + colorX];
				
				pointColor.b() = (float)color.rgbBlue / 255;
				pointColor.g() = (float)color.rgbGreen / 255;
				pointColor.r() = (float)color.rgbRed / 255;
				pointColor.a() = 1;

				if (_GetPclPointCloud)
				{
					pclPoint.b = color.rgbBlue;
					pclPoint.g = color.rgbGreen;
					pclPoint.r = color.rgbRed;
				}

				colorVec->push_back(pointColor);
			}

			// Coordinate Mapping Depth to Camera Space
			CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
			m_pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, currDepth, &cameraSpacePoint);
			if ((0 <= colorX) && (colorX < nColorWidth) && (0 <= colorY) && (colorY < nColorHeight))
			{
				pointCoodr.x() = cameraSpacePoint.X;
				pointCoodr.y() = cameraSpacePoint.Y;
				pointCoodr.z() = cameraSpacePoint.Z;

				if (_GetPclPointCloud)
				{
					pclPoint.x = cameraSpacePoint.X;
					pclPoint.y = cameraSpacePoint.Y;
					pclPoint.z = cameraSpacePoint.Z;
				}

				coodrVec->push_back(pointCoodr);
			}
			// pcl 点云
			if (_GetPclPointCloud && pclPoint.x != 0 && pclPoint.y != 0 && pclPoint.z != 0)
			{
				modelPointcloud->push_back(pclPoint);
			}
		}
	}
	// 取消在获取时保存
	// 保存函数请单独见 saveCloudPoints函数
	//if (_GetPclPointCloud)
	//{
	//	printf("正在保存...\n");
	//	const std::string pcdfilename = "pcd_cloud" + std::to_string(pcdCount)+".pcd";
	//	pcl::io::savePCDFileASCII(pcdfilename, *modelPointcloud);
	//	std::cerr << "已保存 " << modelPointcloud->points.size() << "个数据至" << pcdfilename << "中" << std::endl;
	//	pcdCount++;
	//}

	// 叶节点
	osg::ref_ptr<osg::Geode> geode = new osg::Geode();

	// 用来存储几何数据信息 构造图像 保存了顶点数组数据的渲染指令
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();

	geom->setVertexArray(coodrVec.get());

	geom->setColorArray(colorVec.get());
	// 每一个颜色对应着一个顶点
	geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	// 指定数据绘制的方式
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, coodrVec->size()));
	// 加载到Geode中
	geode->addDrawable(geom.get());
	// 关闭光照
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	// 注释以加快速度
	//设置属性状态，并设置线宽为2个像素,点为4个像素。
	osg::ref_ptr<osg::StateSet> stateGeode = geode->getOrCreateStateSet();
	// 改变点的大小
	osg::ref_ptr<osg::Point> ptr = new osg::Point(2);
	
	stateGeode->setAttribute(ptr);
	//osg::ref_ptr<osg::LineWidth>lw = new osg::LineWidth(2.0);
	//stateGeode->setAttribute(lw);

	return geode;
}

int CMotionRecognition::SaveCloudPoints(bool _saveAsPCD, bool _saveAsTXT,std::string _filename,std::string _dir){
	const std::string pcdfilenameNoSuffix = _dir +"\\"+"pcd_cloud" + _filename;
	if (_saveAsPCD)
	{

		//pcl::io::savePCDFileASCII(pcdfilenameNoSuffix + ".pcd", *modelPointcloud);
		pcl::io::savePCDFileBinary(pcdfilenameNoSuffix + ".pcd", *modelPointcloud);

	}
	if (_saveAsTXT) 
	{
		ofstream file(pcdfilenameNoSuffix + ".txt", ios::out);
		if (!file.is_open())
		{
			return 1;
		}
		for (size_t i = 0; i < modelPointcloud->size(); i++)
		{
			pcl::PointXYZRGB & points = modelPointcloud->at(i);
			file 
				<< points.x << " " 
				<< points.y << " " 
				<< points.z << " " 
				<< (int)points.r << " " 
				<< (int)points.g << " "
				<< (int)points.b << endl;
		}
		file.close();
	}
	return 0;
}

osg::ref_ptr<osg::Node> CMotionRecognition::GetPointCloud() const{
	return model;
}