#pragma once
#include "Globaler.h"

// 显示而用,需检查是否有互锁可能
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER; 

// 开始显示当前帧信号量
pthread_cond_t startCond = PTHREAD_COND_INITIALIZER;   

// 刷新帧信号量
pthread_cond_t displayCond = PTHREAD_COND_INITIALIZER;

// 顶层节点
osg::ref_ptr<osg::Group> root = new osg::Group();

// 来自kinect的数据
osg::ref_ptr<osg::Node> kinectModel = new osg::Node();

// 实时显示
osg::ref_ptr<osg::Node> leftFrame = new osg::Node(); 

// 融合显示
osg::ref_ptr<osg::Node> rightFrame = new osg::Node(); 

// 三个场景的位置矩阵
osg::Matrix x = osg::Matrix::rotate(-180, 1, 0, 0)*osg::Matrix::rotate(-180, 0, 0, 1);

osg::ref_ptr<osg::MatrixTransform> leftMT = new osg::MatrixTransform();
osg::Matrix leftM = x*osg::Matrix::rotate(osg::inDegrees(90.0f), 1, 0, 0)*osg::Matrix::translate(-8, 0, 2);

osg::ref_ptr<osg::MatrixTransform> rightMT = new osg::MatrixTransform();
osg::Matrix rightM = x*osg::Matrix::rotate(osg::inDegrees(90.0f), 1, 0, 0)*osg::Matrix::translate(8, 0, 0);

// 关于显示窗体
osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer;
osg::observer_ptr<osgViewer::GraphicsWindow> window = new osgViewer::GraphicsWindow;
osg::ref_ptr<osg::Node> getfirstFrame(void);

bool flag_getCloudPoints = false;

size_t saveCount = 0;
// 创建窗体所需参数
struct argc_s{
	int argc;
	char ** argv;
};
static argc_s argcS;

pthread_t show_t;
pthread_t glut_t;

// 场景数据预处理
void preProcess(){

	leftMT->setMatrix(leftM);
	rightMT->setMatrix(rightM);

	leftMT->addChild(leftFrame);
	rightMT->addChild(rightFrame);

	root->addChild(leftMT);
	root->addChild(rightMT);
}

// 创建 mykinect线程
int createKinectThread(){
	pthread_t kinect_t;

	int kinect_t_flag;

	kinect_t_flag = pthread_create(&kinect_t, NULL, myKinect, NULL);

	if (kinect_t_flag != 0)
	{
		std::cout << "Kinect线程创建失败" << std::endl;
		return -1;
	}
	return 0;
}
 
// 创建showAgain线程
int createShowThread(){
	int show_t_flag;

	show_t_flag = pthread_create(&show_t, NULL, showAgain, NULL);

	if (show_t_flag != 0)
	{
		std::cout << "show线程创建失败" << std::endl;
		return -1;
	}
	return 0;
}

// 创建窗体线程
int createGLUTThread(int _argc,char **_argv){
	int glut_t_flag;
	
	argcS.argc = _argc;
	argcS.argv = _argv;

	glut_t_flag = pthread_create(&glut_t, NULL, myGlut,(void *)&argcS);

	if (glut_t_flag != 0)
	{
		std::cout << "glut线程创建失败" << std::endl;
		return -1;
	}
	return 0;
}

// 发送两种信号
// 1. 发给getfirstFrame 告知kinect已经具备获取图像的能力
// 2. 发给showAgain 告知已获取图像数据,并需要重绘
void * myKinect(void * argc){
	// 设置为可分离
	pthread_detach(pthread_self());

	std::cout <<"kinect线程打开"<< std::endl;

	CMotionRecognition motionRecognition;

	size_t frameCount = 0;

	// 初始化kinect
	HRESULT hr = motionRecognition.InitializeDefaultSensor();
	HRESULT hrUpdate = E_FAIL;
	while (!SUCCEEDED(hrUpdate))
	{
		hrUpdate = motionRecognition.update();
	}

	// 通知getfirstFrame() 已经获取到数据
	// 第一种信号
	pthread_mutex_lock(&mutex);

	kinectModel = motionRecognition.CalPointCloud(flag_getCloudPoints);
	
	pthread_mutex_unlock(&mutex);
	
	pthread_cond_signal(&startCond);

	// 进入循环获取
	while (true)
	{
		hr = motionRecognition.update();
		
		if (SUCCEEDED(hr))
		{
			pthread_mutex_lock(&mutex);
			
			kinectModel = motionRecognition.CalPointCloud(flag_getCloudPoints);

			if (flag_getCloudPoints)
			{
				rightMT->setChild(0, kinectModel);
				cout <<"正在保存点云数据..."<< endl;
				motionRecognition.SaveCloudPoints(true, false, std::to_string(++saveCount), "resultUpdate");
				cout << "保存完成" << endl;
				flag_getCloudPoints = false;
			}

			// 发送消息给showAgain 通过它启动glutPostRedisplay()绘图函数
			// 第二种信号
			pthread_cond_signal(&displayCond);
			
			pthread_mutex_unlock(&mutex);
		}
	}
	return NULL;
}

// 不断接受信号以调用重绘信号
void * showAgain(void * argc){
	// 不断接受信号以调用重绘信号
	while (true)
	{
		//std::cout << "等待接收重绘信号..." << std::endl;

		pthread_mutex_lock(&mutex);

		pthread_cond_wait(&displayCond, &mutex);

		// 下面是相关操作
		//std::cout << "已接收重绘信号" << std::endl;

		glutPostRedisplay();

		// 结束操作
		pthread_mutex_unlock(&mutex);
	}
}

// 为了创建窗体接受信号而返回显示的第一帧图像
osg::ref_ptr<osg::Node> getfirstFrame(void)
{
	std::cout << "正在等待kinect获取图像..." << std::endl;

	pthread_mutex_lock(&mutex);

	pthread_cond_wait(&startCond,&mutex);

	// 下面是相关操作
	std::cout << "获取成功..." << std::endl;

	// 结束操作
	pthread_mutex_unlock(&mutex);

	return kinectModel;
}

// 创建窗体程序
void * myGlut(void *argc){
	argc_s argcS = *(argc_s * )(argc);
	glutInit(&(argcS.argc), argcS.argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(800, 600);
	glRotatef(-180.0, 0.0, 0.0, 1.0);
	glutCreateWindow("GLUT");

	glutDisplayFunc	(display);	//注册重画的回调函数
	glutReshapeFunc	(reshape);	//注册窗口大小改变的回调函数
	glutMouseFunc	(mousebutton);	//注册鼠标按键回调函数
	glutMotionFunc	(mousemove);	//注册鼠标移动回调函数
	glutKeyboardFunc(keyboard);

	leftMT->setChild(0, getfirstFrame());
	rightMT->setChild(0,kinectModel);

	// create the view of the scene.
	window = viewer->setUpViewerAsEmbeddedInWindow(100, 100, 800, 600);
	viewer->setSceneData(root.get());
	viewer->setCameraManipulator(new osgGA::TrackballManipulator);
	viewer->addEventHandler(new osgViewer::StatsHandler);
	viewer->realize();

	glutMainLoop();
	return NULL;
}

// 窗体响应回调函数
void display(void)
{
	leftMT->setChild(0, kinectModel);

	// update and render the scene graph
	if (viewer.valid())
	{
		viewer->frame();
	}

	// Swap Buffers
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	// update the window dimensions, in case the window has been resized.
	if (window.valid())
	{
		window->resized(window->getTraits()->x, window->getTraits()->y, w, h);
		window->getEventQueue()->windowResize(window->getTraits()->x, window->getTraits()->y, w, h);
	}
}

void mousebutton(int button, int state, int x, int y)
{
	if (window.valid())
	{
		if (state == 0) window->getEventQueue()->mouseButtonPress(x, y, button + 1);
		else window->getEventQueue()->mouseButtonRelease(x, y, button + 1);
	}
}

void mousemove(int x, int y)
{
	if (window.valid())
	{
		window->getEventQueue()->mouseMotion(x, y);
	}
}

void keyboard(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		// clean up the viewer 
		if (viewer.valid()) viewer = 0;
		glutDestroyWindow(glutGetWindow());
		break;
	// V为捕捉当前帧
	case 'V':
		startUpSaveAsFile();
		break;
	default:
		if (window.valid())
		{
			window->getEventQueue()->keyPress((osgGA::GUIEventAdapter::KeySymbol) key);
			window->getEventQueue()->keyRelease((osgGA::GUIEventAdapter::KeySymbol) key);
		}
		break;
	}
}

int destroyPthread(){
	// 线程相关销毁操作
	// 各个变量都需要一个销毁函数

	pthread_join(glut_t,NULL);
	pthread_join(show_t,NULL);
	
	pthread_mutex_destroy(&mutex);
	pthread_cond_destroy(&displayCond);
	pthread_cond_destroy(&startCond);
	return 0;
}

void startUpSaveAsFile(){
	pthread_mutex_lock(&mutex);
	flag_getCloudPoints = true;
	pthread_mutex_unlock(&mutex);
}

int pcbConvertintoOsg(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcbResult, osg::ref_ptr<osg::Node> &_frame){
	// 一帧空间坐标
	osg::ref_ptr<osg::Vec3Array> coodrVec = new osg::Vec3Array();
	// 一帧颜色值
	osg::ref_ptr<osg::Vec4Array> colorVec = new osg::Vec4Array();
	for (size_t i = 0; i < _pcbResult->size(); i++)
	{
		coodrVec->push_back(osg::Vec3f(_pcbResult->at(i).x, _pcbResult->at(i).y, _pcbResult->at(i).z));
		colorVec->push_back(osg::Vec4f((float)_pcbResult->at(i).r / 255, (float)_pcbResult->at(i).g / 255, (float)_pcbResult->at(i).b / 255, 1));
	}
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
	osg::ref_ptr<osg::Point> ptr = new osg::Point(4);
	stateGeode->setAttribute(ptr);

	_frame = geode;
	return 0;
}

