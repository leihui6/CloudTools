#pragma once
#include "Header.h"
#include "MotionRecognition.h"

// 场景预加载
void preProcess();

// 创建线程
int createKinectThread();
int createShowThread();
int createGLUTThread(int _argc,char **argv);

// 被创建线程
void * myKinect(void * argc);
void * myGlut(void *argc);
void * showAgain(void * argc);

// 销毁线程数据
int destroyPthread();

// 窗体回调函数
void display(void);
void reshape(int w, int h);
void mousebutton(int button, int state, int x, int y);
void mousemove(int x, int y);
void keyboard(unsigned char key, int /*x*/, int /*y*/);

// 相应键盘事件
// 保存数据
void startUpSaveAsFile();