/*!
* @file main.h
*  @brief     Header file for main.cpp.
*  @author    Florian J.T. Wachter
*  @version   1.0
*  @date      April, 8th 2017
*/

#pragma once
#include <fstream>
#include <windows.h>
#include <conio.h>
#include <tchar.h>
#include <stdio.h>
#include <iostream>
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>
#include <gl/glu.h>
#include <sstream>
#include <thread>
#include <future>
#include <atomic>

#include "communication.h"

#include "RigidTrack.h"
#include <QtWidgets/QApplication>
#include <QUdpSocket>
#include "cameralibrary.h"     //!/<== Camera Library header file ======================---
#include "modulevector.h"
#include "modulevectorprocessing.h"
#include "coremath.h"

//!/<==-- OpenCV stuff import
#include <opencv\cv.h>
#include "opencv2\core.hpp"
#include "opencv2\calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\video\tracking.hpp>

using namespace CameraLibrary;
using namespace cv;

extern int methodPNP;
extern bool safetyEnable;
extern bool safety2Enable;
extern double safetyBoxLength; 
extern int safetyAngle; 
extern QHostAddress IPAdressObject;
extern QHostAddress IPAdressSafety;
extern QHostAddress IPAdressSafety2;
extern int portObject; 
extern int portSafety;
extern int portSafety2;
extern int invertZ;
extern commObject commObj;

int startTracking();
void startStopCamera();
int setReference();
int calibrateCamera();
void loadCalibration(int method);
void testAlgorithms();
void projectCoordinateFrame(Mat pictureFrame);
void setUpUDP();
void setHeadingOffset(double d);
void sendDataUDP(cv::Vec3d &Position, cv::Vec3d &Euler);
void closeUDP();
void loadMarkerConfig(int method);
void drawPositionText(cv::Mat &Picture, cv::Vec3d &Position, cv::Vec3d &Euler, double error);
void loadCameraPosition();
int determineExposure();
void determineOrder();
int calibrateGround();