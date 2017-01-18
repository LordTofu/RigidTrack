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

#include "RigidTrack.h"
#include <QtWidgets/QApplication>
#include <QUdpSocket>
#include "cameralibrary.h"     //== Camera Library header file ======================---
#include "modulevector.h"
#include "modulevectorprocessing.h"
#include "coremath.h"

//==-- OpenCV stuff import
#include <opencv\cv.h>
#include "opencv2\core.hpp"
#include "opencv2\calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\video\tracking.hpp>

using namespace CameraLibrary;
using namespace cv;

	int start_camera();
	int setZero();
	int calibrate_camera();
	void load_calibration();
	void test_Algorithm();
	void setupKalmanFilter();
	void projectCoordinateFrame(Mat pictureFrame);
	void setUpUDP();
	void setUpMMF();
	void sendDataUDP(double &velocityz, cv::Vec3d &Euler);