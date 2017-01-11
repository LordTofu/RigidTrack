#include "RigidTrack.h"
#include <QtWidgets/QApplication>
#include <QUdpSocket>
#include "cameralibrary.h"     //== Camera Library header file ======================---
#include "modulevector.h"
#include "modulevectorprocessing.h"
#include "coremath.h"
#include "supportcode.h"

//==-- OpenCV stuff import
#include <opencv\cv.h>
#include "opencv2\core.hpp"
#include "opencv2\calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\video\tracking.hpp>

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
#include <time.h>
#include <cmath>
#include <vector>
#include <algorithm>

#include "main.h"
#include "communication.h"

using namespace CameraLibrary;
using namespace cv;

commObject commObj;

double frameTime = 1 / 100;
double timeOld = 0.0;

Vec3d position = Vec3d();
Vec3d eulerAngles = Vec3d();
Vec3d positionOld = Vec3d();
Vec3d velocity = Vec3d();
Vec3d scaleVec = Vec3d();
Vec3d posRef = Vec3d();
Vec3d eulerRef = Vec3d();

double distFromRef = 0;
double latitudeRef = 47;
double longitudeRef = 11;
double heightRef = 0;
double latitude = 47;
double longitude = 11;
double height = 0;
double earthRadius = 6366743.0; // Radius of the Earth at 47° North in Meters
std::ofstream logfile;


int intIntensity = 6; // max Intensity is 15 1-6 is strobe 7-15 is continuous 13 and 14 are meaningless 
int intExposure = 60; // max is 60 increase if markers are badly visible
int intFrameRate = 100;

//== Rotation, translation etc. matrix for PnP results
Mat Rmat = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	//Rotation Matrix from Marker CoSy to Camera
Mat RmatRef = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	//Reference Rotation Matrix from Marker CoSy to Camera
Mat M_NC = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	// Rotation Matrix from Camera to Ground
Mat Rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	//Rotation Vector (Axis-Angle) from Marker CoSy to Camera
Mat Tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	//Translation Vector from Marker CoSy to Camera
Mat RvecOriginal;
Mat TvecOriginal;

float Value[100] = { 0 };	//100 Values can be sent via MMF, can be more but should be enough for now

RotatedRect rotRect;
double majorAxis = 0;
double heading = 0;
Vec4d positionHeight = 0;
double cameraAbove = 4300.0; // camera is 4300mm above marker system
double circleRadius = 545.0 / 2.0;  // Radius of the markers in mm
double picturePlanedistance = 0; // doesnt change, is computed afterwards
double FoV = 57.5; // FoV of Camera in degrees, constant

bool useGuess = true; // not used
int methodPNP = 0; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used

int numberMarkers = 4; // number of markers, the more the better. 5 is minimum, 6 minimum for position and heading
//== Marker points in real world coordinates and in camera pixel coordinates	==--
std::vector<Point3d> list_points3d(4);
std::vector<Point2d> list_points2d(4);
std::vector<Point2d> list_points2dOld(4);
std::vector<double> list_points2dDifference(4);
std::vector<Point2d> list_points2dProjected(4);
std::vector<Point2d> list_points2dUnsorted(4);
std::vector<Point3d> coordinateFrame(4);
std::vector<Point2d> coordinateFrameProjected(4);
int pointOrderIndices[] = { 0, 1, 2, 3 };
int pointOrderIndicesNew[] = { 0, 1, 2, 3 };
double currentPointDistance = 5000;
double minPointDistance = 5000;
int currentMinIndex = 0;
bool gotOrder = false;
int xFactor = 1;

bool enableKalman = false; 
KalmanFilter KF(6, 3, 0);
Mat_<float> measurement(3, 1);

Mat cameraMatrix;
Mat distCoeffs;
Core::DistortionModel distModel;

// IP adress of the circuit breaker that disables the drone if a specified region is exited. 
QUdpSocket *udpSocketCB;
QUdpSocket *udpSocketDrone;

const int BACKBUFFER_BITSPERPIXEL = 8;
std::string strBuf;
std::stringstream ss;
QByteArray data;


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	RigidTrack w;
	w.show();
	QObject::connect(&commObj, SIGNAL(statusChanged(QString)), &w, SLOT(setStatus(QString)), Qt::DirectConnection);
	QObject::connect(&commObj, SIGNAL(imageChanged(QPixmap)), &w, SLOT(setImage(QPixmap)), Qt::DirectConnection);
	QObject::connect(&commObj, SIGNAL(logAdded(QString)), &w, SLOT(setLog(QString)), Qt::DirectConnection);

	//list_points3d[0] = cv::Point3d( 203.0,    0.0,  0.0);
	//list_points3d[1] = cv::Point3d(   0.0, -309.0,  0.0);
	//list_points3d[2] = cv::Point3d(-319.0,    0.0,  0.0);
	//list_points3d[3] = cv::Point3d(-470.0,  158.0,  0.0);

	// Coordinates of the markers in plane reference or arbitrary other frame
	list_points3d[0] = cv::Point3d(  -25.0, -1000.0, 0.0);
	list_points3d[1] = cv::Point3d( -138.0,  +390.0, 0.0);
	list_points3d[2] = cv::Point3d( -400.0,     0.0, 0.0);
	list_points3d[3] = cv::Point3d(-1000.0,     0.0, 0.0);

	// Cardboard Frame Prototype
	//list_points3d[1] = cv::Point3d(0.0, 160.0, 0.0);
	//list_points3d[2] = cv::Point3d(200.0, 100.0, 0.0);
	//list_points3d[0] = cv::Point3d(100.0, 70.0, 0.0);
	//list_points3d[3] = cv::Point3d(50.0, 70.0, 0.0);

	multiply(list_points3d, 1., list_points3d);

	//Initial Guesses, important for Iterative Method!
	Tvec.at<double>(0) = 0;   
	Tvec.at<double>(1) = 0;
	Tvec.at<double>(2) = 5000;
	Rvec.at<double>(0) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(1) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(2) = -90*3.141592653589/180.0;
	
	coordinateFrame[0] = cv::Point3d(0, 0, 0);
	coordinateFrame[1] = cv::Point3d(300, 0, 0);
	coordinateFrame[2] = cv::Point3d(0, 300, 0);
	coordinateFrame[3] = cv::Point3d(0, 0, 300);

	test_Algorithm();
	return a.exec();
}

QPixmap Mat2QPixmap(cv::Mat src)
{
	QImage dest((const uchar *)src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
	dest.bits(); // enforce deep copy, see documentation 
				 // of QImage::QImage ( const uchar * data, int width, int height, Format format )
	QPixmap pixmapDest = QPixmap::fromImage(dest);
	return pixmapDest;
}

void calcBoardCornerPositions(Size boardSize, float squareSize, std::vector<Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
}

void getEulerAngles(Mat &rotCamerMatrix, Vec3d &eulerAngles) {

	Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
	double* _r = rotCamerMatrix.ptr<double>();
	double projMatrix[12] = { _r[0],_r[1],_r[2],0,
		_r[3],_r[4],_r[5],0,
		_r[6],_r[7],_r[8],0 };

	decomposeProjectionMatrix(Mat(3, 4, CV_64FC1, projMatrix),
		cameraMatrix,
		rotMatrix,
		transVect,
		rotMatrixX,
		rotMatrixY,
		rotMatrixZ,
		eulerAngles);
}

int start_camera() {

	// Creating UDP slot
	commObj.addLog("Opening UDP Port");
	udpSocketCB = new QUdpSocket(0);
	udpSocketDrone = new QUdpSocket(0);

	QHostAddress bcast = QHostAddress("192.168.4.1");
	udpSocketCB->connectToHost(bcast, 5000);

	bcast = QHostAddress("192.168.4.2");
	udpSocketDrone->connectToHost(bcast, 5000);

	commObj.addLog("Opened UDP Port");

	//=0 Set Stepper Motor Position to 0
	data.setNum((int)(0));
	udpSocketCB->write(data);

	commObj.changeStatus("Creating MMF");

	HANDLE hMapFile;
	LPCTSTR pBuf;

	TCHAR szMsg[] = TEXT("MMF Text");

	hMapFile = CreateFileMapping(
		INVALID_HANDLE_VALUE,    // use paging file
		NULL,                    // default security
		PAGE_READWRITE,          // read/write access
		0,                       // maximum object size (high-order DWORD)
		256,                // maximum object size (low-order DWORD)
		L"SIMULINK_MMF");                 // name of mapping object

	if (hMapFile == NULL)
	{
		_tprintf("Could not create file mapping object (%d).\n",
			GetLastError());
		return 1;
	}

	pBuf = (LPTSTR)MapViewOfFile(hMapFile,   // handle to map object
		FILE_MAP_ALL_ACCESS, // read/write permission
		0,
		0,
		256);

	if (pBuf == NULL)
	{
		_tprintf("Could not map view of file (%d).\n",
			GetLastError());
		CloseHandle(hMapFile);
		return 1;
	}

	commObj.changeStatus("MMF Created");

	CopyMemory((PVOID)pBuf, &Value, 100 * sizeof(float));
	//== For OptiTrack Ethernet cameras, it's important to enable development mode if you
	//== want to stop execution for an extended time while debugging without disconnecting
	//== the Ethernet devices.  Lets do that now:

	CameraLibrary_EnableDevelopment();

	//== Initialize Camera SDK ==--
	CameraLibrary::CameraManager::X();

	//== At this point the Camera SDK is actively looking for all connected cameras and will initialize
	//== them on it's own.

	//== Get a connected camera ================----
	CameraManager::X().WaitForInitialization();
	Camera *camera = CameraManager::X().GetCamera();

	//== If no device connected, pop a message box and exit ==--
	if (camera == 0)
	{
		commObj.addLog("No Camera found!");
		return 1;
	}

	//== Determine camera resolution to size application window ==----
	int cameraWidth = camera->Width();
	int cameraHeight = camera->Height();

	//== Set Video Mode ==--

	//== We set the camera to Segment Mode here.  This mode is support by all of our products.
	//== Depending on what device you have connected you might want to consider a different
	//== video mode to achieve the best possible tracking quality.  All devices that support a
	//== mode that will achieve a better quality output with a mode other than Segment Mode are
	//== listed here along with what mode you should use if you're looking for the best head
	//== tracking:
	//==
	//==     V100:R1/R2    Precision Mode
	//==     TrackIR 5     Bit-Packed Precision Mode
	//==     V120          Precision Mode
	//==     TBar          Precision Mode
	//==     S250e         Precision Mode
	//==
	//== If you have questions about a new device that might be conspicuously missing here or
	//== have any questions about head tracking, email support or participate in our forums.

	camera->SetVideoType(Core::PrecisionMode);

	//== Start camera output ==--
	camera->Start();

	//== Turn on some overlay text so it's clear things are     ===---
	//== working even if there is nothing in the camera's view. ===---
	camera->SetTextOverlay(true);

	camera->SetExposure(intExposure);
	camera->SetIntensity(intIntensity);
	camera->SetFrameRate(intFrameRate);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(true);

	//== Fetch a new frame from the camera ===---
	cv::Mat matFrame = Mat::zeros(cv::Size(cameraWidth, cameraHeight), CV_8UC1);
	QPixmap QPFrame;

	//==-- Kalman Filter creation and init values
	if (enableKalman)
	{
		setupKalmanFilter();
	}

	//Helper Variables used to print ouput only every 30th time and cick Circuit Breaker
	int u = 0;
	int v = 0;

	//Enable Circuit Breaker
	data.setNum((int)(9));
	udpSocketCB->write(data);
	data.setNum((int)(1));
	udpSocketCB->write(data);

	while (1)
	{
		//== Fetch a new frame from the camera ===---
		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.
			if (frame->ObjectCount() == numberMarkers)
			{
				for (int i = 0; i < frame->ObjectCount(); i++)
				{
					cObject *obj = frame->Object(i);
					list_points2dUnsorted[i] = cv::Point2d(obj->X(), obj->Y());
				}

				// Now its time to determine the order of the points
				// for that the distance from the new points to the old points is calculated
				// for each point the new index corresponds to the old point with the smallest distance
				// Loop over every point and calculate the min distance to every other point. 
				// Then pick the smallest one and assign its index to the new order pointOrderIndices.
				for (int j = 0; j < numberMarkers; j++)
				{
					minPointDistance = 5000;
					for (int k = 0; k < numberMarkers; k++)
					{
						currentPointDistance = norm(list_points2dUnsorted[pointOrderIndices[j]] - list_points2dOld[k]);
						if (currentPointDistance < minPointDistance)
						{
							minPointDistance = currentPointDistance;
							pointOrderIndicesNew[j] = k;
						}
					}
				}
				for (int w = 0; w < numberMarkers; w++)
				{
					pointOrderIndices[w] = pointOrderIndicesNew[w];
				}
				
				list_points2dOld = list_points2dUnsorted;

				list_points2d[0] = list_points2dUnsorted[pointOrderIndices[0]];
				list_points2d[1] = list_points2dUnsorted[pointOrderIndices[1]];
				list_points2d[2] = list_points2dUnsorted[pointOrderIndices[2]];
				list_points2d[3] = list_points2dUnsorted[pointOrderIndices[3]];

				//Compute the pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
				
				double maxValue = 0;
				double minValue = 0;
				minMaxLoc(Tvec, &minValue, &maxValue);
				if (maxValue < 10000 && minValue > -10000)
				{
					Mat _Rmat;	
					Rodrigues(Rvec, _Rmat);
					Mat V = RmatRef *_Rmat.t() * (Mat)Tvec; // translation in floor system 
					position = V;
					subtract(position, posRef, position);
					
					// Realtive angle between reference orientation and current orientation
					Rodrigues(Rvec, Rmat);
					Rmat = RmatRef* Rmat.t();
					//==-- Euler Angles, finally 
					getEulerAngles(Rmat, eulerAngles);
				
					if (enableKalman) {
						Mat prediction = KF.predict();
						measurement(0) = (float)position[0];
						measurement(1) = (float)position[1];
						measurement(2) = (float)position[2];
						Mat estimation = KF.correct(measurement);
						position[0] = (double)estimation.at<float>(0);
						position[1] = (double)estimation.at<float>(1);
						position[2] = (double)estimation.at<float>(2);
					}

					frameTime = frame->TimeStamp() - timeOld;
					timeOld = frame->TimeStamp();
					velocity[0] = (position[0] - positionOld[0]) / frameTime;
					velocity[1] = (position[1] - positionOld[1]) / frameTime;
					velocity[2] = (position[2] - positionOld[2]) / frameTime;
					positionOld = position;

					latitude = latitudeRef + atan(Value[0] / earthRadius);
					longitude = longitudeRef + atan(Value[1] / earthRadius);

					Value[0] = position[0] / 1000.;
					Value[1] = position[1] / 1000.;
					Value[2] = position[2] / 1000.;
					Value[3] = eulerAngles[0];
					Value[4] = eulerAngles[1];
					Value[5] = eulerAngles[2];

					CopyMemory((PVOID)pBuf, &Value, 100 * sizeof(double));
					distFromRef = norm(position);

					//u++;  
					//v++;
					if (u == 100) {
						ss.str("");
						ss << Tvec << "\n";
						ss << Rmat << "\n";
						commObj.addLog(QString::fromStdString(ss.str()));
						u = 0;
					}

					// send enable signal to Circuit Breaker if everything is fine and drone is within allowed area
					if(false)
					{
					if (distFromRef < 1000 || eulerAngles[1] < 15 || eulerAngles[2] < 15) {
						if (v == 3) {
							data.setNum((int)(1));
							udpSocketCB->write(data);
							v = 0;
						}
					}
					else
					{
						data.setNum((int)(0));
						udpSocketCB->write(data);
						ss.str("");
						ss << "Drone left allowed Area, shutting it down!" << "\n";
						commObj.addLog(QString::fromStdString(ss.str()));
					}
					}

					if (u == 100) {
						ss.str("");
						ss << "X      =  " << position[0] << "\tY    =  " << position[1] << "\tZ     = " << position[2] << "\n";
						ss << "VX     =  " << velocity[0] << "\tVY   =  " << velocity[1] << "\tVZ    = " << velocity[2] << "\n";
						ss << "pitch  =  " << eulerAngles[0] << "\tyaw  =  " << eulerAngles[1] << "\troll  = " << eulerAngles[2];
						commObj.addLog(QString::fromStdString(ss.str()));
						u = 0;
					}

					logfile.open("logData.txt", std::ios::app);
					logfile << frame->TimeStamp() << ";" << position[0] << ";" << position[1] << ";" << position[2] << ";";
					logfile << eulerAngles[0] << ";" << eulerAngles[1] << ";" << eulerAngles[2] << ";";
					logfile << velocity[0] << ";" << velocity[1] << ";" << velocity[2] << "\n";
					logfile.close();
				}
			}

			frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);

			Mat cFrame(480, 640, CV_8UC3, Scalar(0, 0, 0));
			cvtColor(matFrame, cFrame, COLOR_GRAY2RGB);

			circle(cFrame, Point(list_points2dUnsorted[0].x, list_points2dUnsorted[0].y), 6, Scalar(0, 0, 255), 3);
			circle(cFrame, Point(list_points2dUnsorted[1].x, list_points2dUnsorted[1].y), 12, Scalar(0, 0, 255), 3);
			circle(cFrame, Point(list_points2dUnsorted[2].x, list_points2dUnsorted[2].y), 18, Scalar(0, 0, 255), 3);
			circle(cFrame, Point(list_points2dUnsorted[3].x, list_points2dUnsorted[3].y), 24, Scalar(0, 0, 255), 3);
			Tvec.at<double>(0) = -Tvec.at<double>(0);
			projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2d);
			for (int i = 0; i < numberMarkers; i++)
			{
				circle(cFrame, Point(list_points2d[i].x, list_points2d[i].y), 3, Scalar(225, 0, 0), 3);
			}
			QPixmap QPFrame;
			QPFrame = Mat2QPixmap(cFrame);
			commObj.changeImage(QPFrame);
			QCoreApplication::processEvents();
			frame->Release();
		}
	}

	//== Release camera ==--
	camera->Release();
	UnmapViewOfFile(pBuf);
	CloseHandle(hMapFile);

	//== Shutdown Camera Library ==--
	CameraManager::X().Shutdown();

	//== Exit the application.  Simple! ==--
	return 0;
}

int setZero()
{
	posRef = 0;
	eulerRef = 0;
	RmatRef = 0;
	ss.str("");
	ss << "Started Reference Coordinate Determination";
	commObj.addLog(QString::fromStdString(ss.str()));

	CameraLibrary_EnableDevelopment();
	//== Initialize Camera SDK ==--
	CameraLibrary::CameraManager::X();

	//== At this point the Camera SDK is actively looking for all connected cameras and will initialize
	//== them on it's own.

	//== Get a connected camera ================----
	CameraManager::X().WaitForInitialization();
	Camera *camera = CameraManager::X().GetCamera();

	//== If no device connected, pop a message box and exit ==--
	if (camera == 0)
	{
		commObj.addLog("No Camera found!");
		return 1;
	}

	//== Determine camera resolution to size application window ==----
	int cameraWidth = camera->Width();
	int cameraHeight = camera->Height();
	camera->GetDistortionModel(distModel);
	cv::Mat matFrame(cv::Size(cameraWidth, cameraHeight), CV_8UC1);

	// Set camera mode to precision mode, it directly provides marker coordinates
	camera->SetVideoType(Core::PrecisionMode);

	//== Start camera output ==--
	camera->Start();

	//== Turn on some overlay text so it's clear things are     ===---
	//== working even if there is nothing in the camera's view. ===---

	camera->SetTextOverlay(true);
	camera->SetExposure(intExposure);
	camera->SetFrameRate(intFrameRate);
	camera->SetIntensity(intIntensity);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	int numberSamples = 0;
	int numberToSample = 200;

	//set exposure such that num markers are visible

	int numberObjects = 0;
	while (numberObjects != numberMarkers)
	{
		Frame *frame = camera->GetFrame();
		if (frame)
		{
			numberObjects = frame->ObjectCount();
			if (numberObjects > numberMarkers) { intExposure--; }
			if (numberObjects < numberMarkers) { intExposure++; }
			camera->SetExposure(intExposure);
			ss.str("");
			ss << "Exposure		  =  " << intExposure << "\n";
			ss << "Objects found  =  " << numberObjects << "\n";
			commObj.addLog(QString::fromStdString(ss.str()));
		}
	}

	while (numberSamples < numberToSample)
	{
		//== Fetch a new frame from the camera ===---
		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.
			if (frame->ObjectCount() == numberMarkers)
			{
				//==for(int i=0; i<frame->ObjectCount(); i++)
				for (int i = 0; i < numberMarkers; i++)
				{
					cObject *obj = frame->Object(i);
					list_points2dUnsorted[i] = cv::Point2d(obj->X(), obj->Y());
				}

				
				if(gotOrder == false)
				{
				minPointDistance = 5000;
				std::sort(pointOrderIndices, pointOrderIndices + 4);
				do {
					list_points2d[0] = list_points2dUnsorted[pointOrderIndices[0]];
					list_points2d[1] = list_points2dUnsorted[pointOrderIndices[1]];
					list_points2d[2] = list_points2dUnsorted[pointOrderIndices[2]];
					list_points2d[3] = list_points2dUnsorted[pointOrderIndices[3]];
					solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
					double maxValue = 0;
					double minValue = 0;
					minMaxLoc(Tvec, &minValue, &maxValue);
					if (maxValue < 10000 && minValue > -10000)
					{
						projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
						for (int w = 0; w < numberMarkers; w++)
						{
							currentPointDistance += norm(list_points2d[w] - list_points2dProjected[w]);
						}

						if (currentPointDistance < minPointDistance)
						{
							minPointDistance = currentPointDistance;
							for (int w = 0; w < numberMarkers; w++)
							{
								pointOrderIndicesNew[w] = pointOrderIndices[w];
							}
						}
					}
				} while (std::next_permutation(pointOrderIndices, pointOrderIndices + 4));

				for (int w = 0; w < numberMarkers; w++)
				{
					pointOrderIndices[w] = pointOrderIndicesNew[w];
				}
				gotOrder = true;
				}

				//Order of Points does matter!!!!
				list_points2d[0] = list_points2dUnsorted[pointOrderIndices[0]];
				list_points2d[1] = list_points2dUnsorted[pointOrderIndices[1]];
				list_points2d[2] = list_points2dUnsorted[pointOrderIndices[2]];
				list_points2d[3] = list_points2dUnsorted[pointOrderIndices[3]];

				list_points2dOld = list_points2dUnsorted;

				//Compute the pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
				
				double maxValue = 0;
				double minValue = 0;
				minMaxLoc(Tvec, &minValue, &maxValue);
				if (maxValue < 10000 && minValue > -10000)
				{
					if (norm(positionOld) - norm(Tvec) < 1)	//Iterative Method needs time to converge to solution
					{
						add(posRef, Tvec, posRef);
						add(eulerRef, Rvec, eulerRef); // That are not the values of yaw, roll and pitch yet! Rodriguez has to be called first. 
						numberSamples += 1;	//==-- one sample more :D
						ss.str("");
						ss << "Tvec = " << Tvec << "\n";
						commObj.addLog(QString::fromStdString(ss.str()));
					}
					
					positionOld = Tvec;

					Mat cFrame(480, 640, CV_8UC3, Scalar(0, 0, 0));
					for (int i = 0; i < numberMarkers; i++)
					{
						circle(cFrame, Point(list_points2d[i].x, list_points2d[i].y), 6, Scalar(0, 225, 0), 3);
					}
					projectCoordinateFrame(cFrame);
					projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2d);
					for (int i = 0; i < numberMarkers; i++)
					{
						circle(cFrame, Point(list_points2d[i].x, list_points2d[i].y), 3, Scalar(225, 0, 0), 3);
					}
					for (int i = 0; i < numberMarkers; i++)
					{
						//circle(cFrame, Point(list_points2dprojected[i].x, list_points2dprojected[i].y), 3, Scalar(0, 0, 255), );
					}
					QPixmap QPFrame;
					QPFrame = Mat2QPixmap(cFrame);
					commObj.changeImage(QPFrame);
					QCoreApplication::processEvents();
				}
			}
			frame->Release();
		}
	}
	//== Release camera ==--
	camera->Release();

	//Divide by the number of samples to get the mean of the reference position
	divide(posRef, numberToSample, posRef);
	divide(eulerRef, numberToSample, eulerRef); // eulerRef is here in Axis Angle notation

	Rodrigues(eulerRef, RmatRef);				// axis angle to rotation matrix
	//==-- Euler Angles, finally 
	getEulerAngles(RmatRef, eulerRef);	//  rotation matrix to euler
	ss.str("");
	ss << "================= RmatRef ==================\n";
	ss << RmatRef << "\n";
	ss.str("");
	ss << "================= Reference Position ==================\n";
	ss << posRef << "\n";
	ss << "=============== Reference Euler Angles ================\n";
	ss << eulerRef << "\n";
	commObj.addLog(QString::fromStdString(ss.str()));
	return 0;
}

int calibrate_camera()
{
	CameraLibrary_EnableDevelopment();

	//== Initialize Camera SDK ==--
	CameraLibrary::CameraManager::X();

	//== At this point the Camera SDK is actively looking for all connected cameras and will initialize
	//== them on it's own.

	//== Get a connected camera ================----
	CameraManager::X().WaitForInitialization();

	Camera *camera = CameraManager::X().GetCamera();
	if (camera == 0)
	{
		commObj.addLog("No Camera found!");
		return 1;
	}

	//== Determine camera resolution
	int cameraWidth = camera->Width();
	int cameraHeight = camera->Height();

	//== Set Video Mode ==--

	//== We set the camera to Segment Mode here.  This mode is support by all of our products.
	//== Depending on what device you have connected you might want to consider a different
	//== video mode to achieve the best possible tracking quality.  All devices that support a
	//== mode that will achieve a better quality output with a mode other than Segment Mode are
	//== listed here along with what mode you should use if you're looking for the best head
	//== tracking:
	//==
	//==     V100:R1/R2    Precision Mode
	//==     TrackIR 5     Bit-Packed Precision Mode
	//==     V120          Precision Mode
	//==     TBar          Precision Mode
	//==     S250e         Precision Mode
	//==
	//== If you have questions about a new device that might be conspicuously missing here or
	//== have any questions about head tracking, email support or participate in our forums.

	camera->SetVideoType(Core::GrayscaleMode);

	//== Start camera output ==--
	camera->Start();

	//== Camera Matrix creation	==--
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	distCoeffs = Mat::zeros(8, 1, CV_64F);

	//== Ok, start main loop.  This loop fetches and displays   ===---
	//== camera frames.                                         ===---
	// But first set some camera parameters
	camera->SetAGC(true);
	camera->SetAEC(true);
	camera->SetExposure(30);
	camera->SetIntensity(4);
	camera->SetFrameRate(30);
	camera->SetIRFilter(false);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	int number_samples = 0;
	int imagesToSample = 80;

	std::vector<std::vector<Point2f> > imagePoints;
	std::vector<Point2f> pointBuf;
	bool found;
	Size boardSize(9, 6);
	Size imageSize(cameraWidth, cameraHeight);
	Mat Rvec(3, 1, DataType<double>::type);
	Mat Tvec(3, 1, DataType<double>::type);
	float squareSize = 23;

	QPixmap QPFrame;

	while (number_samples < imagesToSample)
	{
		//== Fetch a new frame from the camera ===---
		cv::Mat matFrame(cv::Size(cameraWidth, cameraHeight), CV_8UC1);

		// which is why we also set this constant to 8 
		const int BACKBUFFER_BITSPERPIXEL = 8;

		// later on, when we get the frame as usual:
		CameraLibrary::Frame * frame = camera->GetFrame();

		if (frame)
		{
			//== Lets have the Camera Library raster the camera's
			//== image into our texture.

			frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);
			//imwrite("test.jpg" + , matFrame);
			QPFrame = Mat2QPixmap(matFrame);
			commObj.changeImage(QPFrame);
			found = findChessboardCorners(matFrame, boardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			if (found)                // If done with success,
			{
				// improve the found corners' coordinate accuracy for chessboard
				cornerSubPix(matFrame, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

				imagePoints.push_back(pointBuf);
				number_samples += 1;
				commObj.addLog(QString::fromStdString(ss.str()));
				QCoreApplication::processEvents();
			}
			frame->Release();
			ss.str("");
			ss << "Samples found  =  " << number_samples;
		}
		Sleep(2);
	}

	std::vector<std::vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]);
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, Rvec, Tvec);

	//== Release camera ==--
	camera->Release();

	//== Shutdown Camera Library ==--
	FileStorage fs("calibration.xml", FileStorage::WRITE);//+ FileStorage::MEMORY);
	fs << "CameraMatrix" << cameraMatrix;
	fs << "DistCoeff" << distCoeffs;
	fs << "RMS" << rms;
	strBuf = fs.releaseAndGetString();
	commObj.changeStatus(QString::fromStdString(strBuf));
	commObj.addLog("Saved Calibration!");
	return 0;
}

void load_calibration() {
	FileStorage fs;
	fs.open("calibration.xml", FileStorage::READ);
	//fs.open("calibrationOptitrack.xml", FileStorage::READ);
	fs["CameraMatrix"] >> cameraMatrix;
	fs["DistCoeff"] >> distCoeffs;
	commObj.addLog("Loaded Calibration!");
	ss.str("");
	ss << "Camera Matrix\n" << "\n" << cameraMatrix << "\n";
	ss << "Distortion Coeff\n" << "\n" << distCoeffs << "\n";
	commObj.changeStatus(QString::fromStdString(strBuf));
	commObj.addLog(QString::fromStdString(ss.str()));
}

void test_Algorithm()
{
	int _methodPNP;
	load_calibration();

	std::vector<Point2d> noise(numberMarkers);

	RvecOriginal = Rvec;
	TvecOriginal = Tvec;
	
	multiply(list_points3d, 1., list_points3d);

	projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
	
	ss.str("");
	ss << "Unsorted Points 2D Projected \n";
	ss << list_points2dProjected << "\n";
	commObj.addLog(QString::fromStdString(ss.str()));

	Mat cFrame(480, 640, CV_8UC3, Scalar(0, 0, 0));
	for (int i = 0; i < numberMarkers; i++)
	{
		circle(cFrame, Point(list_points2dProjected[i].x, list_points2dProjected[i].y), 6, Scalar(0, 255, 0), 3);
	}

	projectCoordinateFrame(cFrame);

	ss.str("");
	ss << "=====================================================\n";
	ss << "================= Projected Points ==================\n";
	ss << list_points2dProjected << "\n";

	randn(noise, 0, 0.5);
	add(list_points2dProjected, noise, list_points2dProjected);

	ss << "================ With Noise Points ==================\n";
	ss << list_points2dProjected << "\n";
	commObj.addLog(QString::fromStdString(ss.str()));


	float tolerance = 0.01; // in mm

	bool useGuess = true;
	_methodPNP = 0; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used

	solvePnP(list_points3d, list_points2dProjected, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, _methodPNP);

	ss.str("");
	ss << "=====================================================\n";
	ss << "==================== Iterative =====================\n";
	ss << "rvec: " << "\n";
	ss << Rvec << "\n";
	ss << "tvec: " << "\n";
	ss << Tvec << "\n";

	commObj.addLog(QString::fromStdString(ss.str()));

	_methodPNP = 1; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
	Rvec = cv::Mat::zeros(3, 1, CV_64F);
	Tvec = cv::Mat::zeros(3, 1, CV_64F);
	solvePnP(list_points3d, list_points2dProjected, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, _methodPNP);

	ss.str("");
	ss << "=====================================================\n";
	ss << "====================    EPNP    =====================\n";
	ss << "rvec: " << "\n";
	ss << Rvec << "\n";
	ss << "tvec: " << "\n";
	ss << Tvec << "\n";

	commObj.addLog(QString::fromStdString(ss.str()));

	if (numberMarkers == 4)
	{
		_methodPNP = 2; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
		Rvec = cv::Mat::zeros(3, 1, CV_64F);
		Tvec = cv::Mat::zeros(3, 1, CV_64F);
		solvePnP(list_points3d, list_points2dProjected, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, _methodPNP);

		ss.str("");
		ss << "=====================================================\n";
		ss << "====================     P3P    =====================\n";
		ss << "rvec: " << "\n";
		ss << Rvec << "\n";
		ss << "tvec: " << "\n";
		ss << Tvec << "\n";

		projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
		for (int i = 0; i < numberMarkers; i++)
		{
			circle(cFrame, Point(list_points2dProjected[i].x, list_points2dProjected[i].y), 3, Scalar(255, 0, 0), 3);
		}
		QPixmap QPFrame;
		QPFrame = Mat2QPixmap(cFrame);
		commObj.changeImage(QPFrame);
		QCoreApplication::processEvents();
		commObj.addLog(QString::fromStdString(ss.str()));
	}

	_methodPNP = 4; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
	Rvec = cv::Mat::zeros(3, 1, CV_64F);
	Tvec = cv::Mat::zeros(3, 1, CV_64F);
	solvePnP(list_points3d, list_points2dProjected, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, _methodPNP);

	ss.str("");
	ss << "=====================================================\n";
	ss << "====================    UPNP    =====================\n";
	ss << "rvec: " << "\n";
	ss << Rvec << "\n";
	ss << "tvec: " << "\n";
	ss << Tvec << "\n";

	commObj.addLog(QString::fromStdString(ss.str()));

	Rvec = RvecOriginal;
	Tvec = TvecOriginal;
}

void setupKalmanFilter()
{
	//==-- Transition Matrix:
	//==	1	0	0	1	0	0
	//==	0	1	0	0	1	0
	//==	0	0	1	0	0	1
	//==	0	0	0	1	0	0
	//==	0	0	0	0	1	0
	//==	0	0	0	0	0	1

	KF.transitionMatrix = (Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);
	measurement.setTo(Scalar(0));

	//==-- Position is position and Velocity is 0
	KF.statePre.at<float>(0) = 0;
	KF.statePre.at<float>(1) = 0;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	KF.statePre.at<float>(4) = 0;
	KF.statePre.at<float>(5) = 0;

	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-0));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e1));
	setIdentity(KF.errorCovPost, Scalar::all(1e-0));
}

void projectCoordinateFrame(Mat pictureFrame)
{
projectPoints(coordinateFrame, Rvec, Tvec, cameraMatrix, distCoeffs, coordinateFrameProjected);
line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[3], Scalar(0, 0, 255), 2); //z-axis
line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[1], Scalar(255, 0, 0), 2); //x-axis
line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[2], Scalar(0, 255, 0), 2); //y-axis
}