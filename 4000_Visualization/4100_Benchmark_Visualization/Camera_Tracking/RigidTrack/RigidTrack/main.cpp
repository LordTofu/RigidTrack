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

#include "main.h"
#include "communication.h"
#include "four-point-groebner.hpp"


using namespace CameraLibrary;
using namespace cv;


commObject commObj;
double refX;
double refY;
double refZ;
double yaw;
double pitch;
double roll;
double refYaw;
double refPitch;
double refRoll;

double frameTime = 1 / 100;
double timeOld = 0;
double velocityX;
double velocityY;
double velocityZ;

double scaleX =1;
double scaleY =1;
double scaleZ =1;

double latitudeRef = 47;
double longitudeRef = 11;
double heightRef = 0;
double latitude = 47;
double longitude = 11;
double height = 0;
double earthRadius = 6366743.0; // Radius of the Earth at 47° North in Meters
std::ofstream logfile;


std::vector<double> position = std::vector<double>(3);
std::vector<double> positionOld = std::vector<double>(3);
double distFromRef = 0;
int intIntensity = 6; // max Intensity is 15 1-6 is strobe 7-15 is continuous 13 and 14 are meaningless 
int intExposure = 50; // max is 60 increase if markers are bad visible
int intFrameRate = 100;

//== Rotation, translation etc. matrix for PnP results
Mat Rmat = cv::Mat::zeros(3, 3, CV_64F);
Mat M_NC = cv::Mat::zeros(3, 3, CV_64F);
Mat Rvec = cv::Mat::zeros(3, 1, CV_64F);
Mat Tvec = cv::Mat::zeros(3, 1, CV_64F);
Mat posRef = cv::Mat::zeros(3, 1, CV_64F);
Mat scaleVec = cv::Mat::zeros(3, 1, CV_64F);

cObject *obj;
double objX;
double objY;
Mat points2d;
RotatedRect rotRect;
double majorAxis =0;
double heading = 0;
Vec4d positionHeight = 0;
double cameraAbove = 4300.0; // camera is 4300mm above marker system
double circleRadius = 545.0/2.0;  // Radius of the markers in mm
double picturePlanedistance = 0; // doesnt change, is computed afterwards
double FoV = 57.5; // FoV of Camera in degrees, constant
bool unscaled = true; // SetZero is run 2 Times to get the Scaling Factors

Mat inliers;
bool useGuess = true; // not used
int methodPNP = 0; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
int numMarker = 7; // number of markers, the more the better. 5 is minimum, 6 minimum for position and heading
//== Marker points in real world coordinates	==--
std::vector<Vec3d> list_points3d(7);	//== 7 Marker points has to be changed by hand !!
std::vector<Point> list_points2f(7);
std::vector<Point> list_points2d(7);


bool enableKalman = true;  // always enable
KalmanFilter KF(6, 3, 0);
Mat_<float> measurement(3, 1);

Mat cameraMatrix;
Mat distCoeffs;

QUdpSocket *udpSocketServo;
QUdpSocket *udpSocketDrone;

// which is why we also set this constant to 8 
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

	refX = 0;
	refY = 0;
	refZ = 0;

	scaleVec.at<double>(0, 0) = scaleX;
	scaleVec.at<double>(1, 0) = scaleY;
	scaleVec.at<double>(2, 0) = scaleZ;

	//list_points3f[0] = cv::Vec3f(    0.0  * scale,    0.0 * scale, 0.0 * scale);
	//list_points3f[0] = cv::Vec3f( -730.0  * scale,    0.0 * scale, 0.0 * scale);
	//list_points3f[1] = cv::Vec3f(  -75.0  * scale, -930.0 * scale, 0.0 * scale);
	//list_points3f[2] = cv::Vec3f(  -75.0  * scale,  930.0 * scale, 0.0 * scale);
	//list_points3f[3] = cv::Vec3f( -175.0 * scale,  260.0 * scale, 0.0 * scale);

	list_points3d[0] = cv::Vec3d(   0.0,    0.0,   0.0);
	list_points3d[1] = cv::Vec3d(  55.0, -455.0, 200.0);
	list_points3d[2] = cv::Vec3d(  55.0,  450.0, 200.0);
	list_points3d[3] = cv::Vec3d( 355.0,  -80.0, 205.0);


	//list_points3f[0] = cv::Vec3f(  0.0,     0.0, 0.0);
	//list_points3f[1] = cv::Vec3f(  0.0,   385.0, 0.0);
	//list_points3f[2] = cv::Vec3f(394.0,   185.0, 0.0);
	//list_points3f[3] = cv::Vec3f(607.0,   385.0, 0.0);

	Tvec.at<double>(0, 0) = 0.0;
	Tvec.at<double>(1, 0) = 0.0;
	Tvec.at<double>(2, 0) = 4000.0;
	Rvec.at<double>(0, 0) = 0.0;
	Rvec.at<double>(1, 0) = 0.0;
	Rvec.at<double>(2, 0) = 1.0;

	picturePlanedistance = 640.0 / (2.0 * tan((FoV*3.14159/180.0) / 2.0));

	return a.exec();
}


QPixmap Mat2QPixmap(cv::Mat src)
{
	
	QImage dest((const uchar *)src.data, src.cols, src.rows, src.step, QImage::Format_Grayscale8);
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
	udpSocketServo = new QUdpSocket(0);
	udpSocketDrone = new QUdpSocket(0);

	QHostAddress bcast = QHostAddress("127.0.0.1");
	udpSocketServo->connectToHost(bcast, 5000);

	bcast = QHostAddress("192.168.4.1");
	udpSocketDrone->connectToHost(bcast, 5000);

	commObj.addLog("Opened UDP Port");

	//Enable Stepper Motor
	data.setNum((int)(99999));
	udpSocketServo->write(data);

	//=0 Set Stepper Motor Position to 0
	data.setNum((int)(0));
	udpSocketServo->write(data);

	commObj.changeStatus("Creating MMF");

	HANDLE hMapFile;
	LPCTSTR pBuf;

	TCHAR szMsg[] = TEXT("Message from first process.");



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

	float Value[100];

	commObj.changeStatus("MMF Created");
	Sleep(1000);
	Value[0] = 50;
	Value[1] = 50;
	Value[2] = 50;
	Value[3] = 0;
	Value[4] = 0;
	Value[5] = 0;
	Value[6] = 0;
	Value[7] = 0;
	Value[8] = 0;
	Value[9] = 0;
	Value[10] = 0;
	Value[11] = 0;

	CopyMemory((PVOID)pBuf, &Value, 100 * sizeof(float));
	//== For OptiTrack Ethernet cameras, it's important to enable development mode if you
	//== want to stop execution for an extended time while debugging without disconnecting
	//== the Ethernet devices.  Lets do that now:

	CameraLibrary_EnableDevelopment();

	//== Initialize Camera SDK ==--

	CameraLibrary::CameraManager::X();

	//== At this point the Camera SDK is actively looking for all connected cameras and will initialize
	//== them on it's own.

	//== Now, lets pop a dialog that will persist until there is at least one camera that is initialized
	//== or until canceled.

	//PopWaitingDialog();

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


	//== Open the application window =============================----

	//if (!CreateAppWindow("Camera Library SDK - Sample", cameraWidth, cameraHeight, 32, gFullscreen))
		//return 0;


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

	//== Ok, start main loop.  This loop fetches and displays   ===---
	//== camera frames.                                         ===---
	camera->SetExposure(intExposure);
	camera->SetIntensity(intIntensity);
	camera->SetFrameRate(intFrameRate);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(true);

	//== Fetch a new frame from the camera ===---
	cv::Mat matFrame(cv::Size(cameraWidth, cameraHeight), CV_8UC1);
	QPixmap QPFrame;

	//==-- Kalman Filter creation and init values

	if (enableKalman)
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

		//==-- Position is 0 and Velocity is 0
		KF.statePre.at<float>(0) = 0;
		KF.statePre.at<float>(1) = 0;
		KF.statePre.at<float>(2) = 0;
		KF.statePre.at<float>(3) = 0;
		KF.statePre.at<float>(4) = 0;
		KF.statePre.at<float>(5) = 0;

		setIdentity(KF.measurementMatrix);
		setIdentity(KF.processNoiseCov, Scalar::all(1e-2));
		setIdentity(KF.errorCovPost, Scalar::all(.1));
	}
	int u = 0;
	while (1)
	{
		
		//== Fetch a new frame from the camera ===---

		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.

			//== Lets have the Camera Library raster the camera's
			//== image into our texture.



			if (frame->ObjectCount() == numMarker)
			{


				for (int i = 0; i < frame->ObjectCount(); i++)
				{
					cObject *obj = frame->Object(i);

					list_points2d[i] = cv::Point(obj->X(), obj->Y());

				}

				//solvePnPRansac(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
				findPose4pt_groebner(list_points3d, list_points2d, 0, 520, cv::Point2d(320, 240), Rvec, Tvec, CV_RANSAC, 0.99, 1, noArray());
				//positionHeight = getPosition(frame);
				Tvec.at<double>(2, 0) = picturePlanedistance / positionHeight[2] * circleRadius*scaleZ;					//height
				Tvec.at<double>(0, 0) = Tvec.at<double>(2, 0)*(positionHeight[0] - cameraWidth / 2) / picturePlanedistance;			// x
				Tvec.at<double>(1, 0) = Tvec.at<double>(2, 0)*(positionHeight[1] - cameraHeight / 2) / picturePlanedistance;			// y
				heading = positionHeight[3] * 180.0 / 3.1415926;

				//==-- Rodrigues (Angle Vector) to Rotation Matrix conversion
				//Rodrigues(Rvec, Rmat);

				//==-- Camera pose in world coordinates / hence marker system cosy

				//transpose(Rmat, Rmat);
				//Tvec = M_NC*Rmat*Tvec;

				//==-- Euler Angles 

				//Vec3d eulerAngles;
				//getEulerAngles(Rmat, eulerAngles);

				//==-- yaw   = eulerAngles[1]; 
				//==-- pitch = eulerAngles[0];
				//==-- roll  = eulerAngles[2];

				Value[0] = (float)(posRef.at<double>(0, 0) - Tvec.at<double>(0, 0));
				Value[1] = (float)(posRef.at<double>(1, 0) - Tvec.at<double>(1, 0));
				Value[2] = (float)(posRef.at<double>(2, 0) - Tvec.at<double>(2, 0));

				//Value[3] = (float)refYaw - (float)eulerAngles[1];
				//Value[4] = (float)refPitch - (float)eulerAngles[0];
				//Value[5] = (float)refRoll - (float)eulerAngles[2];

				if (enableKalman) {
					Mat prediction = KF.predict();
					measurement(0) = Value[0];
					measurement(1) = Value[1];
					measurement(2) = Value[2];
					Mat estimation = KF.correct(measurement);
					Value[0] = (float)estimation.at<float>(0) / 1000.0; // mm to meters
					Value[1] = (float)estimation.at<float>(1) / 1000.0;
					Value[2] = (float)estimation.at<float>(2) / 1000.0;
					//velocityX = (float)estimation.at<float>(3) / 10.0; // *1000 from mm/s to meters/s
					//velocityY = (float)estimation.at<float>(4) / 10.0;
					//velocityZ = (float)estimation.at<float>(5) / 10.0;
				}

				position[0] = Value[0];
				position[1] = Value[1];
				position[2] = Value[2];
				frameTime = frame->TimeStamp() - timeOld;
				timeOld = frame->TimeStamp();
				velocityX = (position[0] - positionOld[0]) / frameTime;
				velocityY = (position[1] - positionOld[1]) / frameTime;
				velocityZ = (position[2] - positionOld[2]) / frameTime;
				positionOld = position;

				latitude = latitudeRef + atan(Value[0] / earthRadius);
				longitude = longitudeRef + atan(Value[1] / earthRadius);


				//Send Start Packet
				data.setNum((int)(-9991));
				udpSocketDrone->write(data);
				//Send Longitude
				data.setNum(latitude);
				udpSocketDrone->write(data);
				//Send latitude
				data.setNum(longitude);
				udpSocketDrone->write(data);
				//Send Height
				data.setNum((double)(Value[2]));
				udpSocketDrone->write(data);
				//Send Heading
				data.setNum((double)(heading));
				udpSocketDrone->write(data);
				//Send Velocity X
				data.setNum((double)(velocityX));
				udpSocketDrone->write(data);
				//Send Velocity Y
				data.setNum((double)(velocityY));
				udpSocketDrone->write(data);
				//Send Velocity Z
				data.setNum((double)(velocityZ));
				udpSocketDrone->write(data);

				CopyMemory((PVOID)pBuf, &Value, 100 * sizeof(double));
				//distFromRef = std::sqrtl(std::pow(Value[0], 2)+ std::pow(Value[1], 2)+ std::pow(Value[2], 2));
				u += 1;
				if (u == 10) {

					ss.str("");
					ss << "X   =  " << Value[0] << "\tY  =  " << Value[1] << "\tZ  = " << Value[2] << "\t Heading = " << heading << "\n";
					ss << "VX  =  " << velocityX << "\tVY  =  " << velocityY << "\tVZ  = " << velocityZ << "\n";
					//ss.str("");
					//ss << "pitch  =  " << eulerAngles[0] << "\tyaw  =  " << eulerAngles[1] << "\troll  = " << eulerAngles[2];

					commObj.addLog(QString::fromStdString(ss.str()));
					u = 0;
				}

				logfile.open("logData.txt", std::ios::app);
				logfile << frame->TimeStamp() << ";" << Value[0] << ";" << Value[1] << ";" << Value[2] << ";" << heading << ";" << velocityX << ";" << velocityY << ";" << velocityZ << "\n";
				logfile.close();

				frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);
				//imwrite("test.jpg" + , matFrame);
				QPFrame = Mat2QPixmap(matFrame);
				commObj.changeImage(QPFrame);
				QCoreApplication::processEvents();
				frame->Release();
			}
		}
			//Sleep(1);
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
	refX = 0;
	refY = 0;
	refZ = 0;
	ss.str("");
	ss << "Started Reference Coordinate Determination";
	commObj.addLog(QString::fromStdString(ss.str()));
	CameraLibrary_EnableDevelopment();

	//== Initialize Camera SDK ==--

	CameraLibrary::CameraManager::X();

	//== At this point the Camera SDK is actively looking for all connected cameras and will initialize
	//== them on it's own.

	//== Now, lets pop a dialog that will persist until there is at least one camera that is initialized
	//== or until canceled.

	//PopWaitingDialog();

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

	cv::Mat matFrame(cv::Size(cameraWidth, cameraHeight), CV_8UC1);
	//== Open the application window =============================----

	//if (!CreateAppWindow("Camera Library SDK - Sample", cameraWidth, cameraHeight, 32, gFullscreen))
	//return 0;


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

	//== Ok, start main loop.  This loop fetches and displays   ===---
	//== camera frames.                                         ===---
	camera->SetExposure(intExposure);
	camera->SetFrameRate(intFrameRate);
	camera->SetIntensity(intIntensity);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	int number_samples = 0;
	int numberToSample = 50;
	while (number_samples<numberToSample)
	{

		//== Fetch a new frame from the camera ===---

		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.

			//== Lets have the Camera Library raster the camera's
			//== image into our texture.

			if (frame->ObjectCount() == numMarker)
			{

				//==-- one sample more :D
				number_samples += 1;

				//==for(int i=0; i<frame->ObjectCount(); i++)
				for (int i = 0; i < numMarker; i++)
				{
					cObject *obj = frame->Object(i);

					list_points2d[i] = cv::Point(obj->X(), obj->Y());

				}


				//solvePnPRansac(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
				findPose4pt_groebner(list_points3d, list_points2d, 0, 520, cv::Point2d(320, 240), Rvec, Tvec, CV_RANSAC, 0.99, 1, noArray());
				//positionHeight = getPosition(frame);
			
				Tvec.at<double>(2, 0); //= picturePlanedistance/positionHeight[2]*circleRadius*scaleZ; //height 
				Tvec.at<double>(0, 0);// = Tvec.at<double>(2, 0)*(positionHeight[0] - cameraWidth / 2) / picturePlanedistance;			// x
				Tvec.at<double>(1, 0);// = Tvec.at<double>(2, 0)*(positionHeight[1] - cameraHeight / 2) / picturePlanedistance;			// y
				heading = positionHeight[3];

				ss.str("");
				ss << "Rvec  =  " << Rvec;

				commObj.addLog(QString::fromStdString(ss.str()));
				//==-- Rodrigues (Angle Vector) to Rotation Matrix conversion
				Rodrigues(Rvec, Rmat);

				//==-- Camera pose in world coordinates / hence marker system cosy

				//transpose(Rmat, Rmat);
				//M_NC += Rmat;
				

				refX += Tvec.at<double>(0, 0);
				refY += Tvec.at<double>(1, 0);
				refZ += Tvec.at<double>(2, 0);

				//==-- Euler Angles 

				Vec3d eulerAngles;
				getEulerAngles(Rmat, eulerAngles);

				yaw   = eulerAngles[1]; 
				pitch = eulerAngles[0];
				roll  = eulerAngles[2];
				refYaw += eulerAngles[1];
				refPitch += eulerAngles[0];
				refRoll += eulerAngles[2];
				
			}


			frame->Release();
		}

		Sleep(2);


	}

	//==-- get average of all samples
	refX /= numberToSample;
	refY /= numberToSample;
	refZ /= numberToSample;

	if(unscaled)
	{
	scaleZ = cameraAbove / refZ;
	}
	refZ = cameraAbove;

	posRef.at<double>(0, 0) = refX;
	posRef.at<double>(1, 0) = refY;
	posRef.at<double>(2, 0) = refZ;

	refYaw /= numberToSample;
	refPitch /= numberToSample;
	refRoll /= numberToSample;
	//== Release camera ==--

	camera->Release();
	
	ss.str("");
	ss << "Ref X = " << refX << " Ref Y = " << refY << "Ref Z = " <<refZ;
	commObj.addLog(QString::fromStdString(ss.str()));
	ss.str("");
	ss << "Yaw = " << refYaw << "Pitch  = " << refPitch << "Roll = " << refRoll;
	commObj.addLog(QString::fromStdString(ss.str()));
	
	if(unscaled){
		unscaled = false;
		setZero();
	}
	return 0;

}

void calibrate_camera()
{
	CameraLibrary_EnableDevelopment();

	//== Initialize Camera SDK ==--

	CameraLibrary::CameraManager::X();

	//== At this point the Camera SDK is actively looking for all connected cameras and will initialize
	//== them on it's own.

	//== Now, lets pop a dialog that will persist until there is at least one camera that is initialized
	//== or until canceled.

	//PopWaitingDialog();

	//== Get a connected camera ================----


	CameraManager::X().WaitForInitialization();

	Camera *camera = CameraManager::X().GetCamera();

	//== If no device connected, pop a message box and exit ==--


	//== Determine camera resolution to size application window ==----

	int cameraWidth = camera->Width();
	int cameraHeight = camera->Height();


	//== Open the application window =============================----

	//if (!CreateAppWindow("Camera Library SDK - Sample", cameraWidth, cameraHeight, 32, gFullscreen))
	//return 0;


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

	//== Turn on some overlay text so it's clear things are     ===---
	//== working even if there is nothing in the camera's view. ===---

	

	//== Camera Matrix creation	==--
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	distCoeffs = Mat::zeros(8, 1, CV_64F);

	//== Ok, start main loop.  This loop fetches and displays   ===---
	//== camera frames.                                         ===---
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
	Size boardSize(9,6);
	Size imageSize(cameraWidth, cameraHeight);
	Mat Rvec(3, 1, DataType<double>::type);
	Mat Tvec(3, 1, DataType<double>::type);
	float squareSize = 23;

	QPixmap QPFrame;

	while (number_samples<imagesToSample)
	{

		//== Fetch a new frame from the camera ===---
		cv::Mat matFrame(cv::Size(cameraWidth, cameraHeight), CV_8UC1);

		// which is why we also set this constant to 8 
		const int BACKBUFFER_BITSPERPIXEL = 8;


		// later on, when we get the frame as usual:
		CameraLibrary::Frame * frame = camera->GetFrame();
		

		if (frame)
		{

			//== Ok, we've received a new frame, lets do something
			//== with it.

			//== Lets have the Camera Library raster the camera's
			//== image into our texture.
			
			frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);
			//imwrite("test.jpg" + , matFrame);
			QPFrame = Mat2QPixmap(matFrame);
			commObj.changeImage(QPFrame);
			found = findChessboardCorners(matFrame, boardSize, pointBuf,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			if (found)                // If done with success,
			{
				// improve the found corners' coordinate accuracy for chessboard
				
					
					cornerSubPix(matFrame, pointBuf, Size(11, 11),
						Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

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

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,distCoeffs, Rvec, Tvec);

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

	//== Exit the application.  Simple! ==--
}

void load_calibration() {
	//try {
		FileStorage fs;
		fs.open("calibration.xml", FileStorage::READ);
		fs["CameraMatrix"] >> cameraMatrix;
		fs["DistCoeff"] >> distCoeffs;
		//double numbers = distCoeffs.at<double>(0);
		//if (distCoeffs.at<double>(0) > 0) {
		//	throw;
		//}
		commObj.addLog("Loaded Calibration!");
		
		ss.str("");
		ss << "CameraMatrix" << cameraMatrix;
		ss << "DistCoeff" << distCoeffs;
		
		commObj.changeStatus(QString::fromStdString(strBuf));
		commObj.addLog(QString::fromStdString(ss.str()));
	//}
	//catch (...){
	//	commObj.addLog("Loading Calibration failed!");
	//}
}

float l2_norm(std::vector<float> const& u) {
	double accum = 0.;
	for (int i = 0; i < u.size(); ++i) {
		accum += u[i] * u[i];
	}
	return sqrt(accum);
}

// Heres the calculation of the ellipse fit and major Axis and Heading
Vec4d getPosition(CameraLibrary::Frame* frame)
{
	Vec4d centerAxis = cv::Vec4d(3);
	Mat r = cv::Mat(numMarker, 2, CV_32F);
	Mat rCropped = cv::Mat(0, 2, CV_32F);
	float heading = 0;
	float centerX = 0;
	float centerY = 0;
	int maxIndex = 0;


	for (int i = 0; i < numMarker; i++)
	{
		obj = frame->Object(i);

		objX = obj->X();
		objY = obj->Y();

		//std::cout << objX << "\t\t" << objY << "\n";
		list_points2f.at(i) = cv::Point(objX, objY);
		r.at<float>(i, 0) = objX;
		r.at<float>(i, 1) = objY;
		centerX += objX;
		centerY += objY;
	}

	centerX /= frame->ObjectCount();
	centerY /= frame->ObjectCount();

	Point2f center = Point2f(centerX, centerY);
	Point2f currentPoint = Point2f(centerX, centerY); // outlier point for heading calculation
	float distance[6];

	for (int i = 0; i < numMarker; i++)
	{
		currentPoint.x = r.at<float>(i, 0);
		currentPoint.y = r.at<float>(i, 1);
		distance[i] = cv::norm(Mat(currentPoint), Mat(center));

	}

	const int N = sizeof(distance) / sizeof(float);
	maxIndex = std::distance(distance, std::max_element(distance, distance + N));
	currentPoint = Point(r.row(maxIndex));


	int u = 0;
	for (int i = 0; i < numMarker; i++)
	{
		if (i != maxIndex) {
			rCropped.push_back(r.row(u));
			u++;
		}
	}

	// Fit ellipse to marker points fitEllipse(r); is for position and height without heading marker
	RotatedRect rotRect = fitEllipse(rCropped); // Change to fitEllipse(rCropped); for heading calculation

	if (rotRect.size.height > rotRect.size.width)
	{
		majorAxis = rotRect.size.height/2.0;
	}
	else
	{
		majorAxis = rotRect.size.width/2.0;
	}

	//Calculate Heading
	float ellips_centerX = rotRect.center.x;
	float ellips_centerY = rotRect.center.y;
	float headX = (currentPoint.x - ellips_centerX);
	float headY = (currentPoint.y - ellips_centerY);
	heading = atan2(headY, headX);

	centerAxis = cv::Vec4d(rotRect.center.x, rotRect.center.y, majorAxis, heading);
	return centerAxis;
}

