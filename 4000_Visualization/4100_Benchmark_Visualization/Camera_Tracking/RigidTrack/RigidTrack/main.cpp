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
int intExposure = 50; // max is 60 increase if markers are badly visible
int intFrameRate = 100;

//== Rotation, translation etc. matrix for PnP results
Mat Rmat = cv::Mat::zeros(3, 3, CV_64F);	//Rotation Matrix from Marker CoSy to Camera
Mat M_NC = cv::Mat::zeros(3, 3, CV_64F);	// Rotation Matrix from Camera to Ground
Mat Rvec = cv::Mat::zeros(3, 1, CV_64F);	//Rotation Vector (Axis-Angle) from Marker CoSy to Camera
Mat Tvec = cv::Mat::zeros(3, 1, CV_64F);	//Translation Vector from Marker CoSy to Camera

float Value[100] = { 0 };	//100 Values can be sent via MMF, can be more but should be enough for now

RotatedRect rotRect;
double majorAxis =0;
double heading = 0;
Vec4d positionHeight = 0;
double cameraAbove = 4300.0; // camera is 4300mm above marker system
double circleRadius = 545.0/2.0;  // Radius of the markers in mm
double picturePlanedistance = 0; // doesnt change, is computed afterwards
double FoV = 57.5; // FoV of Camera in degrees, constant
bool unscaled = true; // SetZero is run 2 Times to get the Scaling Factors

bool useGuess = true; // not used
int methodPNP = 0; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
int numMarker = 4; // number of markers, the more the better. 5 is minimum, 6 minimum for position and heading

//== Marker points in real world coordinates and in camera pixel coordinates	==--
std::vector<Point3d> list_points3d(4);
std::vector<Point2d> list_points2d(4);

bool enableKalman = true;  // always enable
KalmanFilter KF(6, 3, 0);
Mat_<float> measurement(3, 1);

Mat cameraMatrix;
Mat distCoeffs;

QUdpSocket *udpSocketServo;
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

	list_points3d[0] = cv::Point3d(   0.0,    0.0,   0.0);
	list_points3d[1] = cv::Point3d(  55.0, -455.0, 200.0);
	list_points3d[2] = cv::Point3d(  55.0,  450.0, 200.0);
	list_points3d[3] = cv::Point3d( 355.0,  -80.0, 205.0);

	picturePlanedistance = 640.0 / (2.0 * tan((FoV*3.14159/180.0) / 2.0));
	test_Algorithm();
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
		setupKalmanFilter();
	}

	//Helper Variable used to print ouput only every 10th time
	int u = 0;

	while (1)
	{
		
		//== Fetch a new frame from the camera ===---
		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.

			if (frame->ObjectCount() == numMarker)
			{
				for (int i = 0; i < frame->ObjectCount(); i++)
				{
					cObject *obj = frame->Object(i);
					list_points2d[i] = cv::Point2d(obj->X(), obj->Y());
				}
				//Compute the pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);

				Rodrigues(Rvec, Rmat);
				Rmat = Rmat.t();  // rotation of inverse
				Mat V = -Rmat * (Mat)Tvec; // translation of inverse
				position = (Mat)V;
				//==-- Euler Angles, finally 
				getEulerAngles(Rmat, eulerAngles);

				if (enableKalman) {
					Mat prediction = KF.predict();
					measurement(0) = (float)Tvec.at<double>(0);
					measurement(1) = (float)Tvec.at<double>(1);
					measurement(2) = (float)Tvec.at<double>(2);
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
				data.setNum((double)(velocity[0]));
				udpSocketDrone->write(data);
				//Send Velocity Y
				data.setNum((double)(velocity[1]));
				udpSocketDrone->write(data);
				//Send Velocity Z
				data.setNum((double)(velocity[2]));
				udpSocketDrone->write(data);

				CopyMemory((PVOID)pBuf, &Value, 100 * sizeof(double));
				//distFromRef = std::sqrtl(std::pow(Value[0], 2)+ std::pow(Value[1], 2)+ std::pow(Value[2], 2));
				u += 1;
				if (u == 10) {
					ss.str("");
					ss << "X   =  " << position[0] << "\tY  =  " << position[1] << "\tZ  = " << position[2] << "\t Heading = " << heading << "\n";
					ss << "VX  =  " << velocity[0] << "\tVY  =  " << velocity[1] << "\tVZ  = " << velocity[2] << "\n";
					ss << "pitch  =  " << eulerAngles[0] << "\tyaw  =  " << eulerAngles[1] << "\troll  = " << eulerAngles[2];
					commObj.addLog(QString::fromStdString(ss.str()));
					u = 0;
				}

				logfile.open("logData.txt", std::ios::app);
				logfile << frame->TimeStamp() << ";" << position[0] << ";" << position[1] << ";" << position[2] << ";";
				logfile << eulerAngles[0] << ";" << eulerAngles[1] << ";" << eulerAngles[2] << ";";
				logfile << velocity[0] << ";" << velocity[1] << ";" << velocity[2] << "\n";
				logfile.close();

				frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);
				QPFrame = Mat2QPixmap(matFrame);
				commObj.changeImage(QPFrame);
				QCoreApplication::processEvents();
				frame->Release();
			}
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
	cv::Mat matFrame(cv::Size(cameraWidth, cameraHeight), CV_8UC1);

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
	camera->SetFrameRate(intFrameRate);
	camera->SetIntensity(intIntensity);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	int numberSamples = 0;
	int numberToSample = 50;

	while (numberSamples<numberToSample)
	{
		//== Fetch a new frame from the camera ===---
		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.
			if (frame->ObjectCount() == numMarker)
			{
				//==-- one sample more :D
				numberSamples += 1;

				//==for(int i=0; i<frame->ObjectCount(); i++)
				for (int i = 0; i < numMarker; i++)
				{
					cObject *obj = frame->Object(i);
					list_points2d[i] = cv::Point2d(obj->X(), obj->Y());
				}

				//Compute the pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
				add(posRef, Tvec, posRef);
				add(eulerRef, Rvec, eulerRef); // That are not the values of yaw, roll and pitch yet! Rodriguez has to be called first. 

				ss.str("");
				ss << "Tvec  =  " << Tvec << "\n";
				commObj.addLog(QString::fromStdString(ss.str()));
			}
			frame->Release();
		}
	}
	//== Release camera ==--
	camera->Release();
	return 0;
	
	//Divide by the number of samples to get the mean of the reference position
	divide(posRef, numberToSample, posRef);
	divide(eulerRef, numberToSample, eulerRef);

	//==-- Rodrigues (Angle Vector) to Rotation Matrix conversion
	Rodrigues(eulerRef, Rmat); 
	Rmat = Rmat.t();  // rotation of inverse
	Mat V = -Rmat * (Mat)posRef; // translation of inverse
	posRef = (Mat)V;
	//==-- Euler Angles, finally 
	getEulerAngles(Rmat, eulerRef);
	
	ss.str("");
	ss << "================= Reference Position ==================\n";
	ss << posRef << "\n";
	ss << "=============== Reference Euler Angles ================\n";
	ss << eulerRef << "\n";
	commObj.addLog(QString::fromStdString(ss.str()));
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
		FileStorage fs;
		fs.open("calibration.xml", FileStorage::READ);
		fs["CameraMatrix"] >> cameraMatrix;
		fs["DistCoeff"] >> distCoeffs;
		commObj.addLog("Loaded Calibration!");
		ss.str("");
		ss << "Camera Matrix\n" << cameraMatrix << "\n";
		ss << "Distortion Coeff\n" << distCoeffs << "\n";
		commObj.changeStatus(QString::fromStdString(strBuf));
		commObj.addLog(QString::fromStdString(ss.str()));
}

float l2_norm(std::vector<float> const& u) {
	double accum = 0.;
	for (int i = 0; i < u.size(); ++i) {
		accum += u[i] * u[i];
	}
	return sqrt(accum);
}

void test_Algorithm()
{

	load_calibration();

	std::vector<Point2d> noise(4);

	Mat Rvec = (cv::Mat_<double>(3, 1) << 0.78, 0.0, 0.0);
	Mat Tvec = (cv::Mat_<double>(3, 1) << 60.0, 30.0, 1000);

	Mat RvecOriginal = Rvec;
	Mat TvecOriginal = Tvec;

	multiply(list_points3d, 0.2, list_points3d);

	projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2d);
	ss.str("");
	ss << "=====================================================\n";
	ss << "================= Projected Points ==================\n";
	ss << list_points2d << "\n";

	randn(noise, 0, 2);
	add(list_points2d, noise, list_points2d);
	
	ss << "================ With Noise Points ==================\n";
	ss << list_points2d << "\n";
	commObj.addLog(QString::fromStdString(ss.str()));

	bool sufficientAccuracy = false;
	float tolerance = 0.01; // in mm

	bool useGuess = true;
	int methodPNP = 0; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
	Rvec = cv::Mat::zeros(3, 1, CV_64F);
	Tvec = cv::Mat::zeros(3, 1, CV_64F);
	solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
	if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }

	ss.str("");
	ss << "=====================================================\n";
	ss << "==================== Iterative =====================\n";
	ss << "rvec: " << "\n";
	ss << Rvec << "\n";
	ss << "tvec: " << "\n";
	ss << Tvec << "\n";

	commObj.addLog(QString::fromStdString(ss.str()));

	methodPNP = 1; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
	Rvec = cv::Mat::zeros(3, 1, CV_64F);
	Tvec = cv::Mat::zeros(3, 1, CV_64F);
	solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
	if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }

	ss.str("");
	ss << "=====================================================\n";
	ss << "====================    EPNP    =====================\n";
	ss << "rvec: " << "\n";
	ss << Rvec << "\n";
	ss << "tvec: " << "\n";
	ss << Tvec << "\n";

	commObj.addLog(QString::fromStdString(ss.str()));

	methodPNP = 2; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
	Rvec = cv::Mat::zeros(3, 1, CV_64F);
	Tvec = cv::Mat::zeros(3, 1, CV_64F);
	solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
	if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }

	ss.str("");
	ss << "=====================================================\n";
	ss << "====================     P3P    =====================\n";
	ss << "rvec: " << "\n";
	ss << Rvec << "\n";
	ss << "tvec: " << "\n";
	ss << Tvec << "\n";

	commObj.addLog(QString::fromStdString(ss.str()));

	methodPNP = 4; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
	Rvec = cv::Mat::zeros(3, 1, CV_64F);
	Tvec = cv::Mat::zeros(3, 1, CV_64F);
	solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
	if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }

	ss.str("");
	ss << "=====================================================\n";
	ss << "====================    UPNP    =====================\n";
	ss << "rvec: " << "\n";
	ss << Rvec << "\n";
	ss << "tvec: " << "\n";
	ss << Tvec << "\n";

	commObj.addLog(QString::fromStdString(ss.str()));

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
	KF.statePre.at<float>(0) = position[0];
	KF.statePre.at<float>(1) = position[1];;
	KF.statePre.at<float>(2) = position[2];;
	KF.statePre.at<float>(3) = 0;
	KF.statePre.at<float>(4) = 0;
	KF.statePre.at<float>(5) = 0;

	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-2));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
}