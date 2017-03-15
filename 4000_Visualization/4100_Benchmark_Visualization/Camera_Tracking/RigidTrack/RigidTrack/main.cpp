#include "RigidTrack.h"
#include <QtWidgets/QApplication>
#include <QDesktopServices>
#include <QUrl>
#include <QThread>
#include <QUdpSocket>
#include <QFileDialog> 
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
#include <random>
#include <thread>
#include <strsafe.h>

#include "main.h"
#include "communication.h"

using namespace CameraLibrary;
using namespace cv;

commObject commObj;

bool safetyEnable = false; // is the safety feature enabled
bool safety2Enable = false;	// is the second receiver enabled
double safetyBoxLength = 1.5; // length of the safety area cube in meters
int safetyAngle = 30; // bank and pitch angle protection in degrees
bool exitRequested = true; // variable if tracking loop should be exited
int invertZ = 1; // dummy variable to invert Z direction on request

double frameTime = 0.01; // 100 Hz CoSy rate, is later on replaced with the hardware timestamp delivered by the camera 
double timeOld = 0.0;		// old time for finite differences velocity calculation. Is later on replaced with the hardware timestamp delivered by the camera
double timeFirstFrame = 0; // Time stamp of the first frame. This value is then subtracted for every other frame so the time in the log start at zero.

Vec3d position = Vec3d();	// position vector x,y,z for object position in O-CoSy, unit is meter
Vec3d eulerAngles = Vec3d(); // Roll Pitch Heading in this order, units in degrees
Vec3d positionOld = Vec3d();	// old position in O-CoSy for finite differences velocity calculation 
Vec3d velocity = Vec3d();	// velocity vector of object in o-CoSy in respect to o-CoSy
Vec3d posRef = Vec3d();		// initial position of object in camera CoSy
Vec3d eulerRef = Vec3d();	// initial euler angle of object respectivley to camera CoSy
double headingOffset = 0;	// heading offset variable for aligning INS heading with tracking heading

int intIntensity = 15; // max infrared spot light intensity is 15 1-6 is strobe 7-15 is continuous 13 and 14 are meaningless 
int intExposure = 1; // max is 480 increase if markers are badly visible but should be determined automatically during setZero()
int intFrameRate = 100;	// CoSy rate of camera, maximum is 100 fps
int intThreshold = 200;	// threshold value for marker detection. If markers are badly visible lower this value but should not be necessary

//== Rotation, translation etc. matrix for PnP results
Mat Rmat = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);		// rotation matrix from camera CoSy to marker CoSy
Mat RmatRef = (cv::Mat_<double>(3, 3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);	// reference rotation matrix from camera CoSy to marker CoSy
Mat M_NC = cv::Mat_<double>(3, 3);							// rotation matrix from camera to ground, fixed for given camera position
Mat M_HeadingOffset = cv::Mat_<double>(3, 3);				// rotation matrix that turns the ground system to the INS magnetic heading for alignment
Mat Rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	// rotation vector (axis-angle notation) from camera CoSy to marker CoSy
Mat Tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	// translation vector from camera CoSy to marker CoSy in camera CoSy
Mat RvecOriginal;	// initial values as start values for algorithms and algorithm tests
Mat TvecOriginal;	// initial values as start values for algorithms and algorithm tests

bool useGuess = true; // set to true and the algorithm uses the last result as starting value
int methodPNP = 0; // solvePNP algorithm 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // 4 and 1 are the same and not implemented correctly by OpenCV
int numberMarkers = 4; // number of markers. Is loaded during start up from the marker configuration file
std::vector<Point3d> list_points3d;	// marker positions in marker CoSy 
std::vector<Point2d> list_points2d;	// marker positions projected in 2D in camera image CoSy
std::vector<Point2d> list_points2dOld;	// marker positions in previous picture in 2D in camera image CoSy
std::vector<double> list_points2dDifference;	// difference of the old and new 2D marker position to determine the order of the points
std::vector<Point2d> list_points2dProjected;	// 3D marker points projected to 2D in camera image CoSy with the algorithm projectPoints
std::vector<Point2d> list_points2dUnsorted;	// marker points in 2D camera image CoSy, sorted with increasing x (camera image CoSy) but not sorted to correspond with list_points3d
std::vector<Point3d> coordinateFrame;	// coordinate visualisazion of marker CoSy
std::vector<Point2d> coordinateFrameProjected;	// marker CoSy projected from 3D to 2D camera image CoSy
int pointOrderIndices[] = { 0, 1, 2, 3 };	// old correspondence from list_points3d and list_points_2d
int pointOrderIndicesNew[] = { 0, 1, 2, 3 };	// new correspondence from list_points3d and list_points_2d
double currentPointDistance = 5000;	// distance from the projected 3D points (hence in 2d) to the real 2d marker positions in camera image CoSy 
double minPointDistance = 5000;	// minimum distance from the projected 3D points (hence in 2d) to the real 2d marker positions in camera image CoSy 
int currentMinIndex = 0;	// helper variable set to the point order that holds the current minimum point distance 
bool gotOrder = false;	// order of the list_points3d and list_points3d already tetermined or not, has to be done once

bool camera_started = false; // variable thats needed to exit the main while loop

Mat cameraMatrix;	// camera matrix of the camera
Mat distCoeffs;	// distortion coefficients of the camera
Core::DistortionModel distModel;	// distortion model of the camera

// IP adress of the circuit breaker that disables the object if a specified region is exited.
QUdpSocket *udpSocketObject;	// socket for the communication with the object
QUdpSocket *udpSocketSafety;	// socket for the communication with the circuit breaker
QUdpSocket *udpSocketSafety2;	// socket for the communication with the rope winch
QHostAddress IPAdressObject = QHostAddress("192.168.137.254");	// IPv4 adress of the object wifi telemetry chip, can change to 192.168.4.x. This is where the position etc is sent to.
QHostAddress IPAdressSafety = QHostAddress("192.168.4.1"); // IPv4 adress of the circuit breaker, stays the same
QHostAddress IPAdressSafety2 = QHostAddress("192.168.4.4");	// IPv4 adress of the rope winch,
int portObject = 9155; // Port of the object
int portSafety = 9155; // Port of the safety switch
int portSafety2 = 9155; // Port of the second receiver
QByteArray datagram;	// data package that is sent to the object 
QByteArray data;	// data package that's sent to the circuit breaker
QDataStream out;	// stream that sends the datagram package via UDP

const int BACKBUFFER_BITSPERPIXEL = 8;	// 8 bit per pixel and greyscale image from camera
std::string strBuf;	// buffer that holds the strings that are sent to the Qt GUI
std::stringstream ss;	// stream that sends the strBuf buffer to the Qt GUI
QString logFileName; // Filename for the logfiles
std::string logName; // Filename for the logfiles as standard string
SYSTEMTIME logDate; // Systemtime struct that saves the current date and time thats needed for the log file name creation
std::ofstream logfile;	// file handler for writing the log file


// main inizialised the GUI and values for the marker position
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	RigidTrack w;
	w.show();	// show the GUI
	// connect the Qt slots and signals for event handling
	QObject::connect(&commObj, SIGNAL(statusChanged(QString)), &w, SLOT(setStatus(QString)), Qt::DirectConnection);
	QObject::connect(&commObj, SIGNAL(imageChanged(QPixmap)), &w, SLOT(setImage(QPixmap)), Qt::DirectConnection);
	QObject::connect(&commObj, SIGNAL(logAdded(QString)), &w, SLOT(setLog(QString)), Qt::DirectConnection);
	QObject::connect(&commObj, SIGNAL(logCleared()), &w, SLOT(clearLog(QString)), Qt::DirectConnection);
	QObject::connect(&commObj, SIGNAL(P3Penabled(bool)), &w, SLOT(enableP3P(bool)), Qt::DirectConnection);
	QObject::connect(&commObj, SIGNAL(progressUpdated(int)), &w, SLOT(progressUpdate(int)), Qt::DirectConnection);

	commObj.addLog("RigidTrack Version:");
	commObj.addLog(QString::number(_MSC_FULL_VER));
	commObj.addLog("Built on:");
	commObj.addLog(QString(__DATE__));

	// initial guesses for position and rotation, important for Iterative Method!
	Tvec.at<double>(0) = 45;
	Tvec.at<double>(1) = 45;
	Tvec.at<double>(2) = 4500;
	Rvec.at<double>(0) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(1) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(2) = -45 * 3.141592653589 / 180.0;

	// Points that make up the marker CoSy axis system, hence one line in each axis direction
	coordinateFrame = std::vector<Point3d>(4);
	coordinateFrameProjected = std::vector<Point2d>(4);
	coordinateFrame[0] = cv::Point3d(0, 0, 0);
	coordinateFrame[1] = cv::Point3d(300, 0, 0);
	coordinateFrame[2] = cv::Point3d(0, 300, 0);
	coordinateFrame[3] = cv::Point3d(0, 0, 300);

	position[0] = 1.1234;	  // set position initial values
	position[1] = 1.2345;	  // set position initial values
	position[2] = 1.3456;	  // set position initial values

	velocity[0] = 0.123;	// set velocity initial values
	velocity[1] = 0.234;	// set velocity initial values
	velocity[2] = 0.345;	// set velocity initial values

	eulerAngles[0] = 1.002;	// set initial euler angles to arbitrary values for testing
	eulerAngles[1] = 1.003;	// set initial euler angles to arbitrary values for testing
	eulerAngles[2] = 1.004;	// set initial euler angles to arbitrary values for testing

	setHeadingOffset(0.0); // set the heading offset to 0 

	ss.precision(4); // outputs in the log etc are limited to 3 decimal values

	loadCameraPosition(); // load the rotation matrix from camera CoSy to ground CoSy
	load_calibration(0); // load the calibration file with the camera intrinsics
	loadMarkerConfig(0); // load the standard marker configuration
	test_Algorithm();	// test the algorithms and their accuracy

	return a.exec();
}

// convert a opencv matrix that represents a picture to a Qt Pixmap object
QPixmap Mat2QPixmap(cv::Mat src)
{
	QImage dest((const uchar *)src.data, src.cols, src.rows, src.step, QImage::Format_RGB888);
	dest.bits(); // enforce deep copy, see documentation 
				 // of QImage::QImage ( const uchar * data, int width, int height, Format format )
	QPixmap pixmapDest = QPixmap::fromImage(dest);
	return pixmapDest;
}

// calculate the chess board corner positions, used for the camera calibration
void calcBoardCornerPositions(Size boardSize, float squareSize, std::vector<Point3f>& corners)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
}

// get the euler angles from a rotation matrix
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

// start the loop that fetches frames, computes the position etc and sends it to the object
int start_camera() {

	gotOrder = false;
	Rvec = RvecOriginal;
	Tvec = TvecOriginal;
	GetLocalTime(&logDate);
	logFileName = "./logs/positionLog_" + QString::number(logDate.wDay) + "_" + QString::number(logDate.wMonth) + "_" + QString::number(logDate.wYear);
	logFileName += "_" + QString::number(logDate.wHour) + "_" + QString::number(logDate.wMinute) + "_" + QString::number(logDate.wSecond) + ".txt";
	logName = logFileName.toStdString(); // save the filename as standard string

	determineExposure();

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

	camera->SetVideoType(Core::PrecisionMode);	// set the camera mode to precision mode, it used greyscale imformation for marker property calculations

	//== Start camera output ==--
	camera->Start();

	//== Turn on some overlay text so it's clear things are     ===---
	//== working even if there is nothing in the camera's view. ===---
	camera->SetTextOverlay(true);
	camera->SetExposure(intExposure);	// set the camera exposure
	camera->SetIntensity(intIntensity);	// set the camera infrared LED intensity
	camera->SetFrameRate(intFrameRate);	// set the camera framerate to 100 Hz
	camera->SetIRFilter(true);	// enable the filter that blocks visible light and only passes infrared light
	camera->SetHighPowerMode(true);	// enable high power mode of the leds
	camera->SetContinuousIR(false);	// enable continuous LED light
	camera->SetThreshold(intThreshold);	// set threshold for marker detection

	// Create a new matrix that stores the picture
	Mat matFrame = Mat::zeros(cv::Size(cameraWidth, cameraHeight), CV_8UC1);
	QPixmap QPFrame;
	// Matrix that stores the colored picture
	Mat cFrame(480, 640, CV_8UC3, Scalar(0, 0, 0));

	int v = 0;	//helper variable used to kick safety switch
	// variables for the min and max values that are needed for sanity checks
	double maxValue = 0;
	double minValue = 0;
	int framesDropped = 0; // if a marker is not visible or accuracy is bad increase this counter.
	double projectionError = 0; // equals the quality of the tracking

	
	setUpUDP();	// open sockets and ports for UDP communication

	if (safetyEnable) // if the safety feature is enabled enable the circuit breaker
	{
		//enable circuit breaker, hence send a 9 and then a 1 to it
		data.setNum((int)(9));
		udpSocketSafety->write(data);
		data.setNum((int)(1));
		udpSocketSafety->write(data);

	}

	//== Fetch a new frame from the camera ===---
	bool gotTime = false;
	while (!gotTime)
	{
		Frame *frame = camera->GetFrame();
		if (frame)
		{
			timeFirstFrame = frame->TimeStamp(); // get the time stamp for the first frame. It is subtracted for the following frames
			frame->Release();
			gotTime = true;
		}
	}
	

	// now enter the main loop that processes each frame
	while (!exitRequested)
	{
		//== Fetch a new frame from the camera ===---
		Frame *frame = camera->GetFrame();

		if (frame)
		{
			framesDropped++;	// increase the helper variable by 1
			//== Ok, we've received a new frame, lets do something
			//== with it.

			// only use this frame it the right number of markers is found in the picture
			if (frame->ObjectCount() == numberMarkers)
			{
				framesDropped = 0;	// set number of subsequent frames dropped to zero

				// get the marker points in 2D in the camera image frame and store them in the list_points2dUnsorted vector
				for (int i = 0; i < numberMarkers; i++)
				{
					cObject *obj = frame->Object(i);
					list_points2dUnsorted[i] = cv::Point2d(obj->X(), obj->Y());
				}

				if (gotOrder == false)
				{
					determineOrder();
				}

				// sort the 2d points with the correct indices as found in the preceeding order determination algorithm
				for (int w = 0; w < numberMarkers; w++)
				{
					list_points2d[w] = list_points2dUnsorted[pointOrderIndices[w]];
				}
				list_points2dOld = list_points2dUnsorted;

				// Now its time to determine the order of the points (or better the 2D-3D correspondences)
				// for that the distance from the new points to the old points is calculated
				// for each point the new index corresponds to the old point with the smallest distance
				// Loop over every point and calculate the min distance to every other point. 
				// Then pick the smallest one and assign its index to the new order pointOrderIndices.
				// (number of markers)^2 possible orders, check every each of them
				for (int j = 0; j < numberMarkers; j++)
				{
					minPointDistance = 5000;
					for (int k = 0; k < numberMarkers; k++)
					{
						// calculate N_2 norm of unsorted points minus old points
						currentPointDistance = norm(list_points2dUnsorted[pointOrderIndices[j]] - list_points2dOld[k]);
						// if the norm is smaller than minPointDistance the correspondence is more likely to be correct
						if (currentPointDistance < minPointDistance)
						{
							// update the array that saves the new point order
							minPointDistance = currentPointDistance;
							pointOrderIndicesNew[j] = k;
						}
					}
				}

				// set the point order to the new value
				for (int k = 0; k < numberMarkers; k++)
				{
					pointOrderIndices[k] = pointOrderIndicesNew[k];
					list_points2d[k] = list_points2dUnsorted[pointOrderIndices[k]];
				}

				// save the unsorted position of the marker points for the next loop
				list_points2dOld = list_points2dUnsorted;

				//Compute the object pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);

				// project the marker 3d points with the solution into the camera image CoSy and calculate difference to true camera image
				projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
				projectionError = norm(list_points2dProjected, list_points2d);

				// get the min and max values from TVec for sanity check
				minMaxLoc(Tvec.at<double>(2), &minValue, &maxValue);

				// sanity check of values. negative z means the marker CoSy is behind the camera, that's not possible.
				if (minValue < 0)
				{
					commObj.addLog("Negative z distance, thats not possible. Start the set zero routine again or restart Programm.\n");
					frame->Release();
					framesDropped++;
					camera->Release();
					closeUDP();
					return 1;
				}

				subtract(posRef, Tvec, position);	// compute the relative object position from the reference position to the current one, given in the camera CoSy
				Mat V = -0.001 * M_HeadingOffset * M_NC.t() * (Mat)position;	// transform the position from the camera CoSy to the ground CoSy with INS alligned heading and into [m]
				position = V;	// position is the result of the preceeding calculation 
				position[2] *= invertZ;	// invert Z if check box in GUI is activated

				// Realtive angle between reference orientation and current orientation
				Rodrigues(Rvec, Rmat);	// compute the rotation matrix from the axis angle respresentation
				Rmat = RmatRef.t() *Rmat;	// the difference of the reference rotation and the current rotation
				//==-- Euler Angles, finally 
				getEulerAngles(Rmat, eulerAngles);	// get the euler angles from the rotation matrix 
				eulerAngles[2] += headingOffset; // add the heading offset to the heading angle

				frameTime = frame->TimeStamp() - timeOld;	// time between the old frame and the current frame
				timeOld = frame->TimeStamp();	// set the old frame time to the current  one
				velocity[0] = (position[0] - positionOld[0]) / frameTime;	// calculate the velocity with finite differences
				velocity[1] = (position[1] - positionOld[1]) / frameTime;	// calculate the velocity with finite differences
				velocity[2] = (position[2] - positionOld[2]) / frameTime;	// calculate the velocity with finite differences
				positionOld = position;	// set the old position to the current one for next frame

				//send position and euler angles else to object over WiFi with 100 Hz
				sendDataUDP(position, eulerAngles);

			}

			// check if the position and euler angles are below the allowed value, if yes send enable to the circuit breaker, if not send shutdown signal
			// absolute x, y and z position in ground CoSy must be smaller than 1.5m
			if (safetyEnable)
			{
				if ((abs(position[0]) < safetyBoxLength && abs(position[1]) < safetyBoxLength && abs(position[2]) < safetyBoxLength))
				{
					// absolute euler angles must be smaller than 30 degrees 
					if ((abs(eulerAngles[0]) < safetyAngle && abs(eulerAngles[1]) < safetyAngle))
					{
						// send the enable signal to the circuit breaker to keep it enabled
						if (v == 5) {
							data.setNum((int)(1));
							udpSocketSafety->write(data);
							v = 0;
						}
					}
					// The euler angles of the object exceeded the allowed euler angles, shut the object down
					else
					{
						data.setNum((int)(0));	// 0 disables the circuit breaker, hence the object
						udpSocketSafety->write(data);
						commObj.addLog("object exceeded allowed Euler Angles, shutting it down!");

					}
				}
				// The position of the object exceeded the allowed position, shut the object down
				else
				{
					data.setNum((int)(0));	// 0 disables the circuit breaker, hence the object
					udpSocketSafety->write(data);
					commObj.addLog("object left allowed Area, shutting it down!");

				}
			}

			// Increase the framesDropped variable if accuracy of tracking is too bad.
			if (projectionError > 5)
			{
				framesDropped++;
			}

			// Inform the user if tracking system is disturbed (marker lost or so)
			if (framesDropped > 10 && safetyEnable)
			{
				data.setNum((int)(0));	// 0 disables the circuit breaker, hence the object
				udpSocketSafety->write(data);
				commObj.addLog("Lost Marker Points or Accuracy was bad!");
				framesDropped = 0;
			}

			// save the values in a log file 
			logfile.open(logName, std::ios::app);
			logfile << frame->TimeStamp() - timeFirstFrame << ";" << position[0] << ";" << position[1] << ";" << position[2] << ";";
			logfile << eulerAngles[0] << ";" << eulerAngles[1] << ";" << eulerAngles[2] << ";";
			logfile << velocity[0] << ";" << velocity[1] << ";" << velocity[2] << "\n";
			logfile.close();

			// rasterize the frame so it can be shown in the GUI
			frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);

			// convert the frame from greyscale as it comes from the camera to rgb color 
			cvtColor(matFrame, cFrame, COLOR_GRAY2RGB);

			// project the marker frame into 2D and save it in the cFrame image
			projectCoordinateFrame(cFrame);

			// project the marker points from 3D to the camera image frame (2d) with the computed pose
			projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2d);
			for (int i = 0; i < numberMarkers; i++)
			{
				// draw a circle around the projected points so the result can be better compared to the real marker position
				circle(cFrame, Point(list_points2d[i].x, list_points2d[i].y), 3, Scalar(225, 0, 0), 3);
			}

			// draw the current position and attitude values as text in the frame
			drawPositionText(cFrame, position, eulerAngles, projectionError);

			// send the new camera picture with infos to the GUI and call the GUI processing routine
			QPixmap QPFrame;
			QPFrame = Mat2QPixmap(cFrame);
			commObj.changeImage(QPFrame);
			QCoreApplication::processEvents();

			// release the camera frame to fetch the new one
			frame->Release();
		}
	}

	// user choose to stop the tracking, clean things up
	closeUDP();	// close the UDP connections so resources are deallocated
	camera->Release();	//== Release camera ==--
	return 0;
}

// Start or stop the camera depending on if the camera is currently running or not
void start_stopCamera()
{
	// tracking is not running so start it
	if (exitRequested)
	{
		exitRequested = false;
		start_camera();
	}
	else  // tracking is currently running, set exitRequest to true so the while loop in start_camera() exits
	{
		exitRequested = true;
	}
}

// determine the initial position of the object that serves as reference point or as ground frame origin
int setZero()
{
	// initialize the variables with starting values
	gotOrder = false;
	posRef = 0;
	eulerRef = 0;
	RmatRef = 0;
	Rvec = RvecOriginal;
	Tvec = TvecOriginal;

	determineExposure();

	ss.str("");
	commObj.addLog("Started Reference Coordinate Determination");

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
	// Set some other parameters as well of the camera
	camera->SetTextOverlay(true);
	camera->SetFrameRate(intFrameRate);
	camera->SetIntensity(intIntensity);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	// sample some frames and calculate the position and attitude. then average those values and use that as zero position
	int numberSamples = 0;
	int numberToSample = 200;
	double projectionError = 0; // difference between the marker points as seen by the camera and the projected marker points with Rvec and Tvec

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

				if (gotOrder == false)
				{
					determineOrder();
				}

				// sort the 2d points with the correct indices as found in the preceeding order determination algorithm
				for (int w = 0; w < numberMarkers; w++)
				{
					list_points2d[w] = list_points2dUnsorted[pointOrderIndices[w]];
				}
				list_points2dOld = list_points2dUnsorted;

				//Compute the pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
				
				// project the marker 3d points with the solution into the camera image CoSy and calculate difference to true camera image
				projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
				projectionError = norm(list_points2dProjected, list_points2d);

				double maxValue = 0;
				double minValue = 0;
				minMaxLoc(Tvec.at<double>(2), &minValue, &maxValue);

				if (maxValue > 10000 || minValue < 0)
				{
					ss.str("");
					ss << "Negative z distance, thats not possible. Start the set zero routine again or restart Programm.\n";
					commObj.addLog(QString::fromStdString(ss.str()));
					frame->Release();
					return 1;
				}

				if (projectionError > 3)
				{
					commObj.addLog("Reprojection Error is bigger than 3 pixel. Correct Marker Configuration Loaded?\nMarker Position measured precisely?");
					frame->Release();
					return 1;
				}

				if (norm(positionOld) - norm(Tvec) < 0.05)	//Iterative Method needs time to converge to solution
				{
					add(posRef, Tvec, posRef);
					add(eulerRef, Rvec, eulerRef); // That are not the values of yaw, roll and pitch yet! Rodriguez has to be called first. 
					numberSamples++;	//==-- one sample more :D
					commObj.progressUpdate(numberSamples * 100 / numberToSample);
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
				drawPositionText(cFrame, position, eulerAngles, projectionError);

				QPixmap QPFrame;
				QPFrame = Mat2QPixmap(cFrame);
				commObj.changeImage(QPFrame);
				QCoreApplication::processEvents();

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
	ss << "====================== RmatRef ========================\n";
	ss << RmatRef << "\n";
	ss << "================= Reference Position ==================\n";
	ss << posRef << "[mm] \n";
	ss << "=============== Reference Euler Angles ================\n";
	ss << eulerRef << "[deg] \n";

	// compute the difference between last obtained TVec and the average Value
	// When it is large the iterative method has not converged properly so it is advised to start the setZero() function once again
	double error = norm(posRef) - norm(Tvec);
	if (error > 5.0)
	{
		ss << "Caution, distance between reference position and last position is: " << error << "\n Start the set zero routine once again.";
	}
	commObj.addLog(QString::fromStdString(ss.str()));
	commObj.progressUpdate(0);
	return 0;
}

// start the camera calibration routine that computes the camera matrix and distortion coefficients
int calibrate_camera()
{
	commObj.addLog("Started camera calibration");
	commObj.addLog("80 pictures are going to be captured");
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

	// Save the obtained calibration coefficients in a file for later use
	QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Calibration File", "", "Calibration File (*.xml);;All Files (*)");
	FileStorage fs(fileName.toUtf8().constData(), FileStorage::WRITE);//+ FileStorage::MEMORY);
	fs << "CameraMatrix" << cameraMatrix;
	fs << "DistCoeff" << distCoeffs;
	fs << "RMS" << rms;
	strBuf = fs.releaseAndGetString();
	commObj.changeStatus(QString::fromStdString(strBuf));
	commObj.addLog("Saved Calibration!");
	return 0;
}

// Load a previously saved calibration from a file
void load_calibration(int method) {

	QString fileName;
	if (method == 0)
	{
		fileName = "calibration.xml";
	}
	else
	{
		fileName = QFileDialog::getOpenFileName(nullptr, "Choose a previous saved calibration file", "", "Calibration Files (*.xml);;All Files (*)");
		if (fileName.length() == 0)
		{
			fileName = "calibration.xml";
		}
	}
	FileStorage fs;
	fs.open(fileName.toUtf8().constData(), FileStorage::READ);
	fs["CameraMatrix"] >> cameraMatrix;
	fs["DistCoeff"] >> distCoeffs;
	commObj.addLog("Loaded calibration from file:");
	commObj.addLog(fileName);
	ss.str("");
	ss << "\nCamera Matrix" << "\n" << cameraMatrix << "\n";
	ss << "\nDistortion Coeff" << "\n" << distCoeffs << "\n";
	commObj.addLog(QString::fromStdString(ss.str()));
}

// project some points from 3D to 2D and then check the accuracy of the algorithms
void test_Algorithm()
{

	int _methodPNP;

	std::vector<Point2d> noise(numberMarkers);

	RvecOriginal = Rvec;
	TvecOriginal = Tvec;

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
		double projectionError = norm(list_points2dProjected, list_points2d);
		putText(cFrame, "Testing Algorithms Finished", cv::Point(5, 420), 1, 1, cv::Scalar(255, 255, 255));
		drawPositionText(cFrame, position, eulerAngles, projectionError);

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

// project a coordinate CoSy with the rotation and translation of the object for visualization
void projectCoordinateFrame(Mat pictureFrame)
{
	projectPoints(coordinateFrame, Rvec, Tvec, cameraMatrix, distCoeffs, coordinateFrameProjected);
	line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[3], Scalar(0, 0, 255), 2); //z-axis
	line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[1], Scalar(255, 0, 0), 2); //x-axis
	line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[2], Scalar(0, 255, 0), 2); //y-axis
}

// open the UDP ports 
void setUpUDP()
{
	// Initialise the QDataStream that stores the data to be send
	QDataStream out(&datagram, QIODevice::WriteOnly);
	out.setVersion(QDataStream::Qt_4_3);

	// Create UDP slots
	commObj.addLog("Opening UDP Ports");
	udpSocketObject = new QUdpSocket(0);
	udpSocketObject->connectToHost(IPAdressObject, portObject);
	commObj.addLog("Opened Object UDP Port");

	udpSocketSafety = new QUdpSocket(0);
	udpSocketSafety2 = new QUdpSocket(0);

	// if the safety feature is activated open the udp port
	if (safetyEnable)
	{
		udpSocketSafety->connectToHost(IPAdressSafety, portSafety);
		commObj.addLog("Opened Safety UDP Port");
	}

	// if the second receiver feature is activated open the udp port
	if (safety2Enable)
	{
		udpSocketSafety2->connectToHost(IPAdressSafety2, portSafety2);
		commObj.addLog("Opened Second Receiver UDP Port");
	}
}

// Add a heading offset to the attitude for the case it is necessary
void setHeadingOffset(double d)
{
	headingOffset = d;
	d = d * 3.141592653589 / 180.0; // Convert heading offset from degrees to rad

	// Calculate rotation about x axis
	Mat R_x = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
		);

	// Calculate rotation about y axis
	Mat R_y = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
		);

	// Calculate rotation about z axis
	Mat R_z = (Mat_<double>(3, 3) <<
		cos(d), -sin(d), 0,
		sin(d), cos(d), 0,
		0, 0, 1);


	// Combined rotation matrix
	M_HeadingOffset = R_z * R_y * R_x;
}

// send the position and attitude over UDP to every receiver, the safety receiver is handled on its own in the start_camera function
void sendDataUDP(cv::Vec3d &Position, cv::Vec3d &Euler)
{
	datagram.clear();
	out << (float)Position[0] << (float)Position[1] << (-1 * (float)Position[2]);
	out << (float)Euler[0] << (float)Euler[1] << (float)Euler[2]; // Roll Pitch Heading
	udpSocketObject->writeDatagram(datagram, IPAdressObject, portObject);

	// if second receiver is activated send it also the tracking data
	if (safety2Enable)
	{
		udpSocketSafety2->writeDatagram(datagram, IPAdressSafety2, portSafety2);
	}

}

// open the documentation of this software in the system interner browser
void show_Help()
{
	// Get the current directory as the help html file is stored in the same directory
	wchar_t buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	std::wstring strBuffer(buffer);

	// Delete the last entry of the path since it is the file name of the executable
	int pos = strBuffer.find_last_of(L"\\ / ");
	std::wstring strPath = strBuffer.substr(0, pos);
	QString qtString = QString::fromWCharArray(strPath.c_str());

	// append help.html to the path since this is the documentation in html format
	QString qtStrFile = "\\help.html";
	QString file = qtString + qtStrFile;

	// open the documentation help file in the standard browser
	QDesktopServices::openUrl(QUrl::fromLocalFile(file));
}

// close the UDP ports again to release network interfaces etc. 
void closeUDP()
{
	// check if the socket is open and if yes close it
	if (udpSocketObject->isOpen())
	{
		udpSocketObject->close();
	}

	if (udpSocketSafety->isOpen())
	{
		udpSocketSafety->close();
	}

	if (udpSocketSafety2->isOpen())
	{
		udpSocketSafety2->close();
	}
}

// load a marker configuration from file. This has to be created by hand, use the standard marker configuration file as template
void loadMarkerConfig(int method)
{
	QString fileName;
	// during start up of the programm load the standard marker configuration
	if (method == 0)
	{
		// open the standard marker configuration file
		FileStorage fs;
		fs.open("marker_standard.xml", FileStorage::READ);

		// copy the values to the respective variables
		fs["numberMarkers"] >> numberMarkers;

		// inizialise vectors with correct length depending on the number of markers
		list_points3d = std::vector<Point3d>(numberMarkers);
		list_points2d = std::vector<Point2d>(numberMarkers);
		list_points2dOld = std::vector<Point2d>(numberMarkers);
		list_points2dDifference = std::vector<double>(numberMarkers);
		list_points2dProjected = std::vector<Point2d>(numberMarkers);
		list_points2dUnsorted = std::vector<Point2d>(numberMarkers);

		// save the marker locations in the points3d vector
		fs["list_points3d"] >> list_points3d;
		fs.release();
		commObj.addLog("Loaded marker configuration from file:");
		commObj.addLog(fileName);



	}
	else
	{
		// if the load marker configuration button was clicked show a open file dialog 
		fileName = QFileDialog::getOpenFileName(nullptr, "Choose a previous saved marker configuration file", "", "marker configuratio files (*.xml);;All Files (*)");

		// was cancel or abort clicked 
		if (fileName.length() == 0)
		{
			// if yes load the standard marker configuration
			fileName = "marker_standard.xml";
		}

		// open the selected marker configuration file
		FileStorage fs;
		fs.open(fileName.toUtf8().constData(), FileStorage::READ);

		// copy the values to the respective variables
		fs["numberMarkers"] >> numberMarkers;

		// inizialise vectors with correct length depending on the number of markers
		list_points3d = std::vector<Point3d>(numberMarkers);
		list_points2d = std::vector<Point2d>(numberMarkers);
		list_points2dOld = std::vector<Point2d>(numberMarkers);
		list_points2dDifference = std::vector<double>(numberMarkers);
		list_points2dProjected = std::vector<Point2d>(numberMarkers);
		list_points2dUnsorted = std::vector<Point2d>(numberMarkers);

		// save the marker locations in the points3d vector
		fs["list_points3d"] >> list_points3d;
		fs.release();
		commObj.addLog("Loaded marker configuration from file:");
		commObj.addLog(fileName);

	}

	// Print out the number of markers and their position to the GUI
	ss.str("");
	ss << "Number of Markers: " << numberMarkers << "\n";
	ss << "Marker 3D Points X,Y and Z [mm]: \n";
	for (int i = 0; i < numberMarkers; i++)
	{
		ss << "Marker " << i + 1 << ":\t" << list_points3d[i].x << "\t" << list_points3d[i].y << "\t" << list_points3d[i].z << "\n";
	}
	commObj.addLog(QString::fromStdString(ss.str()));

	// check if P3P algorithm can be enabled, it needs exactly 4 marker points to work
	if (numberMarkers == 4)
	{
		// if P3P is possible, let the user choose which algorithm he wants but keep iterative active
		methodPNP = 0;
		commObj.enableP3P(true);
	}
	else
	{
		// More (or less) marker than 4 loaded, P3P is not possible, hence user cant select P3P in GUI
		methodPNP = 0;
		commObj.enableP3P(false);
		commObj.addLog("P3P Algorithm disabled, only works with 4 markers");
	}

	// now display the marker configuration in the camera view 
	Mat cFrame(480, 640, CV_8UC3, Scalar(0, 0, 0));

	// Set the camera pose parallel to the marker coordinate system
	Tvec.at<double>(0) = 0;
	Tvec.at<double>(1) = 0;
	Tvec.at<double>(2) = 4500;
	Rvec.at<double>(0) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(1) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(2) = -90. * 3.141592653589 / 180.0;

	projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
	for (int i = 0; i < numberMarkers; i++)
	{
		circle(cFrame, Point(list_points2dProjected[i].x, list_points2dProjected[i].y), 3, Scalar(255, 0, 0), 3);
	}

	projectCoordinateFrame(cFrame);
	QPixmap QPFrame;
	QPFrame = Mat2QPixmap(cFrame);
	commObj.changeImage(QPFrame);
	QCoreApplication::processEvents();

}

// draw the position and attitude in the picture 
void drawPositionText(cv::Mat &Picture, cv::Vec3d & Position, cv::Vec3d & Euler, double error)
{
	ss.str("");
	ss << "X: " << Position[0] << " m";
	putText(Picture, ss.str(), cv::Point(200, 440), 1, 1, cv::Scalar(255, 255, 255));

	ss.str("");
	ss << "Y: " << Position[1] << " m";
	putText(Picture, ss.str(), cv::Point(200, 455), 1, 1, cv::Scalar(255, 255, 255));

	ss.str("");
	ss << "Z: " << Position[2] << " m";
	putText(Picture, ss.str(), cv::Point(200, 470), 1, 1, cv::Scalar(255, 255, 255));

	ss.str("");
	ss << "Heading: " << Euler[2] << " deg";
	putText(Picture, ss.str(), cv::Point(350, 440), 1, 1, cv::Scalar(255, 255, 255));

	ss.str("");
	ss << "Pitch: " << Euler[1] << " deg";
	putText(Picture, ss.str(), cv::Point(350, 455), 1, 1, cv::Scalar(255, 255, 255));

	ss.str("");
	ss << "Roll: " << Euler[0] << " deg";
	putText(Picture, ss.str(), cv::Point(350, 470), 1, 1, cv::Scalar(255, 255, 255));

	ss.str("");
	ss << "Error: " << error << " px";
	putText(Picture, ss.str(), cv::Point(10, 470), 1, 1, cv::Scalar(255, 255, 255));
}

// load the rotation matrix from camera CoSy to ground CoSy
// It is determined during ground calibration and once the camera is mounted and fixed stays the same
void loadCameraPosition()
{
	// Open the referenceData.xml that contains the rotation from camera CoSy to ground CoSy
	FileStorage fs;
	fs.open("referenceData.xml", FileStorage::READ);
	fs["M_NC"] >> M_NC;
	fs["M_NC"] >> RmatRef;
	fs["posRef"] >> posRef;
	fs["eulerRef"] >> eulerRef;
	commObj.addLog("Loaded Reference Data!");
}

// get the optimal exposure for the camera
int determineExposure()
{
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

	camera->SetVideoType(Core::PrecisionMode);	// set the camera mode to precision mode, it used greyscale imformation for marker property calculations

												//== Start camera output ==--
	camera->Start();

	//== Turn on some overlay text so it's clear things are     ===---
	//== working even if there is nothing in the camera's view. ===---
	camera->SetTextOverlay(true);
	camera->SetExposure(intExposure);	// set the camera exposure
	camera->SetIntensity(intIntensity);	// set the camera infrared LED intensity
	camera->SetFrameRate(intFrameRate);	// set the camera framerate to 100 Hz
	camera->SetIRFilter(true);	// enable the filter that blocks visible light and only passes infrared light
	camera->SetHighPowerMode(true);	// enable high power mode of the leds
	camera->SetContinuousIR(false);	// enable continuous LED light
	camera->SetThreshold(intThreshold);	// set threshold for marker detection

	//set exposure such that num markers are visible
	int numberObjects = 0;	// Number of objects (markers) found in the current picture with the given exposure	
	int minExposure = 1;	// exposure when objects detected the first time is numberMarkers 
	int maxExposure = 480;	// exposure when objects detected is first time numberMarkers+1
	intExposure = minExposure;	// set the exposure to the smallest value possible
	int numberTries = 0;	// if the markers arent found after numberTries then there might be no markers at all in the real world

							// Determine minimum exposure, hence when are numberMarkers objects detected
	camera->SetExposure(intExposure);
	while (numberObjects != numberMarkers && numberTries < 48)
	{
		// get a new camera frame
		Frame *frame = camera->GetFrame();
		if (frame) // frame received
		{
			numberObjects = frame->ObjectCount();	// how many objects are detected in the image
			if (numberObjects == numberMarkers) { minExposure = intExposure; frame->Release(); break; } // if the right amount if markers is found, exit while loop
			// not the right amount of markers was found so increase the exposure and try again
			numberTries++;
			intExposure += 10;
			camera->SetExposure(intExposure);
			ss.str("");
			ss << "Exposure: " << intExposure << "\t";
			ss << "Objects found: " << numberObjects;
			commObj.addLog(QString::fromStdString(ss.str()));
			frame->Release();
		}
	}

	// Now determine maximum exposure, hence when are numberMarkers+1 objects detected
	numberTries = 0;	// if the markers arent found after numberTries then there might be no markers at all in the real world
	intExposure = maxExposure;
	camera->SetExposure(intExposure);
	numberObjects = 0;
	while (numberObjects != numberMarkers && numberTries < 48)
	{
		Frame *frame = camera->GetFrame();
		if (frame)
		{
			numberObjects = frame->ObjectCount(); // how many objects are detected in the image
			if (numberObjects == numberMarkers) { maxExposure = intExposure; frame->Release(); break; } // if the right amount if markers is found, exit while loop

			// not the right amount of markers was found so decrease the exposure and try again
			intExposure -= 10;
			numberTries++;
			camera->SetExposure(intExposure);
			ss.str("");
			ss << "Exposure: " << intExposure << "\t";
			ss << "Objects found: " << numberObjects;
			commObj.addLog(QString::fromStdString(ss.str()));
			frame->Release();
		}
	}

	// set the exposure to the mean of min and max exposure determined
	camera->SetExposure((minExposure + maxExposure) / 2.0);

	// and now check if the correct amount of markers is detected with that new value
	while (1)
	{
		Frame *frame = camera->GetFrame();
		if (frame)
		{
			numberObjects = frame->ObjectCount(); // how many objects are detected in the image
			if (numberObjects != numberMarkers) // are all markers and not more or less detected in the image
			{
				frame->Release();
				commObj.addLog("Was not able to detect the right amount of markers");
				//== Release camera ==--
				camera->Release();
				return 1;
			}
			else  // all markers and not more or less are found
			{
				frame->Release();
				intExposure = (minExposure + maxExposure) / 2.0;
				commObj.addLog("Found the correct number of markers");
				commObj.addLog("Exposure set to:");
				commObj.addLog(QString::number(intExposure));
				break;
			}
		}
	}

	camera->Release();
	return 0;

}

void determineOrder()
{
	// determine the 3D-2D correspondences that are crucial for the PnP algorithm
	// Try every possible correspondence and solve PnP
	// Then project the 3D marker points into the 2D camera image and check the difference 
	// between projected points and points as seen by the camera
	// the corresponce with the smallest difference is probably the correct one
	
		// the difference between true 2D points and projected points is super big
		minPointDistance = 5000;
		std::sort(pointOrderIndices, pointOrderIndices + 4);

		// now try every possible permutation of correspondence
		do {
			// reset the starting values for solvePnP
			Rvec = RvecOriginal;
			Tvec = TvecOriginal;

			// sort the 2d points with the current permutation
			for (int m = 0; m < numberMarkers; m++)
			{
				list_points2d[m] = list_points2dUnsorted[pointOrderIndices[m]];
			}

			// Call solve PNP with P3P since its more robust and sufficient for start value determination
			solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, SOLVEPNP_P3P);

			// set the current difference of all point correspondences to zero
			currentPointDistance = 0;

			// project the 3D points with the solvePnP solution onto 2D
			projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);

			// now compute the absolute difference (error)
			for (int n = 0; n < numberMarkers; n++)
			{
				currentPointDistance += norm(list_points2d[n] - list_points2dProjected[n]);
			}

			// if the difference with the current permutation is smaller than the smallest value till now
			// it is probably the more correct permutation
			if (currentPointDistance < minPointDistance)
			{
				minPointDistance = currentPointDistance;	// set the smallest value of difference to the current one
				for (int b = 0; b < numberMarkers; b++)	// now safe the better permutation
				{
					pointOrderIndicesNew[b] = pointOrderIndices[b];
				}
			}


		}
		// try every permutation 
		while (std::next_permutation(pointOrderIndices, pointOrderIndices + 4));

		// now that the correct order is found assign it to the indices array
		for (int w = 0; w < numberMarkers; w++)
		{
			pointOrderIndices[w] = pointOrderIndicesNew[w];
		}
		gotOrder = true;
}

int calibrateGround()
{
	// initialize the variables with starting values
	gotOrder = false;
	posRef = 0;
	eulerRef = 0;
	RmatRef = 0;
	Rvec = RvecOriginal;
	Tvec = TvecOriginal;

	determineExposure();

	ss.str("");
	commObj.addLog("Started Ground Calibration");

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
	// Set some other parameters as well of the camera
	camera->SetTextOverlay(true);
	camera->SetFrameRate(intFrameRate);
	camera->SetIntensity(intIntensity);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	// sample some frames and calculate the position and attitude. then average those values and use that as zero position
	int numberSamples = 0;
	int numberToSample = 200;
	double projectionError = 0; 

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

				if (gotOrder == false)
				{
					determineOrder();
				}

				// sort the 2d points with the correct indices as found in the preceeding order determination algorithm
				for (int w = 0; w < numberMarkers; w++)
				{
					list_points2d[w] = list_points2dUnsorted[pointOrderIndices[w]];
				}
				list_points2dOld = list_points2dUnsorted;

				//Compute the pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
				
				// project the marker 3d points with the solution into the camera image CoSy and calculate difference to true camera image
				projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
				projectionError = norm(list_points2dProjected, list_points2d);
				
				if (projectionError > 3)
				{
					commObj.addLog("Reprojection Error is bigger than 3 pixel. Correct Marker Configuration Loaded?\nMarker Position measured precisely?");
					frame->Release();
					return 1;
				}

				double maxValue = 0;
				double minValue = 0;
				minMaxLoc(Tvec.at<double>(2), &minValue, &maxValue);

				if (maxValue > 10000 || minValue < 0)
				{
					
				
					commObj.addLog("Negative z distance, thats not possible. Start the set zero routine again and check marker configurations.");
					frame->Release();
					return 1;
				}

				if (norm(positionOld) - norm(Tvec) < 0.05)	//Iterative Method needs time to converge to solution
				{
					add(posRef, Tvec, posRef);
					add(eulerRef, Rvec, eulerRef); // That are not the values of yaw, roll and pitch yet! Rodriguez has to be called first. 
					numberSamples++;	//==-- one sample more :D
					commObj.progressUpdate(numberSamples * 100 / numberToSample);
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

				QPixmap QPFrame;
				QPFrame = Mat2QPixmap(cFrame);
				commObj.changeImage(QPFrame);
				QCoreApplication::processEvents();

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
	ss << "====================== RmatRef ========================\n";
	ss << RmatRef << "\n";
	ss << "================= Reference Position ==================\n";
	ss << posRef << "[mm] \n";
	ss << "=============== Reference Euler Angles ================\n";
	ss << eulerRef << "[deg] \n";

	// Save the obtained calibration coefficients in a file for later use
	QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Ground Calibration File", "referenceData.xml", "Calibration File (*.xml);;All Files (*)");
	FileStorage fs(fileName.toUtf8().constData(), FileStorage::WRITE);//+ FileStorage::MEMORY);
	fs << "posRef" << posRef;
	fs << "M_NC" << RmatRef;
	fs << "eulerRef" << eulerRef;
	strBuf = fs.releaseAndGetString();
	commObj.changeStatus(QString::fromStdString(strBuf));
	commObj.addLog("Saved Ground Calibration!");
	commObj.progressUpdate(0);
	return 0;
}
