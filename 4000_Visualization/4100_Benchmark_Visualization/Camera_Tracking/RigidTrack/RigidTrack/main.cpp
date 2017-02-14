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
#include <random>

#include "main.h"
#include "communication.h"

using namespace CameraLibrary;
using namespace cv;

commObject commObj;

bool debug = false;

double frameTime = 0.01; // 100 Hz frame rate
double timeOld = 0.0;		// old time for finite differences velocity calculation

Vec3d position = Vec3d();	// position vector x,y,z for drone position in O-frame
Vec3d WGS84 = Vec3d();		// WGS vector, latitude, longitude and height 
Vec3d eulerAngles = Vec3d(); // Roll Pitch Heading in this order
Vec3d positionOld = Vec3d();	// old position in O-frame for finite differences velocity calculation 
Vec3d velocity = Vec3d();	// velocity vector of drone in o-frame in respect to o-frame
Vec3d posRef = Vec3d();		// zero position of drone in camera frame
Vec3d eulerRef = Vec3d();	// euler angle of drone respectivley to camera frame
Vec3d ropePosition = Vec3d();	// point where the rope is attached or redirected in o-frame

double ropeLength = 0;	// nominal rope length, hence the distance from the rope redirection point to the drone position
double latitudeRef = 47;	// latitude reference for flat earth WGS84 calculation, corresponds to munich
double longitudeRef = 11;	// longitude reference for flat earth WGS84 calculation, corresponds to munich
double heightRef = 0;	//WGS84 reference height
double latitude = 47;	// actual WGS84 latitude sent to drone
double longitude = 11;	// actual WGS84 longitude sent to drone
double height = 0;	// actual WGS84 height sent to drone
double earthRadius = 6366743.0; // Radius of the Earth at 47° North in Meters

std::ofstream logfile;	// file handler for writing the log file

int intIntensity = 6; // max infra red spot light intensity is 15 1-6 is strobe 7-15 is continuous 13 and 14 are meaningless 
int intExposure = 100; // max is 480 increase if markers are badly visible
int intFrameRate = 100;	// frame rate of camera, maximum is 100 fps
int intThreshold = 200;	// threshold value for marker detection. If markers are badly visible lower this value

//== Rotation, translation etc. matrix for PnP results
Mat Rmat = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	// rotation matrix from camera frame to marker frame
Mat RmatRef = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	// reference rotation matrix from camera frame to marker frame
Mat M_NC = cv::Mat_<double>(3, 3);							// rotation matrix from camera to ground
Mat Rvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	// rotation vector (axis-angle notation) from camera frame to marker frame
Mat Tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);	// translation vector from camera frame to marker frame in camera frame
Mat RvecOriginal;	// initial values as start values for algorithms
Mat TvecOriginal;	// initial values as start values for algorithms

float Value[100] = { 0 };	//100 values that are sent via MMF, can be more but should be enough for now

bool useGuess = true; // set to true and the algorithm uses the last result as starting value
int methodPNP = 0; // solvePNP algorithm 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // 4 and 1 are the same

int numberMarkers = 4; // number of markers
std::vector<Point3d> list_points3d;	// marker positions in marker frame 
std::vector<Point2d> list_points2d;	// marker positions projected in 2D in camera image frame
std::vector<Point2d> list_points2dOld;	// marker positions of previous frame in 2D in camera image frame
std::vector<double> list_points2dDifference;	// difference of the old and new 2D marker position to determine the order of the points
std::vector<Point2d> list_points2dProjected;	// 3D marker points projected to 2D in camera image frame with the algorithm projectPoints
std::vector<Point2d> list_points2dUnsorted;	// marker points in 2D camera image frame, sorted with increasing x (camera image frame) but not sorted to correspond with list_points3d
std::vector<Point3d> coordinateFrame;	// coordinate visualisazion of marker frame
std::vector<Point2d> coordinateFrameProjected;	// marker frame projected from 3D to 2D camera image frame
int pointOrderIndices[] = { 0, 1, 2, 3 };	// old correspondence from list_points3d and list_points_2d
int pointOrderIndicesNew[] = { 0, 1, 2, 3 };	// new correspondence from list_points3d and list_points_2d
double currentPointDistance = 5000;	// distance from the projected 3D points (hence in 2d) to the real 2d marker positions in camera image frame 
double minPointDistance = 5000;	// minimum distance from the projected 3D points (hence in 2d) to the real 2d marker positions in camera image frame 
int currentMinIndex = 0;	// helper variable set to the point order that holds the current minimum point distance 
bool gotOrder = false;	// order of the list_points3d and list_points3d already tetermined or not 

int decimator = 1; // Decimate the velocity frequency from 100Hz to 100Hz

Mat cameraMatrix;	// camera matrix of the camera
Mat distCoeffs;	// distortion coefficients of the camera
Core::DistortionModel distModel;	// distortion model of the camera

// IP adress of the circuit breaker that disables the drone if a specified region is exited. 
QUdpSocket *udpSocketCB;	// socket for the communication with the circuit breaker
QUdpSocket *udpSocketDrone;	// socket for the communication with the drone
QUdpSocket *udpSocketWinch;	// socket for the communication with the rope winch
QHostAddress IPAdressCB = QHostAddress("192.168.4.1"); // IPv4 adress of the circuit breaker, stays the same
QHostAddress IPAdressDrone = QHostAddress("192.168.4.2");	// IPv4 adress of the drone wifi telemetry chip, can change to 192.168.4.x. This is where the position etc is sent to.
QHostAddress IPAdressWinch = QHostAddress("192.168.4.4");	// IPv4 adress of the rope winch,
QByteArray datagram;	// data package that is sent to the drone 
QByteArray data;	// data package that's sent to the circuit breaker
QDataStream out;	// stream that sends the datagram package via UDP

const int BACKBUFFER_BITSPERPIXEL = 8;	// 8 bit per pixel and greyscale image from camera
std::string strBuf;	// buffer that holds the strings that are sent to the Qt GUI
std::stringstream ss;	// stream that sends the strBuf buffer to the Qt GUI

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

	// inizialise vectors with correct length
	list_points3d = std::vector<Point3d>(numberMarkers);
	list_points2d = std::vector<Point2d>(numberMarkers);
	list_points2dOld = std::vector<Point2d>(numberMarkers);
	list_points2dDifference = std::vector<double>(numberMarkers);
	list_points2dProjected = std::vector<Point2d>(numberMarkers);
	list_points2dUnsorted = std::vector<Point2d>(numberMarkers);
	coordinateFrame = std::vector<Point3d>(numberMarkers);
	coordinateFrameProjected = std::vector<Point2d>(numberMarkers);

	// coordinates of the markers the marker frame
	list_points3d[0] = cv::Point3d(227.1, 0.0, -21.5);
	list_points3d[1] = cv::Point3d(-66.5, 920.0, -23.0);
	list_points3d[2] = cv::Point3d(-157.6, 500.0, -8.6);
	list_points3d[3] = cv::Point3d(-720.0, 0.0, -13.0);

	// initial guesses for position and rotation, important for Iterative Method!
	Tvec.at<double>(0) = 45;
	Tvec.at<double>(1) = 45;
	Tvec.at<double>(2) = 4500;
	Rvec.at<double>(0) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(1) = 0 * 3.141592653589 / 180.0;
	Rvec.at<double>(2) = -45 * 3.141592653589 / 180.0;

	// Points that make up the marker frame axis system, hence one line in each axis direction
	coordinateFrame[0] = cv::Point3d(0, 0, 0);
	coordinateFrame[1] = cv::Point3d(300, 0, 0);
	coordinateFrame[2] = cv::Point3d(0, 300, 0);
	coordinateFrame[3] = cv::Point3d(0, 0, 300);

	ropePosition = cv::Point3d(0, 0, -4500); // the rope is redirected 4.5m above the o-frame
	ropeLength = ropePosition[2];	// set the rope length to initial value

	test_Algorithm();	// test the algorithms and their accuracy 

	setUpUDP();	// open sockets and ports for UDP communication
	WGS84[0] = latitudeRef;	// latitude in WGS84 set to reference latitude
	WGS84[1] = longitudeRef; // longitude in WGS84 set to reference longitude
	WGS84[2] = heightRef;	// height in WGS84 set to reference height

	velocity[0] = 0.1;	// set velocity initial values
	velocity[1] = 0.2;	// set velocity initial values
	velocity[2] = 0.3;	// set velocity initial values

	eulerAngles[0] = 1.0;	// set initial euler angles to arbitrary values for testing
	eulerAngles[1] = 1.1;	// set initial euler angles to arbitrary values for testing
	eulerAngles[2] = 1.2;	// set initial euler angles to arbitrary values for testing

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

// start the loop that fetches frames, computes the position etc and sends it to the drone and CB
int start_camera() {

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

	//helper variables used to print ouput only every 30th time and kick circuit breaker
	int u = 0;
	int v = 0;
	int decimatorHelper = 0;
	int framesDropped = 0; // if a marker is not visible or accuracy is bad increase this counter.
	double projectionError = 0; // equals the quality of the tracking

	// enable circuit breaker, hence send a 9 and then a 1 to it
	data.setNum((int)(9));
	udpSocketCB->write(data);
	data.setNum((int)(1));
	udpSocketCB->write(data);

	// now enter the main loop that processes each frame
	while (1)
	{
		//== Fetch a new frame from the camera ===---
		Frame *frame = camera->GetFrame();

		if (frame)
		{
			framesDropped++;
			u++;  // helper variable to print data every x-frame
			v++;  // helper variable to print data every x-frame
			decimatorHelper++;  // helper variable for decimator and udp send
			//== Ok, we've received a new frame, lets do something
			//== with it.

			// only use this frame it the right number of markers is found in the picture or debug is on
			if (frame->ObjectCount() == numberMarkers || debug)
			{
				framesDropped = 0;	// set number of subsequent frames dropped to zero
				// get the marker points in 2D in the camera image frame and store them in the list_points2dUnsorted vector
				for (int i = 0; i < numberMarkers; i++)
				{
					cObject *obj = frame->Object(i);
					list_points2dUnsorted[i] = cv::Point2d(obj->X(), obj->Y());
				}

				// save the marker positions in a file for debug and test purposes
				if (debug == true)
				{
					logfile.open("markerPoints.txt", std::ios::app);
					logfile << list_points2dUnsorted[0] << ";" << list_points2dUnsorted[1] << ";" << list_points2dUnsorted[2] << ";" << list_points2dUnsorted[3] << "\n";
					logfile.close();
				}

				// Now its time to determine the order of the points
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

				//Compute the drone pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);

				// project the marker 3d points with the solution into the camera image frame and calculate difference to true camera image
				projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
				projectionError = norm(list_points2dProjected, list_points2d);

				double maxValue = 0;
				double minValue = 0;
				minMaxLoc(Tvec.at<double>(2), &minValue, &maxValue);

				// sanity check of values. negative z means the marker frame is behind the camera, that's not possible. And usually the marker frame is nearer than 10000mm
				if ((maxValue > 10000 || minValue < 0) && debug == false)
				{
					commObj.addLog("Negative z distance, thats not possible. Start the set zero routine again or restart Programm.\n");
					frame->Release();
					framesDropped++;
				}

				subtract(posRef, Tvec, position);	// compute the relative drone position from the reference position to the current one, given in the camera frame
				Mat V = -1 * M_NC.t() * (Mat)position;	// transform the position from the camera frame to the ground frame 
				position = V;	// position is the result of the preceeding calculation

				// Realtive angle between reference orientation and current orientation
				Rodrigues(Rvec, Rmat);	// compute the rotation matrix from the axis angle respresentation
				Rmat = RmatRef.t() *Rmat;	// the difference of the reference rotation and the current rotation
				//==-- Euler Angles, finally 
				getEulerAngles(Rmat, eulerAngles);	// get the euler angles from the rotation matrix 

				frameTime = frame->TimeStamp() - timeOld;	// time between the old frame and the current frame
				timeOld = frame->TimeStamp();	// set the old frame time to the current  one
				velocity[0] = (position[0] - positionOld[0]) / frameTime;	// calculate the velocity with finite differences
				velocity[1] = (position[1] - positionOld[1]) / frameTime;	// calculate the velocity with finite differences
				velocity[2] = (position[2] - positionOld[2]) / frameTime;	// calculate the velocity with finite differences
				positionOld = position;	// set the old position to the current one for next frame
				velocity *= 0.001;	// convert from mm/s to m/s

				latitude = latitudeRef + atan(Value[0] / earthRadius);	// calculate the current latitude in WGS84 
				longitude = longitudeRef + atan(Value[1] / earthRadius);	// calculate the current lon in WGS84 

				//send position and everything else to drone over WiFi with 100 Hz
				if (decimatorHelper >= decimator) {
					sendDataUDPDrone(latitude, longitude, position[2], velocity, eulerAngles);
					decimatorHelper = 0;
				}
			}

			// send the new rope length to the winch
			ropeLength = norm(ropePosition - position);
			data.setNum((int)(ropeLength));
			udpSocketWinch->write(data);

			// check if the position and euler angles are below the allowed value, if yes send enable to the circuit breaker, if not send shutdown signal
			// absolute x, y and z position in ground frame must be smaller than 1.5m
			if ((abs(position[0]) < 1500 && abs(position[1]) < 1500 && abs(position[2]) < 1500) || debug == true)
			{
				// absolute euler angles must be smaller than 30 degrees 
				if ((abs(eulerAngles[0]) < 30 && abs(eulerAngles[1]) < 30) || debug == true)
				{
					// send the enable signal to the circuit breaker to keep it enabled
					if (v == 5) {
						data.setNum((int)(1));
						udpSocketCB->write(data);
						v = 0;
					}
				}
				// The euler angles of the drone exceeded the allowed euler angles, shut the drone down
				else
				{
					data.setNum((int)(0));	// 0 disables the circuit breaker, hence the drone
					udpSocketCB->write(data);
					commObj.addLog("Drone exceeded allowed Euler Angles, shutting it down!");

				}
			}
			// The position of the drone exceeded the allowed position, shut the drone down
			else
			{
				data.setNum((int)(0));	// 0 disables the circuit breaker, hence the drone
				udpSocketCB->write(data);
				commObj.addLog("Drone left allowed Area, shutting it down!");

			}

			// Increase the framesDropped variable if accuracy of tracking is too bad.
			if (projectionError > 50 && debug == false)
			{
				framesDropped++;
			}

			//Stop the drone is tracking system is disturbed (marker lost or so)
			if (framesDropped > 10 && debug == false)
			{
				data.setNum((int)(0));	// 0 disables the circuit breaker, hence the drone
				udpSocketCB->write(data);
				commObj.addLog("Lost Marker Points or Accuracy was bad!");
			}

			// Output every second if debug is true. This can slow down the whole programm and introduce spikes or lags in the measurements 
			if (u == 100 && debug) {
				ss.str("");
				ss << "X      =  " << position[0] << "\tY    =  " << position[1] << "\tZ     = " << position[2] << "\n";
				ss << "VX     =  " << velocity[0] << "\tVY   =  " << velocity[1] << "\tVZ    = " << velocity[2] << "\n";
				ss << "roll  =  " << eulerAngles[0] << "\t pitch  =  " << eulerAngles[1] << "\t heading  = " << eulerAngles[2];
				commObj.addLog(QString::fromStdString(ss.str()));	// send the string to the GUI 
				u = 0;
			}

			// save the values in a log file 
			logfile.open("logData.txt", std::ios::app);
			logfile << frame->TimeStamp() << ";" << position[0] << ";" << position[1] << ";" << position[2] << ";";
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

			QPixmap QPFrame;
			QPFrame = Mat2QPixmap(cFrame);
			commObj.changeImage(QPFrame);
			QCoreApplication::processEvents();
			frame->Release();
		}
	}

	//== Release camera ==--
	camera->Release();

	//== Shutdown Camera Library ==--
	CameraManager::X().Shutdown();

	//== Exit the application.  Simple! ==--
	return 0;
}

// determine the initial position of the drone that serves as reference point or as ground frame origin
int setZero()
{
	posRef = 0;
	eulerRef = 0;
	RmatRef = 0;
	Rvec = RvecOriginal;
	Tvec = TvecOriginal;
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

	camera->SetTextOverlay(true);
	camera->SetExposure(intExposure);
	camera->SetFrameRate(intFrameRate);
	camera->SetIntensity(intIntensity);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	int numberSamples = 0;
	int numberToSample = 100;

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


				if (gotOrder == false)
				{
					minPointDistance = 5000;
					std::sort(pointOrderIndices, pointOrderIndices + 4);
					do {
						Rvec = RvecOriginal;
						Tvec = TvecOriginal;
						for (int m = 0; m < numberMarkers; m++)
						{
							list_points2d[m] = list_points2dUnsorted[pointOrderIndices[m]];
						}

						solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
						double maxValue = 0;
						double minValue = 0;
						currentPointDistance = 0;
						minMaxLoc(Tvec, &minValue, &maxValue);
						if (maxValue < 5000 && minValue > -5000)
						{
							projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2dProjected);
							for (int n = 0; n < numberMarkers; n++)
							{
								currentPointDistance += norm(list_points2d[n] - list_points2dProjected[n]);
							}

							if (currentPointDistance < minPointDistance)
							{
								minPointDistance = currentPointDistance;
								for (int b = 0; b < numberMarkers; b++)
								{
									pointOrderIndicesNew[b] = pointOrderIndices[b];
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

				for (int w = 0; w < numberMarkers; w++)
				{
					list_points2d[w] = list_points2dUnsorted[pointOrderIndices[w]];
				}
				list_points2dOld = list_points2dUnsorted;

				//Compute the pose from the 3D-2D corresponses
				solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);

				double maxValue = 0;
				double minValue = 0;
				minMaxLoc(Tvec.at<double>(2), &minValue, &maxValue);

				if (maxValue > 10000 || minValue < 0)
				{
					//ss.str("");
					//ss << "Negative z distance, thats not possible. Start the set zero routine again or restart Programm.\n";
					//commObj.addLog(QString::fromStdString(ss.str()));
					//frame->Release();
					//
					//return 1;
				}
				if (norm(positionOld) - norm(Tvec) < 0.05)	//Iterative Method needs time to converge to solution
				{
					add(posRef, Tvec, posRef);
					add(eulerRef, Rvec, eulerRef); // That are not the values of yaw, roll and pitch yet! Rodriguez has to be called first. 
					numberSamples++;	//==-- one sample more :D
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
	ss << "=============== Point Order ================\n";
	ss << pointOrderIndices[0] << pointOrderIndices[1] << pointOrderIndices[2] << pointOrderIndices[3] << "\n";
	double error = norm(posRef) - norm(Tvec);
	if (error > 5.0)
	{
		ss << "Caution, distance between reference position and last position is: " << error << "\n Start the set zero routine once again.";
	}
	commObj.addLog(QString::fromStdString(ss.str()));

	//FileStorage fs("referenceData.xml", FileStorage::WRITE);//+ FileStorage::MEMORY);
	//fs << "RmatRef" << RmatRef;
	//fs << "posRef" << posRef;
	//fs << "eulerRef" << eulerRef;
	//strBuf = fs.releaseAndGetString();
	//commObj.changeStatus(QString::fromStdString(strBuf));
	//commObj.addLog("Saved Reference Data!");

	FileStorage fs;
	fs.open("referenceData.xml", FileStorage::READ);
	//fs.open("calibrationOptitrack.xml", FileStorage::READ);
	fs["M_NC"] >> M_NC;
	//fs["posRef"] >> posRef;
	commObj.addLog("Loaded Reference Data!");
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

void projectCoordinateFrame(Mat pictureFrame)
{
	projectPoints(coordinateFrame, Rvec, Tvec, cameraMatrix, distCoeffs, coordinateFrameProjected);
	line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[3], Scalar(0, 0, 255), 2); //z-axis
	line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[1], Scalar(255, 0, 0), 2); //x-axis
	line(pictureFrame, coordinateFrameProjected[0], coordinateFrameProjected[2], Scalar(0, 255, 0), 2); //y-axis
}

void setUpUDP()
{
	// Creating UDP slot
	commObj.addLog("Opening UDP Port");
	udpSocketCB = new QUdpSocket(0);
	udpSocketDrone = new QUdpSocket(0);
	udpSocketWinch = new QUdpSocket(0);

	udpSocketCB->connectToHost(IPAdressCB, 9156);
	udpSocketWinch->connectToHost(IPAdressWinch, 9155);

	commObj.addLog("Opened UDP Port");

}

void sendDataUDPDrone(double &Latitude, double &Longitude, double &Altitude, cv::Vec3d &Velocity, cv::Vec3d &Euler)
{
	datagram.clear();
	QDataStream out(&datagram, QIODevice::WriteOnly);
	out.setVersion(QDataStream::Qt_4_3);
	Latitude *= 10 ^ 7;		// save bandwith 
	Longitude *= 10 ^ 7;	// save bandwith
	Altitude *= 0.001; // mm to meter
	out << (int)Latitude << (int)Longitude << (float)Altitude;
	out << (float)Euler[0] << (float)Euler[1] << (float)Euler[2]; // Roll Pitch Heading
	out << (float)Velocity[0] << (float)Velocity[1] << (float)Velocity[2];
	udpSocketDrone->writeDatagram(datagram, IPAdressDrone, 9155);
}
