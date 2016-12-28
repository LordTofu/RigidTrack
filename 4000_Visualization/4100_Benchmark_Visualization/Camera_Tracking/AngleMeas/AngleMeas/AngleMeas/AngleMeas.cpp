
#include "stdafx.h"
#include "cameralibrary.h"     //== Camera Library header file ======================---
#include "supportcode.h"

//==-- OpenCV stuff import
#include <opencv\cv.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2\core.hpp"
#include "opencv2\calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>


using namespace CameraLibrary;
using namespace cv;

const double PI = 3.141592653589;
int intIntensity = 6; // max Intensity is 15 1-6 is strobe 7-15 is continuous 13 and 14 are meaningless 
int intExposure = 3;
int intFrameRate = 100;

int intNumSamples = 500; // Samples to take for ellipse fit

void load_calibration();
void start_camera();

Mat_<float> measurement(3, 1);

Mat cameraMatrix;
Mat distCoeffs;
cObject *obj;


// which is why we also set this constant to 8 
const int BACKBUFFER_BITSPERPIXEL = 8;


int main()
{
	load_calibration();
	start_camera();
    return 0;
}

void load_calibration() {
	FileStorage fs;
	fs.open("calibration.xml", FileStorage::READ);
	fs["CameraMatrix"] >> cameraMatrix;
	fs["DistCoeff"] >> distCoeffs;
	std::cout << "Loaded Calibration:\n";
	std::cout << cameraMatrix << "\n";
	std::cout << distCoeffs << "\n";
}

void start_camera() {

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

		std::cout << "No camera found!";
		return;
	}

	//== Determine camera resolution to size application window ==----

	int cameraWidth = camera->Width();
	int cameraHeight = camera->Height();

	cv::Mat mapx, mapy;
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, cv::Size(cameraWidth, cameraHeight), CV_32F, mapx, mapy);


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

	//== Create point array for the ellipse fit points. ==--
	std::vector<Vec2f> list_points2f(intNumSamples);

	//== Set some camera parameters
	camera->SetExposure(intExposure);
	camera->SetIntensity(intIntensity);
	camera->SetFrameRate(intFrameRate);
	camera->SetIRFilter(true);
	camera->SetContinuousIR(false);
	camera->SetHighPowerMode(false);

	//== Create a openCVMat frame ===---
	cv::Mat matFrame(cv::Size(cameraWidth, cameraHeight), CV_8UC1);

	std::ofstream myfile;
	int minArea = 400;
	float objXunrect, objYunrect, objX, objY;
	int intFrames = 0;
	while (intFrames < intNumSamples)
	{

		//== Fetch a new frame from the camera ===---
		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.
			
			for (int i = 0; i < frame->ObjectCount(); i++)
			{
				
				obj = frame->Object(i);
				
				if (obj->Area() > minArea)
				{
					std::cout << obj->Area() << "\n";
					//objXunrect = obj->X();
					//objYunrect = obj->Y();
					//== undistort points
					//objX = mapy.at<float>(objXunrect, objYunrect);
					//objY = mapx.at<float>(objXunrect, objYunrect);
					objX = obj->X();
					objY = obj->Y();
					
					//std::cout << objX << "\t\t" << objY << "\n";
					list_points2f[intFrames] = cv::Vec2f(objX, objY);
					std::cout << obj->Area();
					intFrames++;
				}
			}

				frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);
				imshow("source", matFrame);
				waitKey(1);
				frame->Release();
			
		}
	}

	//== 500 points collected, now lets fit an ellipse to them ==--
	Mat points2f;
	Mat(list_points2f).convertTo(points2f, CV_32F);
	RotatedRect rotRect = fitEllipse(list_points2f);
	std::cout << rotRect.size.height << "\t" << rotRect.size.width << "\t" << rotRect.angle << "\t";
	ellipse(matFrame, rotRect, Scalar(255, 0, 0));
	imshow("source", matFrame);
	waitKey(1);
	
	float fTheta = -rotRect.angle;
	double sinT = sin(fTheta*PI/180.0);
	double cosT = cos(fTheta*PI/180.0);
	float angle = 0;
	float majorAxis = 0;
	if (rotRect.size.height > rotRect.size.width)
	{
		majorAxis = rotRect.size.height;
	}
	else
	{
		majorAxis = rotRect.size.width;
	}

	//== Now that we have the ellipse, start the measurement of the angle ==--
	while (1)
	{

		//== Fetch a new frame from the camera ===---

		Frame *frame = camera->GetFrame();

		if (frame)
		{
			//== Ok, we've received a new frame, lets do something
			//== with it.

			frame->Rasterize(cameraWidth, cameraHeight, matFrame.step, BACKBUFFER_BITSPERPIXEL, matFrame.data);
			ellipse(matFrame, rotRect, Scalar(255, 0, 0));

			for(int i = 0; i < frame->ObjectCount();i++)
			{
				obj = frame->Object(i);
				if (obj->Area() > minArea)
				{

					myfile.open("Values.txt", std::ios::out | std::ios::app);

					//objXunrect = obj->X();
					//objYunrect = obj->Y();
					//== undistort points
					//objX = mapy.at<float>(objXunrect, objYunrect);
					//objY = mapx.at<float>(objXunrect, objYunrect);
					objX = obj->X();
					objY = obj->Y();
					float fX = (objX - rotRect.center.x);
					float fY = (objY - rotRect.center.y);
					cv::Point2f pointRotated = cv::Point2f(cosT*fX - sinT*fY, sinT*fX + cosT*fY);
					
					if (pointRotated.y > 0)
					{
						angle = -acos(pointRotated.x / (majorAxis / 2)) * 180 / PI;
					}
					else if (pointRotated.y <= 0)
					{
						angle = +acos(pointRotated.x / (majorAxis / 2)) * 180 / PI;
					}
					std::cout << angle << "\n";
					
					line(matFrame, rotRect.center, cv::Point2f(objX, objY), Scalar(255, 0, 0));
					//std::cout << "Angle: \t " << angle << "\n";
					myfile << frame->TimeStamp() << "\t" << angle << "\n";
					myfile.close();
				}
				
				//std::cout << pointRotated << "\n";
				//std::cout << "Major Axis components " << pointRotated << "\n";
				//std::cout << frame->TimeStamp() << "\t" << objX << "\t\t" << objY << "\n";
				
				
				//myfile << angle << "\t" << pointRotated[0] << "\t" << cosT*fX - sinT*fY << "\t" << sinT*fX + cosT*fY << "\t"  << frame->TimeStamp() << "\n";
				
							
		}

			frame->Release();
			imshow("source", matFrame);
			waitKey(1);
		}	

	}

	//== Release camera ==--
	camera->Release();

	//== Shutdown Camera Library ==--
	CameraManager::X().Shutdown();

}