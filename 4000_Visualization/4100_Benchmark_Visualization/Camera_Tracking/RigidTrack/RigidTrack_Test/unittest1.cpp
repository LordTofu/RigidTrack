#include "stdafx.h"
#include "CppUnitTest.h"
#include <opencv\cv.h>
#include "opencv2\core.hpp"
#include "opencv2\calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2\video\tracking.hpp>


using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace cv;

namespace RigidTrack_Test
{		
	TEST_CLASS(UnitTest1)
	{

	public:
		
		TEST_METHOD(TestMethod1)
		{
			Mat cameraMatrix;
			Mat distCoeffs;

			load_calibration();

			std::vector<Vec3d> list_points3d(4);
			std::vector<Point2d> list_points2d(4);

			Mat Rvec = (cv::Mat_<double>(3, 1) << 0.78, 0.0, 0.0);
			Mat Tvec = (cv::Mat_<double>(3, 1) << 60.0, 30.0, 1000);
			Mat RvecOriginal = Rvec;
			Mat TvecOriginal = Tvec;
	

			list_points3d[0] = cv::Vec3d(0.0, 0.0, 0);
			list_points3d[1] = cv::Vec3d(55.0, -455.0, 0);
			list_points3d[2] = cv::Vec3d(55.0, 450.0, 0);
			list_points3d[3] = cv::Vec3d(355.0, -80.0, 0);

			projectPoints(list_points3d, Rvec, Tvec, cameraMatrix, distCoeffs, list_points2d);
			
			bool sufficientAccuracy = false;
			float tolerance = 0.01; // in mm

			bool useGuess = true;
			int methodPNP = 0; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
			solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
			if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }
			Assert::IsTrue(sufficientAccuracy);

			methodPNP = 1; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
			solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
			if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }
			Assert::IsTrue(sufficientAccuracy);

			methodPNP = 2; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
			solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
			if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }
			Assert::IsTrue(sufficientAccuracy);

			methodPNP = 4; // 0 = iterative 1 = EPNP 2 = P3P 4 = UPNP  // not used
			solvePnP(list_points3d, list_points2d, cameraMatrix, distCoeffs, Rvec, Tvec, useGuess, methodPNP);
			if (norm(Rvec - RvecOriginal) <= tolerance) { sufficientAccuracy = true; }
			Assert::IsTrue(sufficientAccuracy);
		}

	};
}