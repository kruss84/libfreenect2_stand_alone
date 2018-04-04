/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <fstream>
#include <signal.h>
#include <Windows.h>
#include <TlHelp32.h>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>



#include <gsl/gsl_statistics.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>

#include <gsl/gsl_multifit.h>

//#include <unistd.h>
//#include <sys/time.h>

//#include <Kinect2Registration/kinect2_registration.h>
#include "HSVDetect\hsv_detect.h"
#include "Kinect2Calib\kinect2_calib.h"



// Variables for the Kinect Driver
libfreenect2::Freenect2 freenect2;
libfreenect2::Freenect2Device *dev;


cv::Size sizeColor(1920, 1080);
double scale = 0.5;
cv::Size sizeScaled = cv::Size(sizeColor.width * scale, sizeColor.height * scale);

// struct for images
IplImage* color;
IplImage* color_2;
IplImage* hr;
IplImage* range;
IplImage* hand_depth;

//using namespace Registration;
//Kinect2Registration *depthRegScaled;

unsigned short* Depth_Ready, *cvDepth_Ready;

CvFont font;

bool protonect_shutdown = false;

void sigint_handler(int s)
{
	protonect_shutdown = true;
}


int main(int argc, char *argv[])
{
	std::string prorangeam_path(argv[0]);
	size_t executable_name_idx = prorangeam_path.rfind("Protonect");

	std::string binpath = "/";

	if (executable_name_idx != std::string::npos)
	{
		binpath = prorangeam_path.substr(0, executable_name_idx);
	}

	double maxDepth = 12;

	int32_t device = -1;


	libfreenect2::PacketPipeline *pipeline = new libfreenect2::OpenGLPacketPipeline;


	HSVdetect *hsvdetect = new HSVdetect();
	kinect2Calib calib;


	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}

	//std::string serial = freenect2.getDefaultDeviceSerialNumber();
	std::string serial = "019228645047";
	
	if (pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
		cout << "with Pipeline\n";
	} else
	{
		dev = freenect2.openDevice(serial);
		cout << "without Pipeline\n";
	}
	
	if (dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	// Memory for frames and arrays
	cvInitFont(&hsvdetect->font, CV_FONT_HERSHEY_COMPLEX, scale, scale, 0, 2,
	CV_AA);

	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
	libfreenect2::FrameMap frames;
	//libfreenect2::Frame undistorted(sizeScaled.width, sizeScaled.height, 4), registered(sizeScaled.width, sizeScaled.height, 4);

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	dev->start();

	cout << "device serial: " << dev->getSerialNumber() << endl;
	//std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

	// Load Calibration DATA

	cv::Mat cameraMatrixColor, distortionColor, cameraMatrixScaled, cameraIrScaled, cameraMatrixIr, distortionIr;
	cv::Mat rotation, translation;
	std::map<std::string, std::string> params;
	params["cl_source"] = "../opencl/kinect2_registration.cl"; //clSource;
	cv::Size sizeIr(512, 424);

	double depthShift;

	//Registration::Kinect2Registration::Method reg = Registration::Kinect2Registration::OPENCL;
	//reg = Kinect2Registration::OPENCL;
	cv::Mat map1Ir, map2Ir, map1Scaled, map2Scaled, map1Color, map2Color;
	const int mapType = CV_16SC2;

	std::string calibPath = "../data/" + serial + '/';

	if (!calib.loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
	{
		std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
	}

	if (!calib.loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixIr, distortionIr))
	{
		std::cerr << "using sensor defaults for ir intrinsic parameters." << std::endl;
	}

	if (!calib.loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
	{
		std::cerr << "using defaults for rotation and translation." << std::endl;
	}

	if (!calib.loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
	{
		std::cerr << "using defaults for depth shift." << std::endl;
		depthShift = 0.0;
	}

	if (scale > 0.0)
	{
		cameraMatrixScaled = cameraMatrixColor.clone();
		cameraMatrixScaled.at<double>(0, 0) *= scale;
		cameraMatrixScaled.at<double>(1, 1) *= scale;
		cameraMatrixScaled.at<double>(0, 2) *= scale;
		cameraMatrixScaled.at<double>(1, 2) *= scale;

	}

	// Init remapping and scaling

	rotation.inv();
	translation.at<double>(0, 0) *= -1;
	translation.at<double>(1, 0) *= -1;
	translation.at<double>(2, 0) *= -1;

	//depthRegScaled = Kinect2Registration::New(reg);
	//depthRegScaled->init(cameraMatrixScaled, sizeScaled, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f,
		//maxDepth, device, params);
	
	cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
	cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixScaled, sizeScaled, mapType,
		map1Scaled, map2Scaled);

	cv::Mat image_color_scaled, image_depth_scaled, image_ir_scaled;
	//image_ir_scaled = cv::Mat(sizeScaled.height, sizeScaled.width, CV_16U);
	//image_depth_scaled = cv::Mat(sizeScaled.height, sizeScaled.width, CV_16U);

	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), big_depth(1920, 1082, 4);
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

	unsigned short *arraySize = new unsigned short[sizeScaled.width * sizeScaled.height];
	Depth_Ready = arraySize;
	cvDepth_Ready = arraySize;

	color_2 = cvCreateImage(sizeScaled, 8, 4);

	hsvdetect->init("config/h.xml", cameraMatrixScaled);


	double now_t, last_t = 0;

	struct Time_m{
		string name;
		double start_t, end_t;
	};
	vector<Time_m> time_m;
	cv::Mat bigdepthMat;

	string logPath = "E:/data_log/hsv_log/";
	CreateDirectory(logPath.c_str(), NULL);
	ofstream hsvProcess(logPath + hsvdetect->currentDateTime() + "_hsv_process.txt");
	int count = 0;
	while (!protonect_shutdown)
	{

		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		time_m.push_back(Time_m{"convert",clock()});
		cv::Mat image_depth = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
		cv::Mat image_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
		time_m.back().end_t = clock();

		time_m.push_back(Time_m{"scale", clock() });
		// Color
		cv::remap(image_color, image_color_scaled, map1Scaled, map2Scaled, cv::INTER_AREA);

		// Depth
		registration->apply(rgb, depth, &undistorted, &registered, true, &big_depth);
		cv::Mat(big_depth.height, big_depth.width, CV_32FC1, big_depth.data).copyTo(bigdepthMat);
		bigdepthMat.convertTo(bigdepthMat, CV_16UC1);
		cv::remap(bigdepthMat, bigdepthMat, map1Scaled, map2Scaled, cv::INTER_AREA);
		
		
		//image_depth.convertTo(image_depth, CV_16U, 1);
		//cv::remap(image_depth, image_depth, map1Ir, map2Ir, cv::INTER_AREA);
		//cout << image_depth.size() << "depth init\n";
		//cout << image_depth_scaled.size() << "depth init\n";
		//depthRegScaled->registerDepth(image_depth, image_depth_scaled);
		time_m.back().end_t = clock();

		color = new IplImage(image_color_scaled);
		//color_2 = new IplImage(image_color_scaled);
		cvCopy(color, color_2);
		range = new IplImage(bigdepthMat);

		memcpy(Depth_Ready, range->imageData, sizeScaled.width * sizeScaled.height * 2);

		//cvSetData(range, range->imageData, range->widthStep);
		//cvConvertScale(range, range, 32, 0);

		//hsvdetect->savePicture(range, "depth.png");
		//hsvdetect->savePicture(color, "color.jpeg");


		time_m.push_back(Time_m{ "background", clock() });
		hsvdetect->selectBackground(color, range, Depth_Ready);
		time_m.back().end_t = clock();
		//hsvdetect->savePicture(range, "depth_background.png");
		//hsvdetect->savePicture(color, "color_background.jpeg");

		//memcpy(cvDepth_Ready, range->imageData, sizeScaled.width * sizeScaled.height * 2);
		time_m.push_back(Time_m{ "face", clock() });
		bool face_ = hsvdetect->faceDetection(color, Depth_Ready);
		time_m.back().end_t = clock();
		time_m.push_back(Time_m{ "body", 0,0 });
		time_m.push_back(Time_m{ "hsv color", 0, 0 });
		if (face_)					// 12 ms
		{
			
			//hsvdetect->savePicture(color, "color_face.jpeg");
			//cvConvertScale(range, range, 0.07, 0);
			time_m[time_m.size() - 2].start_t = clock();
			hsvdetect->selectBody(color,range, Depth_Ready);
			//cvConvertScale(range, range, 16, 0);

			time_m[time_m.size() - 2].end_t = clock();

			time_m[time_m.size()-1].start_t = clock();
			bool hsv_ = hsvdetect->meanFaceColor(color, Depth_Ready);
			time_m[time_m.size()-1].end_t = clock();

			//hsvdetect->savePicture(range, "depth_body.png");
			if (hsv_)			// 3 ms
			{
				//hsvdetect->savePicture(color, "color_hsv.jpeg");

				//gettimeofday(&start, 0);
				hsvdetect->searchHand(Depth_Ready);						// 3,1 ms
				//gettimeofday(&end, 0);
				
				if (hsvdetect->searchObjekt(Depth_Ready))				// 4 ms
				{
					hsvdetect->lineEstimationObject();					// 1,5 ms
					hsvdetect->depthEstimationObject(Depth_Ready);		// 0,023 ms
					//hsvdetect->drawObejctFrame(color);					// 0.01 ms

				} else
				{
					hsvdetect->lineEstimationHand();
					hsvdetect->depthEstimationHand(Depth_Ready);
					hsvdetect->searchHandArea(Depth_Ready);
					hsvdetect->drawHandFrame(color);
				}
				hsvdetect->debugingWindow(color);			// 0,012 ms
				
			}

		}

		//cvSetData(range, range->imageData, range->widthStep);
		//cvConvertScale(range, range, 32, 0);
	
		//hsvdetect->savePicture(color, "color_final.jpeg");

		// imshow

		//cvShowImage("RGB 2",color_2);

		cvShowImage("RGB SCALED", color);
		//cvShowImage("DEPTH REGISTERED",range);
		//cvShowImage("RGB 2",color_2);
		hsvdetect->count++;

		now_t = clock();
		cout << "Intenta Zykluszeit:  " + to_string(((now_t - last_t) / CLOCKS_PER_SEC)) + " Sekunden \n";
		if (last_t > 0)
		{
			hsvProcess << hsvdetect->currentDateTime() << " ; ";
			for (int i = 0; i < time_m.size(); i++)
			{
				hsvProcess << to_string(((time_m[i].end_t - time_m[i].start_t) / CLOCKS_PER_SEC)) << " ; ";
			}
			
			hsvProcess << to_string(((now_t - last_t) / CLOCKS_PER_SEC)) + " ; ";
			hsvProcess << endl;
		}
		else
		{
			hsvProcess << "Date ; ";
			for (int i = 0; i < time_m.size(); i++)
			{
				hsvProcess << time_m[i].name.c_str() << " ; ";
			}
			hsvProcess << "Cycel Time ; ";
			hsvProcess << endl;
		}
		time_m.clear();
		last_t = now_t;

		int key = cv::waitKey(1);
		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

		listener.release(frames);
		//libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));



	}

	// TODO: restarting ir stream doesn't work!
	// TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
	dev->stop();
	dev->close();

	return 0;
}
