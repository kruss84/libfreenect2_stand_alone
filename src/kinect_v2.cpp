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

#include "Kinect2Calib\kinect2_calib.h"
#include "WorldFrame.h"
#include "VideoStreamer/VideoStreamer.h"
#include <math.h>




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

bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion)
{
	FileStorage fs;
	if (fs.open(filename, cv::FileStorage::READ))
	{
		fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
		fs[K2_CALIB_DISTORTION] >> distortion;
		fs.release();
	}
	else
	{
		std::cerr << "can't open calibration file: " << filename << std::endl;
		return false;
	}
	return true;
}

bool extern_parameter_output(Mat img, bool calibrate_flag = false){
	if (calibrate_flag){
		CoordinateFrame::WorldFrame worldFrame_1;

		std::string calib_path = "../data/";
		std::string serial = dev->getSerialNumber();
		std::string calibPath = calib_path + serial + '/';

		Mat cameraMatrixColor = Mat::eye(3, 3, CV_64F);
		Mat distortionColor = Mat::zeros(1, 5, CV_64F);
		if (!loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
		{
			std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
		}

		worldFrame_1.points_per_row = 9;
		worldFrame_1.points_per_col = 7;
		worldFrame_1.squareSize = 0.1;
		/*
		worldFrame_1.points_per_row = 7;
		worldFrame_1.points_per_col = 5;
		worldFrame_1.squareSize = 0.048;
		*/
		worldFrame_1.scale_dis = 2;
		worldFrame_1.scale_display = true;

		worldFrame_1.calibPath = calib_path + serial;
		worldFrame_1.calibFile = "external_Parameters_" + serial + ".xml";
		worldFrame_1.serial = serial;

		worldFrame_1.K = cameraMatrixColor;
		worldFrame_1.dist = distortionColor;

		worldFrame_1.init();

		worldFrame_1.calibrate = true;
		worldFrame_1.save = true;
		worldFrame_1.calibWorldFrame(img);
	}
	return true;
}

bool load_extern_parameter(string filename, Mat rvecs, Mat tvecs){
	FileStorage fs(filename, FileStorage::READ);
	fs ["r_C"] >> rvecs;
	fs ["t_C"] >> tvecs;
	fs.release();
	for (int i = 0; i<3; i++)
	{
		cout << rvecs.at<double>(i,0) <<endl;
	}
	return true;
}

void move_callback(int event, int x, int y, int flags, void* pixel_location)
{
	Point* temp = (Point*)pixel_location;
	temp->x = x;
	temp->y = y;
	ostringstream ss;
	//float f = show.at<float>(y, x);
	//cout << "f is " << f;
	//ss << f;
	//putText(*temp, ss.str(), Point(20, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255,255,255), 4, 8);
	//cout << "x:" << x << " y:" << y << " depth:" << show.at<float>(y, x) << endl;
	
	//imshow("calibration", show);
}

Mat depth_calibrate_test(Mat depth, cv::Mat cameraMatrixIr, cv::Mat rvecs, cv::Mat tvecs)
{
	Mat depth_calibration(depth.rows, depth.cols, depth.type());
	Mat depth_calibration_test(depth.rows, depth.cols, depth.type());
	//intrinsische Parameter
	double* I_P = (double*)cameraMatrixIr.data;
	float fx = I_P[0], fy = I_P[4], cx = I_P[2], cy = I_P[5];
	//extrinsische Parameter
	Mat r_matrix, r_matrixinv;

	Rodrigues(rvecs, r_matrix);
	//Rotation Matrix
	r_matrixinv = r_matrix.inv();

	double *R_M = (double*)r_matrixinv.data;
	/*
	for (int i = 0; i<3; i++)
	{
	for (int j = 0; j<3; j++)
	{
	//R_M[j + 3 * i] = r_matrixinv.at<double>(i, j);
	cout << R_M[j + 3 * i] << endl;
	}

	}
	*/
	//Weltkoordinatesystem
	float x_w, y_w, z_w;


	//x richtung
	Point up(200, 200), down(200, 400);
	double angle_x;
	//double r_x[9];
	//r_x[0] = 1;
	//r_x[1] = r_x[2] = r_x[3] = r_x[6] = 0;
	float z_up = depth.at<float>(up);
	float y_up = (up.y - cy) * z_up / fy;
	float z_down = depth.at<float>(down);
	float y_down = (down.y - cy) * z_down / fy;
	float diff_y, diff_y_max = 0;
	float z_up_temp = z_up;
	float z_down_temp = z_down;
	for (int i = -90; i < 90; i++){
		z_up = y_up * sin(i * 3.1415 / 180) + depth.at<float>(up) * cos(i * 3.1415 / 180);
		z_down = y_down * sin(i * 3.1415 / 180) + depth.at<float>(down) * cos(i * 3.1415 / 180);
		diff_y = abs(z_up - z_down);
		//cout << "i " << i << " diff " << diff_y << endl;
		if (diff_y > diff_y_max){
			diff_y_max = diff_y;
			angle_x = i;
		}
	}
	cout << "angle_x " << angle_x << endl;
	//x richtung

	R_M[0] = 1;
	R_M[1] = R_M[2] = R_M[3] = R_M[6] = 0;
	R_M[4] = R_M[8] = cos(angle_x * 3.1415 / 180);
	R_M[7] = sin(angle_x * 3.1415 / 180);
	R_M[5] = -sin(angle_x * 3.1415 / 180);

	for (int i = 0; i < depth.rows; i++){
		for (int j = 0; j < depth.cols; j++){
			z_w = depth.at<float>(i, j) / 1000;
			x_w = (j - cx) *  z_w / fx;
			y_w = (i - cy) * z_w / fy;

			if (z_w>0)
				z_w = (R_M[6] * x_w + R_M[7] * y_w + R_M[8] * z_w) * 1000;
			//cout << "z_c " << z_c << endl;
			/*
			if (z_w > d_max){
			d_max = z_w;
			}
			if (z_w < d_min){
			d_min = z_w;
			}
			*/
			//cout << "x " << x_c << "y " << y_c << "z "  << endl;
			depth_calibration.at<float>(i, j) = z_w;

		}
	}

	//imshow("depth", depth);
	//imshow("calibration", depth_calibration);
	//setMouseCallback("depth", move_callback, &depth);
	//setMouseCallback("calibration", move_callback, &depth_calibration);
	//waitKey(0);
	return depth_calibration;
}
Mat depth_point_test(Mat depth, cv::Mat cameraMatrixIr, cv::Mat rvecs, cv::Mat tvecs)
{
	//kreuz
	Point up(200,200), down(200,400), left, right;
	/*
	float mean_y, diff_y, diff_y_sum = 0;
	for (int i = up.y; i < down.y; i++){
		diff_y = depth.at<float>(i + 1, up.x) - depth.at<float>(i, up.x);
		diff_y_sum = diff_y_sum + diff_y;
	}
	mean_y = diff_y_sum / (down.y - up.y);
	depth.at<float>(down.y, down.x) = depth.at<float>(up.y, up.x) + 


	float diff_x, diff_x_sum = 0;
	for (int i = left.y; i < right.y; i++){
		diff_x = depth.at<float>(left.y, i+1) - depth.at<float>(left.y, i);
		diff_x_sum = diff_x_sum + diff_x;
	}
	*/

	//kreuz

	Mat depth_calibration(depth.rows, depth.cols, depth.type());
	Mat depth_calibration_test(depth.rows * 4, depth.cols * 4, depth.type());
	//intrinsische Parameter
	double* I_P = (double*)cameraMatrixIr.data;
	float fx = I_P[0], fy = I_P[4], cx = I_P[2], cy = I_P[5];
	//extrinsische Parameter
	Mat r_matrix, r_matrixinv;

	Rodrigues(rvecs, r_matrix);
	//Rotation Matrix
	r_matrixinv = r_matrix.inv();

	double *R_M = (double*)r_matrixinv.data;
	double* temp_M = (double*)r_matrix.data;
	for (int i = 0; i < 9; i++){
		cout << i << "  R_M" << R_M[i] << "temp_M" << temp_M[i] << endl;
	}
	/*
	for (int i = 0; i<3; i++)
	{
	for (int j = 0; j<3; j++)
	{
	//R_M[j + 3 * i] = r_matrixinv.at<double>(i, j);
	cout << R_M[j + 3 * i] << endl;
	}

	}
	*/
	//wold coordinatesystem
	float x_w, y_w, z_w;
	//camera coordinatesystem
	float x_c, y_c, z_c;
	float x_min = 0, x_max = 0, y_min = 0, y_max = 0, x_shift = 1200, y_shift = 1400;
	float x_temp, y_temp;

	Mat point_cloud(1080, 1920, depth.type());

	//x richtung
	double angle_x;
	//double r_x[9];
	//r_x[0] = 1;
	//r_x[1] = r_x[2] = r_x[3] = r_x[6] = 0;
	float z_up = depth.at<float>(up);
	float y_up = (up.y - cy) * z_up / fy;
	float z_down = depth.at<float>(down);
	float y_down = (down.y - cy) * z_down / fy;
	float diff_y, diff_y_max = 0;
	float z_up_temp = z_up;
	float z_down_temp = z_down;
	for (int i = -90; i < 90; i++){
		z_up = y_up * sin(i * 3.1415 / 180) + depth.at<float>(up) * cos(i * 3.1415 / 180);
		z_down = y_down * sin(i * 3.1415 / 180) + depth.at<float>(down) * cos(i * 3.1415 / 180);
		diff_y = abs(z_up - z_down);
		//cout << "i " << i << " diff " << diff_y << endl;
		if (diff_y > diff_y_max){
			diff_y_max = diff_y;
			angle_x = i;
		}
	}
	//x richtung
	//cout << "angle_x" << angle_x << endl;


	R_M[0] = 1;
	R_M[1] = R_M[2] = R_M[3] = R_M[6] = 0;
	R_M[4] = R_M[8] = cos(angle_x * 3.1415 / 180);
	R_M[7] = sin(angle_x * 3.1415 / 180);
	R_M[5] = -sin(angle_x * 3.1415 / 180);
	//cout << "angle_x " << angle_x << " sin " << sin(angle_x * 3.1415 / 180) << endl;

	for (int i = 0; i < depth.rows; i++){
		for (int j = 0; j < depth.cols; j++){
			z_c = depth.at<float>(i, j) / 1000;
			x_c = (j - cx) *  z_c / fx;
			y_c = (i - cy) * z_c / fy;

			//test
			float x_p, y_p;
			if (z_c > 0){

				x_w = R_M[0] * x_c + R_M[1] * y_c + R_M[2] * z_c;
				y_w = R_M[3] * x_c + R_M[4] * y_c + R_M[5] * z_c;
				z_w = R_M[6] * x_c + R_M[7] * y_c + R_M[8] * z_c;

				//x_w = x_c;
				//y_w = y_c;
				//z_w = z_c;

				x_p = x_w * fx / 1 + cx;
				y_p = y_w * fy / 1 + cy;


				x_temp = x_p;
				y_temp = y_p;
				//x_c = temp_M[7] * x_temp + temp_M[8] * y_temp;// +temp_M[3] * z_w;
				//x_c = R_M[7] * x_temp + R_M[8] * y_temp;// +R_M[2] * z_w;
				//x_c = 0.707 * x_temp - 0.707 * y_temp;
				//y_c = temp_M[4] * x_temp + temp_M[5] * y_temp;// +temp_M[6] * z_w;
				//y_c = R_M[4] * x_temp + R_M[5] * y_temp;// +R_M[5] * z_w;
				//y_c = 0.707 * x_temp + 0.707 * y_temp;
			}

			//if (2 * depth.cols+ 10 > x_c || x_c > 2 * depth.cols -13)
			//x_c = 0;
			//if (2 * depth.rows+ 10 > y_c || y_c > 2 * depth.rows -13)
			//y_c = 0;

			//test

			//cout << "x " << x_c << "y " << y_c << "z "  << endl;
			//test
			int x_pixel = -x_p + x_shift;
			//x_pixel = depth_calibration_test.cols - x_pixel;
			int y_pixel = y_p;// +x_shift;
			//y_pixel = depth_calibration_test.rows - y_pixel;
			if (x_pixel<0 || x_pixel > point_cloud.cols - 1)
				x_pixel = 0;
			if (y_pixel<0 || y_pixel > point_cloud.rows - 1)
				y_pixel = 0;


			//cout << "x " << x_pixel << "y " << y_pixel << "z " << endl;
			point_cloud.at<float>(y_pixel, x_pixel) = z_w * 1000;

		}
	}
	return point_cloud;
}

int main(int argc, char *argv[]){
	
	std::string prorangeam_path(argv[0]);
	size_t executable_name_idx = prorangeam_path.rfind("Protonect");

	std::string binpath = "/";
	std::string video_path = "../";

	if (executable_name_idx != std::string::npos)
	{
		binpath = prorangeam_path.substr(0, executable_name_idx);
	}

	double maxDepth = 12;

	int32_t device = -1;


	libfreenect2::PacketPipeline *pipeline = new libfreenect2::OpenGLPacketPipeline;

	kinect2Calib calib;


	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}

	cout << freenect2.getDefaultDeviceSerialNumber() << endl;
	std::string serial = ("018470145047");
	cout << serial << endl;
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

	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
	libfreenect2::FrameMap frames;
	//libfreenect2::Frame undistorted(sizeScaled.width, sizeScaled.height, 4), registered(sizeScaled.width, sizeScaled.height, 4);

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	dev->start();

	//std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	//std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

	// Load Calibration DATA

	cv::Mat cameraMatrixColor, distortionColor, cameraMatrixScaled, cameraIrScaled, cameraMatrixIr, distortionIr;
	cv::Mat rotation, translation;
	std::map<std::string, std::string> params;
	params["cl_source"] = "../opencl/kinect2_registration.cl"; //clSource;
	cv::Size sizeIr(512, 424);

	double depthShift;

	cv::Mat map1Ir, map2Ir, map1Scaled, map2Scaled, map1Color, map2Color;
	const int mapType = CV_16SC2;
	cout << serial << endl;
	cout << "Breakpoint\n";
	std::string calibPath = "../data/" + serial + "/" ;
	cout << "Breakpoint\n";
	if (!calib.loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
	{
		std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
	}

	if (!calib.loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixIr, distortionIr))
	{
		std::cerr << "using sensor defaults for ir intrinsic parameters." << std::endl;
	}
	double ir_d;
	double* ir_p = (double*)cameraMatrixIr.data;
	for (int i = 0; i < cameraMatrixIr.rows; i++){
		for (int j = 0; j < cameraMatrixIr.cols; j++){

			ir_d = ir_p[i];
			cout << ir_d << endl;

		}
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
	

	//Mat r_temp, t_temp;
	//load_extern_parameter(calibPath + "external_Parameters_018470145047.xml", r_temp, t_temp);
	


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
	
	cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
	cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixScaled, sizeScaled, mapType,
		map1Scaled, map2Scaled);



	int count = 0;

	//Viedo output
	//VideoCapture video_reader(video_path + "color_2018-04-06-11-06-52.avi");

	VideoStreamer vs;
	Mat depth_c, point_cloud;
	Point pixel_location;

	bool recorde_flag = 0;
	bool depth_calibrate_flag = 0;
	bool calibrate_flag = 0;
	CoordinateFrame::CaliWithFloor floor_test;

	while (!protonect_shutdown)
	{
		int key = cv::waitKey(1);
		recorde_flag = (key == 114 || key == 82) ? (!recorde_flag) : recorde_flag;
		depth_calibrate_flag = (key == 100 || key == 68) ? (!depth_calibrate_flag) : depth_calibrate_flag;
		calibrate_flag = (key == 67 || key == 99) ? true : false;

		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];


		cv::Mat image_depth = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
		cv::Mat image_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);

		//video record 
		vs.video_recorder(video_path, image_color, 25.0, VideoStreamer::COLOR, recorde_flag);
		vs.video_recorder(video_path, image_depth, 25.0, VideoStreamer::DEPTH, recorde_flag);

		extern_parameter_output(image_color, calibrate_flag);

		if (recorde_flag){
			putText(image_color, "(R)ecording..., (C)alibrate, (D)epth calibration, (Esc)exit", Point(50, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 23, 0), 4, 8);
		}
		else if (!recorde_flag){
			putText(image_color, "(R)ecord start, (C)alibrate, (D)epth calibration, (Esc)exit", Point(50, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 23, 0), 4, 8);
		}

		

		if (depth_calibrate_flag){

			if (floor_test.angle_x == 0){
				floor_test.init(image_depth, cameraMatrixIr);
			}


			Mat rvecs(3, 1, CV_64FC1), t_temp, tvecs;
			load_extern_parameter(calibPath + "external_Parameters_018470145047.xml", rvecs, tvecs);
			//setMouseCallback("depth", move_callback, &depth);
			//depth_c = CoordinateFrame::depth_calibrate(image_depth, cameraMatrixIr, rvecs, tvecs);
			depth_c = floor_test.depth_calibrate(image_depth);

			//point_cloud = CoordinateFrame::depth_point_cloud(image_depth, cameraMatrixIr, rvecs, tvecs);
			//point_cloud = depth_point_test(image_depth, cameraMatrixIr, rvecs, tvecs);
			point_cloud = floor_test.point_cloud(image_depth);

			setMouseCallback("calibration", move_callback, &pixel_location);
			setMouseCallback("depth", move_callback, &pixel_location);
			ostringstream ss, ss2;
			float f = image_depth.at<float>(pixel_location.y, pixel_location.x);
			float f_c = depth_c.at<float>(pixel_location.y, pixel_location.x);
			//cout << "x " << pixel_location.x << " y " << pixel_location.y << endl;
			ss << f_c;
			putText(depth_c, ss.str(), Point(20, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 4, 8);
			ss2 << f;
			putText(image_depth, ss2.str(), Point(20, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 4, 8);

			imshow("calibration", depth_c);
			imshow("point_cloud", point_cloud);
		}
		else{
			destroyWindow("calibration");
		}

		imshow("RGB", image_color);
		imshow("depth", image_depth);


		protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

		listener.release(frames);
		//libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));


		// read video
		/*
		Mat temp_read;
		if (vs.video_play(video_path + "depth_2018-04-06-15-17-53.avi", temp_read, true))
			imshow("temp_read", temp_read);
		*/
	}


	// TODO: restarting ir stream doesn't work!
	// TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
	dev->stop();
	dev->close();

	return 0;
}
