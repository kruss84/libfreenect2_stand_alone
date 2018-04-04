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

#include <gsl/gsl_statistics.h>
#include <gsl/gsl_sort.h>
#include <gsl/gsl_statistics.h>

//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>

#include <gsl/gsl_multifit.h>

//#include <unistd.h>
//#include <sys/time.h>

#include <Kinect2Registration/kinect2_registration.h>
#include "HSVDetect\hsv_detect.h"
#include "Kinect2Calib\kinect2_calib.h"

cv::Size sizeColor(1920, 1080);
double scale = 0.5;
cv::Size sizeScaled = cv::Size(sizeColor.width * scale, sizeColor.height * scale);

// struct for images
IplImage* color;
IplImage* color_2;
IplImage* hr;
IplImage* range;
IplImage* hand_depth;

using namespace Registration;
Kinect2Registration *depthRegScaled;

unsigned short* Depth_Ready, *cvDepth_Ready;

CvFont font;

bool protonect_shutdown = false;

void sigint_handler(int s)
{
	protonect_shutdown = true;
}

struct FileInfo{
	string name;
	string path;
	int index;
};

bool sortVector(FileInfo i, FileInfo j) { return (i.index < j.index); }

void BrightnessAndContrastAuto(const cv::Mat &src, cv::Mat &dst, float clipHistPercent = 0)
{

	CV_Assert(clipHistPercent >= 0);
	CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));

	int histSize = 256;
	float alpha, beta;
	double minGray = 0, maxGray = 0;

	//to calculate grayscale histogram
	cv::Mat gray;
	if (src.type() == CV_8UC1) gray = src;
	else if (src.type() == CV_8UC3) cvtColor(src, gray, CV_BGR2GRAY);
	else if (src.type() == CV_8UC4) cvtColor(src, gray, CV_BGRA2GRAY);
	if (clipHistPercent == 0)
	{
		// keep full available range
		cv::minMaxLoc(gray, &minGray, &maxGray);
	}
	else
	{
		cv::Mat hist; //the grayscale histogram

		float range[] = { 0, 256 };
		const float* histRange = { range };
		bool uniform = true;
		bool accumulate = false;
		calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

		// calculate cumulative distribution from the histogram
		std::vector<float> accumulator(histSize);
		accumulator[0] = hist.at<float>(0);
		for (int i = 1; i < histSize; i++)
		{
			accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
		}

		// locate points that cuts at required value
		float max = accumulator.back();
		clipHistPercent *= (max / 100.0); //make percent as absolute
		clipHistPercent /= 2.0; // left and right wings
		// locate left cut
		minGray = 0;
		while (accumulator[minGray] < clipHistPercent)
			minGray++;

		// locate right cut
		maxGray = histSize - 1;
		while (accumulator[maxGray] >= (max - clipHistPercent))
			maxGray--;
	}

	// current range
	float inputRange = maxGray - minGray;

	alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
	beta = -minGray * alpha;             // beta shifts current range so that minGray will go to 0

	// Apply brightness and contrast normalization
	// convertTo operates with saurate_cast
	src.convertTo(dst, -1, alpha, beta);

	// restore alpha channel from source 
	if (dst.type() == CV_8UC4)
	{
		int from_to[] = { 3, 3 };
		cv::mixChannels(&src, 4, &dst, 1, from_to, 1);
	}
	return;
}

std::vector<std::string> GetFileNamesInDirectory(std::string directory)
{
	std::vector<std::string> files;
	WIN32_FIND_DATA fileData;
	HANDLE hFind;

	if (!((hFind = FindFirstFile(directory.c_str(), &fileData)) == INVALID_HANDLE_VALUE)) {
		while (FindNextFile(hFind, &fileData)) {
			files.push_back(fileData.cFileName);
		}
	}

	FindClose(hFind);
	return files;
}

int main(int argc, char *argv[])
{

	HANDLE hFind_;
	WIN32_FIND_DATA data_;
	string srcPath_ = "E:/data_set/video/" + string("*");
	hFind_ = FindFirstFile(srcPath_.c_str(), &data_);

	vector<string> path_dataSet;

	if (hFind_ != INVALID_HANDLE_VALUE)
	{
		do {
			printf("%s\n", data_.cFileName);
			string name(data_.cFileName);
			if (name != "." && name != "..")
			{
				path_dataSet.push_back("E:/data_set/video/" + name + "/");
				cout << path_dataSet.back() << endl;
			}

		} while (FindNextFile(hFind_, &data_));
		FindClose(hFind_);
	}


	cout << path_dataSet[0] << endl;
	std::string prorangeam_path(argv[0]);
	size_t executable_name_idx = prorangeam_path.rfind("Protonect");

	std::string binpath = "/";

	if (executable_name_idx != std::string::npos)
	{
		binpath = prorangeam_path.substr(0, executable_name_idx);
	}

	

	HSVdetect *hsvdetect = new HSVdetect();
	kinect2Calib calib;

	//std::string serial = freenect2.getDefaultDeviceSerialNumber();
	//std::string serial = "019228645047";
	std::string serial = "017732145047";
	//std::string serial = "019228645047";


	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	// Load Calibration DATA

	cv::Mat cameraMatrixColor, distortionColor, cameraMatrixScaled, cameraIrScaled, cameraMatrixIr, distortionIr;
	cv::Mat rotation, translation;
	double depthShift;

	std::string calibPath = "../data/" + serial + '/';

	if (!calib.loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
	{
		std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
	}
	cout << calibPath << endl;
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
	cout << calibPath << endl;
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

	cv::Mat image_color_scaled, image_depth_scaled, image_ir_scaled;
	unsigned short *arraySize = new unsigned short[sizeScaled.width * sizeScaled.height];
	Depth_Ready = arraySize;
	cvDepth_Ready = arraySize;

	color_2 = cvCreateImage(sizeScaled, 8, 4);




	cv::Mat bigdepthMat;
	vector<vector<double>> skin_pixel;
	vector<FileInfo> color_raw_str;
	vector<FileInfo> depth_raw_str;
	vector<FileInfo> ir_raw_str;
	vector<Mat> channels;

	Mat image_rgb_raw;
	Mat image_depth_raw;
	Mat image_ir_raw;

		
	double now_t, last_t = 0;
		struct Time_m{
			string name;
			double start_t, end_t;
		};


		cv::VideoWriter output;

		
	int i = 2;	
	for (int i = 0; i < path_dataSet.size(); i++)
	{

		vector<Time_m> time_m;
		//string folder = "image_raw/";
		//string folder = "image_face/";
		string folder = "image_hsv/";

		HANDLE hFind;
		WIN32_FIND_DATA data;
		string srcPath = path_dataSet[i] + folder + string("*.png");
		cout << srcPath << endl;
		hFind = FindFirstFile(srcPath.c_str(), &data);

		color_raw_str.clear();
		depth_raw_str.clear();
		ir_raw_str.clear();

		if (hFind != INVALID_HANDLE_VALUE) 
		{
			do {
				//printf("%s\n", data.cFileName);
				string name(data.cFileName);
				if (name.find("color") != std::string::npos)
				{
					color_raw_str.push_back(FileInfo{ name, string(path_dataSet[i]), stof(name.substr(name.find("_") + 1, name.find(".") - 1)) });
					//printf("%d Path: %s File: %s\n", color_raw_str.back().index, color_raw_str.back().path.c_str(), color_raw_str.back().name);
				}
				if (name.find("depth") != std::string::npos)
				{
					depth_raw_str.push_back(FileInfo{ name, string(path_dataSet[i]), stof(name.substr(name.find("_") + 1, name.find(".") - 1)) });
					//printf("%d Path: %s File: %s\n", depth_raw_str.back().index, depth_raw_str.back().path.c_str(), depth_raw_str.back().name);
				}
				if (name.find("ir") != std::string::npos)
				{
					ir_raw_str.push_back(FileInfo{ name, string(path_dataSet[i]), stof(name.substr(name.find("_") + 1, name.find(".") - 1)) });
					//printf("%d Path: %s File: %s\n", depth_raw_str.back().index, depth_raw_str.back().path.c_str(), depth_raw_str.back().name);
				}
			} while (FindNextFile(hFind, &data));
			FindClose(hFind);
		}

		std::sort(color_raw_str.begin(), color_raw_str.end(),sortVector);
		std::sort(depth_raw_str.begin(), depth_raw_str.end(), sortVector);

		string hsv_methode[] = { "color_stretch_full"};

		string imagesResultPath = path_dataSet[i] + "result_" + hsv_methode[0] + "/";
		last_t = 0;
		vector<std::string> vFileNames = GetFileNamesInDirectory(imagesResultPath + "*");
		for (int i = 0; i < vFileNames.size(); i++) {
			remove(string(imagesResultPath + vFileNames[i]).c_str());
		}
		CreateDirectory(imagesResultPath.c_str(), NULL);
		vFileNames.clear();

		string imagePath = path_dataSet[i] + "image_hsv" + "/";
		//last_t = 0;
		//vFileNames = GetFileNamesInDirectory(imagePath + "*");
		//for (int i = 0; i < vFileNames.size(); i++) {
		//	remove(string(imagePath + vFileNames[i]).c_str());
		//}
		//CreateDirectory(imagePath.c_str(), NULL);

		float clipHistPercent_[] = {0.0,5.0,10.0,15.0};
		vector<float> clipHistPercent(clipHistPercent_, clipHistPercent_ + sizeof(clipHistPercent_)/sizeof(float));
		int k = 0;
		int l = 2;
		for (int l = 0; l < clipHistPercent.size(); l++)
		{


			hsvdetect = new HSVdetect();
			// Memory for frames and arrays
			cvInitFont(&hsvdetect->font, CV_FONT_HERSHEY_COMPLEX, 1, 1, 0, 1,
				CV_AA);
			last_t = 0;
			ofstream hsvProcess(path_dataSet[i] + "hsv_process_"+ hsv_methode[k] + "_" + to_string(clipHistPercent[l]) +  ".txt");
			hsvdetect->logPath = path_dataSet[i];
			hsvdetect->init("config/h.xml", cameraMatrixScaled);
			hsvdetect->minDist_background = 400;
			hsvdetect->maxDist_background = 2000;
			hsvdetect->maxDist_face = 2000;

			hsvdetect->minSafeDist_hsv = 80;
			hsvdetect->maxSafeDist_hsv = 400;

			hsvdetect->hsvData = new ofstream(hsvdetect->logPath + "hsvData_" + hsv_methode[k] + "_" + to_string(clipHistPercent[l]) + ".txt");
			*hsvdetect->hsvData << "frame ; face pixel ;";
			//*hsvData << " H_mean ; H_s ; H_min ; H_max ; H_min ; H_max ;";
			//*hsvData << " S_mean ; S_s ; S_min ; S_max ; S_min ; S_max ;";
			//*hsvData << " V_mean ; V_s ; V_min ; V_max ; V_min ; V_max ;";
			*hsvdetect->hsvData << "mask pixel ; skin pixel ; object pixel ; " << endl;

			output.open(string(imagesResultPath)  + "result_" + to_string(clipHistPercent[l]) + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 5, cv::Size(1920 / 2, 1080 / 2), true);

			cout << "Init\n";
			int count = 0;

			for (int j = 0; j < color_raw_str.size() && !protonect_shutdown; j++)
			{



				image_color_scaled = imread(color_raw_str[i].path + folder + color_raw_str[j].name, CV_LOAD_IMAGE_COLOR);
				image_depth_scaled = imread(depth_raw_str[i].path + folder + depth_raw_str[j].name, CV_LOAD_IMAGE_UNCHANGED);
				//image_ir_scaled = imread(ir_raw_str[i].path + folder + ir_raw_str[j].name, CV_LOAD_IMAGE_UNCHANGED);

				//Mat image_rgb_raw_ = image_color_scaled.clone();
				//Mat image_depth_raw_ = image_depth_scaled.clone();
				//Mat image_ir_raw_ = image_ir_scaled.clone();

				time_m.push_back(Time_m{ "color stretch", (double)getTickCount() });
				cv::split(image_color_scaled, channels);
				
				//image_color_scaled.convertTo(image_color_scaled,alpha[l],beta[l]);
				BrightnessAndContrastAuto(channels[0], channels[0], clipHistPercent[l]);
				BrightnessAndContrastAuto(channels[1], channels[1], clipHistPercent[l]);
				BrightnessAndContrastAuto(channels[2], channels[2], clipHistPercent[l]);

				cv::merge(channels, image_color_scaled);
				time_m.back().end_t = (double)getTickCount();
				channels.clear();

				image_rgb_raw = image_color_scaled.clone();


				color = new IplImage(image_color_scaled);
				range = new IplImage(image_depth_scaled);
				memcpy(Depth_Ready, range->imageData, sizeScaled.width * sizeScaled.height * 2);
				cvSetData(range, range->imageData, range->widthStep);

				time_m.push_back(Time_m{ "background", (double)getTickCount() });
				hsvdetect->selectBackground(color, range, Depth_Ready);
				time_m.back().end_t = (double)getTickCount();
				//bool face_ = true;
				time_m.push_back(Time_m{ "face", (double)getTickCount() });
				bool face_ = hsvdetect->faceDetection(color, Depth_Ready);
				time_m.back().end_t = (double)getTickCount();
				time_m.push_back(Time_m{ "body", 0, 0 });
				time_m.push_back(Time_m{ "hsv color", 0, 0 });
				time_m.push_back(Time_m{ "search hand", 0, 0 });
				time_m.push_back(Time_m{ "fit hand", 0, 0 });
				bool hsv_ = false;
				if (face_ && true)					// 12 ms
				{

					time_m[time_m.size() - 4].start_t = (double)getTickCount();
					hsvdetect->selectBody(color, range, Depth_Ready);

					time_m[time_m.size() - 4].end_t = (double)getTickCount();

					time_m[time_m.size() - 3].start_t = (double)getTickCount();
					//bool hsv_ = hsvdetect->meanFaceColor(color, Depth_Ready);
					
					//if (k == 0)
					//{
					//	hsv_ = hsvdetect->meanFaceColor(color, Depth_Ready);
					//}
					//else if (k == 1)
					//{
					//	hsv_ = hsvdetect->meanStdFaceColor(color, Depth_Ready);
					//}
					if (k == 0)
					{
						hsv_ = hsvdetect->getSkinColor(color, Depth_Ready);
					}
					time_m[time_m.size() - 3].end_t = (double)getTickCount();

					//hsvdetect->savePicture(range, "depth_body.png");
					if (hsv_)			// 3 ms
					{
						//hsvdetect->savePicture(color, "color_hsv.jpeg");

						time_m[time_m.size() - 2].start_t = (double)getTickCount();
						hsvdetect->searchHand(Depth_Ready);						// 3,1 ms
						time_m[time_m.size() - 2].end_t = (double)getTickCount();

						//if (hsvdetect->searchObjekt(Depth_Ready))				// 4 ms
						//{
						//	hsvdetect->lineEstimationObject();					// 1,5 ms
						//	hsvdetect->depthEstimationObject(Depth_Ready);		// 0,023 ms
						//	//hsvdetect->drawObejctFrame(color);					// 0.01 ms

						//}
						//else
						//{
						time_m[time_m.size() - 1].start_t = (double)getTickCount();

							hsvdetect->lineEstimationHand();
							hsvdetect->depthEstimationHand(Depth_Ready);
							hsvdetect->searchHandArea(Depth_Ready);
							hsvdetect->drawHandFrame(color);
						////}
						hsvdetect->debugingWindow(color);			// 0,012 ms
						time_m[time_m.size() - 1].end_t = (double)getTickCount();

					}

				}
				else
				{
					hsvdetect->skin_pixel_.push_back(0);
				}

				hsvdetect->exportTrackingPoint(color, Depth_Ready);



				//cvShowImage("RGB SCALED", color);
				////cvShowImage("DEPTH REGISTERED",range);
				////cvShowImage("RGB 2",color_2);
				////
				if (face_ && hsvdetect->trackerData.isHand)
				{
					char text[30];
					sprintf(text, "Clip Percent: %.2f", clipHistPercent[l]);
					cout << text << endl;
					cvPutText(color, text, cvPoint(50, 100), &hsvdetect->font, CV_RGB(250, 0, 150));
					string path_ = string(imagesResultPath) + to_string(depth_raw_str[j].index) + "_" + to_string(clipHistPercent[l]) + "_result.png";
					//cvSaveImage(path_.c_str(), color);

					putText(image_rgb_raw, to_string(clipHistPercent[l]), Point(50, 50),1,1,1);
					//imwrite(string(imagesResultPath) + to_string(depth_raw_str[j].index) + "_" + to_string(clipHistPercent[l]) + "_color.png", image_rgb_raw);
					output.write(image_color_scaled);

					//if (l == 2 && imagePath != folder)
					//{
					//	imwrite(string(imagePath) + "color_" + to_string(depth_raw_str[j].index) + ".png", image_rgb_raw_);
					//	imwrite(string(imagePath) + "depth_" + to_string(depth_raw_str[j].index) + ".png", image_depth_raw_);
					//	imwrite(string(imagePath) + "ir_" + to_string(depth_raw_str[j].index) + ".png", image_ir_raw_);
					//	//imwrite(string(imagesResultPath) + "color_" + to_string(depth_raw_str[j].index) + "_" + to_string(clipHistPercent[l]) + ".png", image_rgb_raw);
					//}

				}
				face_ = false;

				now_t = (double)getTickCount();
				cout << "Zykluszeit:  " + to_string(((now_t - last_t) / getTickFrequency())) + " Sekunden \n";
				if (last_t > 0)
				{
					hsvProcess << hsvdetect->currentDateTime() << " ; " << hsvdetect->count << " ; ";
					for (int i = 0; i < time_m.size(); i++)
					{
						hsvProcess << to_string(((time_m[i].end_t - time_m[i].start_t) /  getTickFrequency())) << " ; ";
					}

					hsvProcess << to_string(((now_t - last_t) / getTickFrequency())) + " ; ";
					hsvProcess << endl;
				}
				else
				{
					hsvProcess << "Date ; count ; ";
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

				//libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
			}		
			skin_pixel.push_back( hsvdetect->skin_pixel_);
			hsvdetect->skin_pixel_.clear();
			output.release();
		}
		
		ofstream pixel(hsvdetect->logPath + "skin_pixel.txt");
		ofstream score(hsvdetect->logPath + "skin_score.txt");
		pixel << " index ; ";
		for (int l = 0; l < clipHistPercent.size(); l++)
		{
			pixel << "skin " + hsv_methode[k] + "_" + to_string(clipHistPercent[l]) << " ; ";
		}
		pixel << endl;
		for (int j = 0; j < color_raw_str.size(); j++)
		{
			pixel << color_raw_str[j].index << " ; ";
			for (int i = 0; i < skin_pixel.size(); i++)
			{
				//if ( skin_pixel[i].size() > j)
				if (skin_pixel[i][j] > 0)
				{
					pixel << skin_pixel[i][j] << " ; ";
				}
				else
				{
					pixel << 0 << " ; ";
				}
					
			}
			pixel << endl;
		}


		skin_pixel.clear();
		color_raw_str.clear();
		depth_raw_str.clear();
		ir_raw_str.clear();
	}

	system("Pause");
	//Sleep(10000000000000);
	// TODO: restarting ir stream doesn't work!
	// TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
	cvDestroyWindow("RGB SCALED");
	return 0;
}
