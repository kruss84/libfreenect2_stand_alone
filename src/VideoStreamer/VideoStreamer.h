#include <cstdio>
#include <iostream>
#include "cv.h"
#include "highgui.h"

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class VideoStreamer
	{
	public:
		enum type{
			COLOR = 0,
			DEPTH,
			IR
		};



		void video_recorder(const String& video_path, Mat img, double fps, type, bool recorde_flag = false);

		bool video_play(const String& file_name, Mat &frame, bool read_flag = true);
		//void color_video_recorder(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false);
		//void depth_video_recorder(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false);

	private:
		VideoWriter color_writer;
		VideoWriter depth_writer;
		VideoWriter ir_writer;
		VideoCapture video_reader;
		string filename;

	};

