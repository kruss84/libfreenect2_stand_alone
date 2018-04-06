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

class VideoProcessor
{
public:
	enum type
	{
		COLOR = 0;
		DEPTH;
		IR;
	};

	VideoProcessor();
	~VideoProcessor();

	void video_input(const String& filename, Mat &frame, bool read_flag = true);
	void color_video_output(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false);
	void depth_video_output(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false);
	
private:
	VideoWriter color_writer;
	VideoWriter depth_writer;
	VideoCapture video_reader;

};

VideoProcessor::VideoProcessor()
{
}

VideoProcessor::~VideoProcessor()
{
}
