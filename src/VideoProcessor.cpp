
#include "videoprocessor.h"

void VideoProcessor::color_video_output(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false){
	if (recorde_flag){
		if (!color_writer.isOpened()){
			color_writer.open(filename, CV_FOURCC('D', 'I', 'B', ' '), fps, img.size(), isColor);
			//cout << "isOpened" << color_writer.isOpened() << endl;
		}
		color_writer.write(img);
	}
	//close video writer
	else if (!recorde_flag){
		if (color_writer.isOpened()){
			color_writer.release();
		}
	}
}

void VideoProcessor::depth_video_output(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false){
	if (recorde_flag){
		if (!depth_writer.isOpened()){
			depth_writer.open(filename, CV_FOURCC('D', 'I', 'B', ' '), fps, img.size(), isColor);
			//cout << "isOpened" << color_writer.isOpened() << endl;
		}
		depth_writer.write(img);
	}
	//close video writer
	else if (!recorde_flag){
		if (depth_writer.isOpened()){
			depth_writer.release();
		}
	}
}

void VideoProcessor::video_input(const String& filename, Mat &frame, bool read_flag = true){

}