
#include "VideoStreamer.h"


const string current_DateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);
	return buf;
}

void VideoStreamer::video_recorder(const String& video_path, Mat img, double fps, type video_type, bool recorde_flag){
	if (recorde_flag){
		switch (video_type)
		{
		case COLOR:
			filename = video_path + "color_" + current_DateTime() + ".avi";
			if (!color_writer.isOpened()){
				color_writer.open(filename, CV_FOURCC('D', 'I', 'B', ' '), fps, img.size());
			}
			color_writer.write(img);
			break;
		case DEPTH:
			filename = video_path + "depth_" + current_DateTime() + ".avi";
			if (!depth_writer.isOpened()){
				depth_writer.open(filename, CV_FOURCC('D', 'I', 'B', ' '), fps, img.size());
			}
			depth_writer.write(img);
			break;
		case IR:
			filename = video_path + "ir_" + current_DateTime() + ".avi";
			if (!ir_writer.isOpened()){
				ir_writer.open(filename, CV_FOURCC('D', 'I', 'B', ' '), fps, img.size());
			}
			ir_writer.write(img);
			break;
		}
	}
	else
	{
		switch (video_type)
		{
		case COLOR:
			if (color_writer.isOpened()){
				color_writer.release();
			}
			break;
		case DEPTH:
			if (depth_writer.isOpened()){
				depth_writer.release();
			}
			break;
		case IR:
			if (ir_writer.isOpened()){
				ir_writer.release();
			}
			break;
		}
	}
}

/*
void VideoStreamer::color_video_recorder(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false){
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


void VideoStreamer::depth_video_recorder(const String& filename, Mat img, double fps, bool isColor = true, bool recorde_flag = false){
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
*/

void VideoStreamer::video_play(const String& file_name, Mat &frame, bool read_flag){
	if (read_flag){
		if (!video_reader.isOpened())
			video_reader.open(file_name);
		video_reader >> frame;
	}
	//close video writer
	else if (!read_flag){
		if (video_reader.isOpened()){
			video_reader.release();
		}
	}
}