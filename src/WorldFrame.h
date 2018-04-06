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

namespace CoordinateFrame
{
/// @brief  Struct for extrisic Paramters of a specific Camera
/// @ingroup  HelperObjects
struct Ext_Camera{
	string uri; ///< Camera uri
	Mat tvecs;	///< Translation Vector
	Mat rvecs;	///< Rotation as Rodrigues Vector
	Mat tvecs_inv;	///< Translation Vector
	Mat rpy;		///< Roll Pitch Yaw Angles
	Mat rpy_inv;		///< Roll Pitch Yaw Angles inverse

	Ext_Camera operator=(const Ext_Camera &inPut) const
	{
		Ext_Camera outPut;
		outPut.uri = inPut.uri;
		outPut.tvecs = inPut.tvecs;
		outPut.rvecs = inPut.rvecs;
		outPut.tvecs_inv = inPut.tvecs_inv;
		outPut.rpy = inPut.rpy;
		outPut.rpy_inv = inPut.rpy_inv;
		return outPut;
	}

};

/// @brief  Object to calculate and visualize the Transformation between Camera and World
/// @ingroup  HelperObjects
	class WorldFrame
	{
	public:
		WorldFrame();	// Construktur
		~WorldFrame();	// Destruktur


		void init();	// Initialization									
		void calibWorldFrame(Mat& image);	// Calculating the external parameters, get Rotation and Translation between World and Camera
		void drawWorldFrame(Mat& image);	// Function for visualization of the world coordinate system in the current picture
		void drawWorldFrameEx(Mat& image, Mat rvecs_Ex, Mat tvecs_Ex); // Function for visualization of the external coordinate system in the current picture

		void writePoints(string uri, Mat R_Points, Mat W_Points, string calibPath, string calibFile);	// Function to write extrisic Paramertes in XML File
		void readPoints(string &uri, Mat &R_Points, Mat &W_Points, string calibPath, string calibFile); // Function to read extrisic Paramertes from XML File

		Mat estimateRigidTransformation(Mat RP, Mat WP);	// Function to estimate the translation and Rotation between two Point Sets
		void quat2Rot(Mat quat, Mat R);						// Function to convert a vector of quaternions to a rotation matrix
		vector<Mat> crossTimesMatrix(Mat R22_1);			// Produce a Cross Time Matrix

		Ext_Camera getTransforamtion(string calibPath, string calibFile);	// Function to get the Transformation to external Coordinate Frame from XML File

		Mat rot2rpy(Mat R_);
		Mat rpy2rot(Mat rpy);

		Mat K;						///< Camera Parameter Matrix
		Mat dist;					///< Distortion Parameter Vector
		int scale_img;				///< scale factor for calibration image
		int scale_dis;				///< scale faktor for displayed image

		string serial;				///< Camera Serial 
		string calibPath;			///< Path, where Parameter Files is saved
		string calibFile;			///< Name of the Parameter File

		Mat imgGray;				///< Grayimage for chessboard detection
		int points_per_row;			///< Points per Row - Chessboard
		int points_per_col;			///< Points per Column - Chessboard
		float squareSize;			///< length of square

		bool calibrate = false;		///< Bool to start the Calibration
		bool save = false;			///< Bool to save the Parameters
		bool abort = false;			///< Bool to abort the Calibration
		bool draw = false;			///< Boold to draw the Coordinate Frame

		bool front = true;			///< Bool to switch between the Corners of the Chessboard
		bool scale_display;			///< Bool to scale the window
		bool scale_image;			///< Bool to scale the image for calibration

		vector<Mat> rvecs;				///< Rotation between Chessboard and Camera (Rodrigues) 
		vector<Mat>  tvecs;				///< Translation between Chessboard and Camera

		void writeXML(string file, Mat tvec, Mat rpy)
		{
			FileStorage fs(file, FileStorage::WRITE);
			fs << "robToCheck";
			fs << "{";
			fs << "tvec" << tvec;
			fs << "rpy" << rpy;
			fs << "}";
		}

		void readXML(string file, Mat &tvec, Mat &rpy, Mat &rvec)
		{
			FileStorage fs(file, FileStorage::READ);
			FileNode r = fs["robToCheck"];
			r["tvec"] >> tvec;
			r["rvec"] >> rpy;

			Rodrigues(rpy2rot(rpy), rvec);
			cout << tvec << endl;
			cout << rpy << endl;
			cout << rvec << endl;
		}
	private:

		vector<Point2f> corners;		///< Vector with detected Corners
		Size chSize;					///< Chessboard Size
		bool isFoundCalibWorldFrame;	///< Bool Chessboard is found


		void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners); // Calculation of the referenc Chess board

	};
}