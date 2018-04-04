/*

 * kinect2_calib.cpp
 *
 *  Created on: 28.08.2015
 *      Author: krusche
 */

#include "kinect2_calib.h"


/**
Function to load the Camera and Distortion Matrix from File
@param [in] filename Name of File
@param [out] cameraMatrix Matrix K with the Parameters (focal length and image center point)
@param [out] distortion Matrix with the Parameters for Distortion (5x1)
@return is succesful
*/
bool kinect2Calib::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion)
{
	FileStorage fs;
	if (fs.open(filename, cv::FileStorage::READ))
	{
		fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
		fs[K2_CALIB_DISTORTION] >> distortion;
		fs.release();
	} else
	{
		std::cerr << "can't open calibration file: " << filename << std::endl;
		return false;
	}
	return true;
}

/**
Function to load the Rotation and Translation between RGB and Depth camera
@param [in] filename Name of File
@param [out] rotation Rotation Matrix
@param [out] translation Translation between RGB and Depth Camera
@return is succesful
*/
bool kinect2Calib::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation)
{
	cv::FileStorage fs;
	if (fs.open(filename, cv::FileStorage::READ))
	{
		fs[K2_CALIB_ROTATION] >> rotation;
		fs[K2_CALIB_TRANSLATION] >> translation;
		fs.release();
	} else
	{
		std::cerr << "can't open calibration pose file: " << filename << std::endl;
		return false;
	}
	return true;
}

/**
Function to load the Shift Parameter for Depth Camera
@param [in] filename Name of File
@param [out] depthShift Value for Depth Shift
@return is succesful
*/
bool kinect2Calib::loadCalibrationDepthFile(const std::string &filename, double &depthShift)
{
	cv::FileStorage fs;
	if (fs.open(filename, cv::FileStorage::READ))
	{
		fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
		fs.release();
	} else
	{
		std::cerr << "can't open calibration depth file: " << filename << std::endl;
		return false;
	}
	return true;
}



