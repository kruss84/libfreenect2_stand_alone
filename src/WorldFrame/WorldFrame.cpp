
#include "worldframe.h"


namespace CoordinateFrame
{
	WorldFrame::WorldFrame(){};
	WorldFrame::~WorldFrame(){};


	/**
	Initialisation of the Object
	- check all Paramertes
	- load  Parameters from XML File
	*/
	void WorldFrame::init()
	{
		if (K.empty() || dist.empty())
		{
			//cout << "[WorldFrame] Default Calibration Parameter." << endl;
			//K = Mat::eye(3, 3, CV_64F);
			//dist = Mat::zeros(1, 5, CV_64F);
		}
		else
		{
			//cout << "[WorldFrame] K = " << K << endl;
			//cout << "[WorldFrame] dist = " << dist << endl;
		}


		if (serial == "")
		{
			//cout << "[WorldFrame] Could not read serial number." << endl;
		}
		else
		{
			//cout << "[WorldFrame] Serialnumber: " << serial << endl;

		}

		if (calibPath == "" || calibFile == "")
		{
			//cout << "[WorldFrame] Could not read calibration path or file." << endl;
		}
		else
		{
			//cout << "[WorldFrame] Calibration path and file: " << calibPath + '/' + calibFile << endl;
			fprintf(stderr, "[WorldFrame] Trying to save parameters into file.\n");

			string calibDest = calibPath + '/' + calibFile;
			////cout << calibDest << endl;

			FileStorage fs(calibDest, FileStorage::READ);
			fs["r_C"] >> rvecs;
			fs["t_C"] >> tvecs;
			fs.release();

			//cout << "t = " << tvecs[0] << endl;
			//cout << "r = " << rvecs[0] << endl;
		}
	}

	/**
	Function to get the Transformation to external Coordinate Frame from XML File
	@param [in] calibPath Path, where Parameter Files is saved
	@param [in] calibFile Name of the Parameter File
	*/
	Ext_Camera WorldFrame::getTransforamtion(string calibPath, string calibFile)
	{
		Ext_Camera ext_camera;
		Mat WP, RP, T, T_inv;
		readPoints(ext_camera.uri, RP, WP, calibPath, calibFile);
		T = estimateRigidTransformation(RP, WP);
		T_inv = T.inv();
		//writePoints(ext_camera.uri, RP, WP, calibPath, calibFile);
		ext_camera.tvecs = T(Rect(3, 0, 1, 3));
		ext_camera.tvecs_inv = T_inv(Rect(3, 0, 1, 3));
		Mat R = T(Rect(0, 0, 3, 3));
		Rodrigues(R, ext_camera.rvecs);

		ext_camera.rpy = rot2rpy(R);
		cout << "RPY: " << ext_camera.rpy << endl;
		ext_camera.rpy_inv = rot2rpy(T_inv(Rect(0, 0, 3, 3))); 
		cout << "t: " << ext_camera.tvecs_inv << endl;
		//cout << "rodrigues: " << ext_camera.rvecs*180/3.14 << endl;

		return ext_camera;
	}

	/**
	Function for visualization of the world coordinate system in the current picture
	@param [in] image current image
	*/
	void WorldFrame::drawWorldFrame(Mat& image)
	{
		if (draw)
		{
			// Koordinatenursprung des Weltsystems im Bild darstellen
			Mat frameW = Mat::zeros(4, 3, CV_32F);
			frameW.at<float>(1, 0) = 0.3;
			frameW.at<float>(2, 1) = 0.3;
			frameW.at<float>(3, 2) = 0.3;
			vector<Point2f> imgPnts;
			projectPoints(frameW, rvecs[0], tvecs[0], K, dist, imgPnts);
			line(image, imgPnts[0], imgPnts[1], Scalar(0, 0, 255), 2);  // X-Achse (Rot)
			line(image, imgPnts[0], imgPnts[2], Scalar(0, 255, 0), 2);  // Y-Achse (Grün)
			line(image, imgPnts[0], imgPnts[3], Scalar(255, 0, 0), 2);  // Z-Achse (Blau)
		}
		return;
	}

	/**
	Function for visualization of the external coordinate system in the current picture
	@param [in] image current image
	@param [in] rvecs_Ex Rotation as Rodrigues Vector
	@param [in] tvecs_Ex Translation Vector
	*/
	void WorldFrame::drawWorldFrameEx(Mat& image, Mat rvecs_Ex, Mat tvecs_Ex)
	{
		if (draw)
		{
			// Koordinatenursprung des Weltsystems im Bild darstellen
			Mat frameW = Mat::zeros(4, 3, CV_32F);
			frameW.at<float>(1, 0) = 0.3;
			frameW.at<float>(2, 1) = 0.3;
			frameW.at<float>(3, 2) = 0.3;
			vector<Point2f> imgPnts;

			Mat R_rvecs;
			Rodrigues(rvecs[0], R_rvecs);
			Mat R_rvecs_Ex;
			Rodrigues(rvecs_Ex, R_rvecs_Ex);

			//		//cout << "2 " << 

			Mat tvecs_new = tvecs[0] + R_rvecs*tvecs_Ex;
			////cout << "tvecs " << tvecs[0] << endl;
			////cout << "tvecs_new " << tvecs_new << endl;
			Mat rvecs_new = (Mat_<double>(1, 3) << 0, 0, 0);
			Rodrigues(R_rvecs*R_rvecs_Ex, rvecs_new);

			projectPoints(frameW, rvecs_new, tvecs_new, K, dist, imgPnts);
			line(image, imgPnts[0], imgPnts[1], Scalar(0, 0, 255), 2);  // X-Achse (Rot)
			line(image, imgPnts[0], imgPnts[2], Scalar(0, 255, 0), 2);  // Y-Achse (Grün)
			line(image, imgPnts[0], imgPnts[3], Scalar(255, 0, 0), 2);  // Z-Achse (Blau)
		}
		return;
	}


	/**
	Calculating the external parameters, get Rotation and Translation between World and Camera
	@param [in] image current image
	*/
	void WorldFrame::calibWorldFrame(Mat& image)
	{
		if (calibrate == true)
		{


			cvtColor(image, imgGray, CV_BGR2GRAY);
			chSize = Size(points_per_row, points_per_col);
			//cout << chSize << endl;
			isFoundCalibWorldFrame = findChessboardCorners(imgGray, chSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);



			if (isFoundCalibWorldFrame) {

				// Subpixelgenauigkeit
				cornerSubPix(imgGray, corners, Size(6, 6), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				/*
				for (int i = 0; i < corners.size(); i++)
				{
				//cout << "x: " << corners.at(i).x << "y: " << corners.at(i).y << endl;
				}*/
				if (front == false)
				{
					reverse(corners.begin(), corners.end());
				}

				// Visualisierung
				drawChessboardCorners(imgGray, chSize, Mat(corners), isFoundCalibWorldFrame);


				// Rotation und Translation der Kamera gegenüber dem CalibWorldFrame herausfinden
				vector<vector<Point2f> > imagePoints(1);
				vector<vector<Point3f> > objectPoints(1);

				calcChessboardCorners(chSize, squareSize, objectPoints[0]);

				imagePoints[0] = corners;

				double rms = calibrateCamera(objectPoints, imagePoints, Size(imgGray.cols, imgGray.rows), K, dist, rvecs, tvecs,
					CV_CALIB_FIX_FOCAL_LENGTH | CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_INTRINSIC |
					CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

				fprintf(stdout, "Found CalibWorldFrame and calculated a translation and rotation. The rms of calibrateCamera is %0.3f\n", rms);

				//cout << "t = " << tvecs[0] << endl;
				//cout << "r = " << rvecs[0] << endl;

				// Für User-Ausgabe Bild wieder als Farbbild darstellen
				cvtColor(imgGray, image, CV_GRAY2BGR);

				// Koordinatenursprung des Weltsystems im Bild darstellen
				Mat frameW = Mat::zeros(4, 3, CV_32F);
				frameW.at<float>(1, 0) = 0.3;
				frameW.at<float>(2, 1) = 0.3;
				frameW.at<float>(3, 2) = 0.3;
				vector<Point2f> imgPnts;
				projectPoints(frameW, rvecs[0], tvecs[0], K, dist, imgPnts);
				line(image, imgPnts[0], imgPnts[1], Scalar(0, 0, 255), 2);  // X-Achse (Rot)
				line(image, imgPnts[0], imgPnts[2], Scalar(0, 255, 0), 2);  // Y-Achse (Grün)
				line(image, imgPnts[0], imgPnts[3], Scalar(255, 0, 0), 2);  // Z-Achse (Blau)

				circle(image, corners.front(), 5, Scalar(255, 0, 0), 2, 8, 0);
				circle(image, corners.back(), 5, Scalar(255, 0, 0), 2, 8, 0);
				//putText(image, "(f)", corners.front(), FONT_HERSHEY_PLAIN, 1.3 * 2, Scalar(255, 0, 0), 2, 8);
				//putText(image, "(b)", corners.back(), FONT_HERSHEY_PLAIN, 1.3 * 2, Scalar(255, 0, 0), 2, 8);

				// Optionen im Bild einblenden
				putText(image, "(s)ave, (a)bort, (c)hange", Point(5, 30), FONT_HERSHEY_PLAIN, 1.6 * scale_dis, Scalar(255, 0, 0), 2, 8);

			}
			if (scale_display) resize(image, image, Size(image.cols / scale_dis, image.rows / scale_dis), 0, 0, CV_INTER_LINEAR);
			string window_name = "World Calibration: " + serial;
			namedWindow(window_name, CV_WINDOW_AUTOSIZE);
			imshow(window_name, image);
			int iKey = (waitKey(1) & 255);

			// Esc,'a' ... Abbruch.
			if ((iKey == 27) || (iKey == 97) || abort == true)
			{
				calibrate = false;
				destroyWindow(window_name);
				return;
			}

			// "s" ... Bild in vordefinierten Pfad speichern.
			if (iKey == 115 || save == true) {
				fprintf(stderr, "Trying to save parameters into file.\n");

				string calibDest = calibPath + '/' + calibFile;
				//cout << calibDest << endl;

				FileStorage fs(calibDest, FileStorage::WRITE);
				fs << "r_C" << rvecs[0];
				fs << "t_C" << tvecs[0];
				fs.release();
				calibrate = false;
				destroyWindow(window_name);
			}

			if (iKey == 'c' && front == false)
			{
				front = true;
			}
			else if (iKey == 'c' && front == true)
			{
				front = false;
			}
		}

		return;
	}

	/**
	Calculation of the referenc Chess board
	@param [in] boardSize Rows and Columns of the Chessboard
	@param [in] squareSize Length of the square
	@param [out] corners Coordinates of the Chessboard Corners
	*/
	void WorldFrame::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
	{
		corners.resize(0);

		for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float(j*squareSize),
			float(i*squareSize), 0));
	}
	
	/**
	Function to form Cross Time Matrix
	@param [in] R22_1 Matrix
	*/
	vector<Mat> WorldFrame::crossTimesMatrix(Mat R22_1)
	{
		vector<Mat> V_times;
		for (int i = 0; i < R22_1.cols; i++)
		{
			Mat v_times = Mat::zeros(R22_1.rows, 3, R22_1.type());

			Mat r22_11 = -R22_1(Rect(i, 2, 1, 1));
			r22_11.copyTo(v_times(Rect(1, 0, 1, 1)));
			R22_1(Rect(i, 1, 1, 1)).copyTo(v_times(Rect(2, 0, 1, 1)));

			R22_1(Rect(i, 2, 1, 1)).copyTo(v_times(Rect(0, 1, 1, 1)));
			Mat r22_12 = -R22_1(Rect(i, 0, 1, 1));
			r22_12.copyTo(v_times(Rect(2, 1, 1, 1)));

			Mat r22_13 = -R22_1(Rect(i, 1, 1, 1));
			r22_13.copyTo(v_times(Rect(0, 2, 1, 1)));
			R22_1(Rect(i, 0, 1, 1)).copyTo(v_times(Rect(1, 2, 1, 1)));

			V_times.push_back(v_times);
			////cout << "v " << v_times << endl;
		}
		return V_times;
	}

	/**
	Function to convert a vector of quaternions to a rotation matrix
	@param [in] quat Vector with Quaternions
	@param [out] R Rotation Matrix
	*/
	void WorldFrame::quat2Rot(Mat quat, Mat R)
	{
		R = Mat::zeros(3, 3, quat.type());

		R.at<double>(0, 0) = quat.at<double>(0) * quat.at<double>(0) + quat.at<double>(1) * quat.at<double>(1)
			- quat.at<double>(2) * quat.at<double>(2) - quat.at<double>(3) * quat.at<double>(3);
		R.at<double>(0, 1) = 2 * (quat.at<double>(1) * quat.at<double>(2) - quat.at<double>(0) * quat.at<double>(3));
		R.at<double>(0, 2) = 2 * (quat.at<double>(1) * quat.at<double>(3) + quat.at<double>(0) * quat.at<double>(2));

		R.at<double>(1, 0) = 2 * (quat.at<double>(1) * quat.at<double>(2) + quat.at<double>(0) * quat.at<double>(3));
		R.at<double>(1, 1) = quat.at<double>(0) * quat.at<double>(0) - quat.at<double>(1) * quat.at<double>(1)
			+ quat.at<double>(2) * quat.at<double>(2) - quat.at<double>(3) * quat.at<double>(3);
		R.at<double>(1, 2) = 2 * (quat.at<double>(2) * quat.at<double>(3) - quat.at<double>(0) * quat.at<double>(1));

		R.at<double>(2, 0) = 2 * (quat.at<double>(1) * quat.at<double>(3) - quat.at<double>(0) * quat.at<double>(2));
		R.at<double>(2, 1) = 2 * (quat.at<double>(2) * quat.at<double>(3) + quat.at<double>(0) * quat.at<double>(1));
		R.at<double>(2, 2) = quat.at<double>(0) * quat.at<double>(0) - quat.at<double>(1) * quat.at<double>(1)
			- quat.at<double>(2) * quat.at<double>(2) + quat.at<double>(3) * quat.at<double>(3);
	}

	/**
	Function to estimate the translation and Rotation between two Point Sets
	@param [in] RP Source Point Set
	@param [in] WP Destination Point Set
	*/
	Mat WorldFrame::estimateRigidTransformation(Mat RP, Mat WP)
	{

		Mat rp_center, wp_center;
		reduce(RP, rp_center, 0, CV_REDUCE_AVG);
		reduce(WP, wp_center, 0, CV_REDUCE_AVG);

		Mat rp_center_, wp_center_;
		repeat(rp_center, RP.rows, 1, rp_center_);
		repeat(wp_center, WP.rows, 1, wp_center_);

		Mat rp_centered(rp_center_), wp_centered(wp_center_);
		subtract(RP, rp_center_, rp_centered);
		subtract(WP, wp_center_, wp_centered);

		Mat R12 = wp_centered - rp_centered;
		Mat R21 = rp_centered.t() - wp_centered.t();
		Mat R22_1 = wp_centered.t() + rp_centered.t();

		vector<Mat> V_times = crossTimesMatrix(R22_1);

		////cout << "V_times Size: " << V_times.size() << endl;
		Mat B = Mat::zeros(4, 4, R22_1.type());
		Mat a = Mat::zeros(4, 4, R22_1.type());
		vector<Mat> A;

		for (int i = 0; i < V_times.size(); i++)
		{
			Mat a = Mat::zeros(4, 4, R22_1.type());
			R12(Rect(0, i, 3, 1)).copyTo(a(Rect(1, 0, 3, 1)));
			R21(Rect(i, 0, 1, 3)).copyTo(a(Rect(0, 1, 1, 3)));
			V_times[i].copyTo(a(Rect(1, 1, 3, 3)));
			A.push_back(a);
			B += a.t() * a;
		}

		SVD svd;
		Mat W, S, V, vt;
		svd.compute(B, S, W, V, SVD::FULL_UV);

		Mat quat = W(Rect(3, 0, 1, 4));

		Mat R = Mat::zeros(3, 3, quat.type());
		;
		quat2Rot(quat, R);

		Mat T1 = Mat::eye(4, 4, R22_1.type());
		Mat T2 = Mat::eye(4, 4, R22_1.type());
		Mat T3 = Mat::eye(4, 4, R22_1.type());

		Mat wp_center_t(-wp_center.t());
		wp_center_t.copyTo(T1(Rect(3, 0, 1, 3)));
		R.copyTo(T2(Rect(0, 0, 3, 3)));
		Mat rp_center_t(rp_center.t());
		rp_center_t.copyTo(T2(Rect(3, 0, 1, 3)));

		Mat T = T3 * T2 * T1;

		////cout << "RP: " << RP << endl;
		////cout << "WP: " << WP << endl;
		////cout << "T: " << T << endl;

		return T;
	}

	/**
	Function to write extrisic Paramertes in XML File
	@param [in] uri Camera Uri
	@param [in] R_Points Source Point Set
	@param [in] W_Points Destination Point Set
	@param [in] calibPath Path, where Parameter Files is saved
	@param [in] calibFile Name of the Parameter File
	*/
	void WorldFrame::writePoints(string uri, Mat R_Points, Mat W_Points, string calibPath, string calibFile)
	{
		string calibDest = calibPath + '/' + calibFile;
		////cout << calibDest << endl;

		FileStorage fs(calibDest, FileStorage::WRITE);
		fs << "Camera_uri" << uri;
		fs << "R_Points" << R_Points;
		fs << "W_points" << W_Points;
		fs.release();

		return;
	}

	/**
	Function to write extrisic Paramertes in XML File
	@param [out] uri Camera Uri
	@param [out] R_Points Source Point Set
	@param [out] W_Points Destination Point Set
	@param [in] calibPath Path, where Parameter Files is saved
	@param [in] calibFile Name of the Parameter File
	*/
	void WorldFrame::readPoints(string &uri, Mat &R_Points, Mat &W_Points, string calibPath, string calibFile)
	{
		string calibDest = calibPath + '/' + calibFile;
		cout << calibDest << endl;

		FileStorage fs(calibDest, FileStorage::READ);


		fs["Camera_uri"] >> uri;
		fs["R_Points"] >> R_Points;
		fs["W_points"] >> W_Points;
		fs.release();

		//cout << "URI: " << uri << endl;
		//cout << "RP: " << R_Points << endl;
		//cout << "WP: " << W_Points << endl;

		return;
	}

	Mat WorldFrame::rot2rpy(Mat R_)
	{
		double deg = 180 / 3.14;
		double yaw = atan2(R_.at<double>(1, 0), R_.at<double>(0, 0))*deg;
		double pitch = atan2(-R_.at<double>(2, 0), sqrt(R_.at<double>(2, 1)*R_.at<double>(2, 1) + R_.at<double>(2, 2)*R_.at<double>(2, 2)))*deg;
		double roll = atan2(R_.at<double>(2, 1), R_.at<double>(2, 2))*deg;

		//cout << "Roll " << roll << endl;
		//cout << "Pitch " << pitch << endl;
		//cout << "Yaw " << yaw << endl;

		return (Mat_<double>(1,3) << roll, pitch, yaw);
	}

	Mat WorldFrame::rpy2rot(Mat rpy)
	{
		double deg = 3.14 / 180;
		double alpha = rpy.at<double>(0, 2)*deg;
		double beta = rpy.at<double>(0, 1)*deg;
		double gamma = rpy.at<double>(0, 0)*deg;

		Mat Rot = (Mat_<double>(3, 3) <<
			cos(alpha)*cos(beta), cos(alpha)*sin(beta)*sin(gamma) - sin(alpha)*cos(gamma), cos(alpha)*sin(beta)*cos(gamma) + sin(alpha)*sin(gamma),
			sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - cos(alpha)*sin(gamma),
			-sin(beta),			  cos(beta)*sin(gamma),									    cos(beta)*cos(gamma));
		//cout << Rot << endl;
		return Rot;

	}


}

