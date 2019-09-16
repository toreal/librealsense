

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include "opencv2/core.hpp"

#include "opencv2/imgproc.hpp"

#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
//#include <opencv2/video/background_segm.hpp>
#include <opencv2/bgsegm.hpp>

#include <string>
#include "calibrate.h"
#include "fm_ocr_scanner.hpp"
#include "stdafx.h"

//#include "GrabCutMF.h"

int runcalib(int mode);




Mat dcameraMatrix, ddistCoeffs;
std::vector<Mat> drvecs, dtvecs;
Mat cameraMatrix, distCoeffs;
std::vector<Mat> rvecs, tvecs;

bool binit = false;
Mat finit;

std::vector<cv::Point3f> prepared3DPoints;


void intersectLineWithPlane3D(const float* q,
	const float* v,
	const float* w,
	float* p,
	float& depth) {

	// Evaluate inner products.
	float n_dot_q = 0, n_dot_v = 0;
	for (int i = 0; i < 3; i++) {
		n_dot_q += w[i] * q[i];
		n_dot_v += w[i] * v[i];
	}

	// Evaluate point of intersection P.
	depth = (w[3] - n_dot_q) / n_dot_v;
	for (int i = 0; i < 3; i++)
		p[i] = q[i] + depth * v[i];
}


//Vec3d Nz(0,0,1);
//cv::Mat Ori = cv::Mat::zeros(3, 1, CV_64F);

void pose2d3d(float book, vector<Point2f>  inputs, vector<Point3f>& newpoints, vector<Mat> prvecs, vector<Mat> ptvecs, Mat pcameraMatrix, Mat  pdistCoeffs)
{
	Vec3d Nz(0, 0, 1);
	cv::Mat Ori = cv::Mat::zeros(3, 1, CV_64F);
	Ori.at<double>(2) = -book;

	int nsize = inputs.size();
	int k;

	Mat cam_dist_points = Mat(inputs);
	Mat cam_undist_points = Mat(inputs);
	undistortPoints(cam_dist_points, cam_undist_points, pcameraMatrix, pdistCoeffs);

	//準備 q, v, w 以求交點
	//q 為 camera center,即 (0,0,0) v 為 下面的值, w 為平面:W(1)*X + W(2)*Y +W(3)*Z = W(3) 
	float q[3], v[3], wz[4];
	Mat  pm = cv::Mat::zeros(3, 1, CV_64F);

	float ipoint[3];
	float depth;

	q[0] = 0; q[1] = 0; q[2] = 0;

	Mat R, RT, pO, Rz, Rx;
	Rodrigues(prvecs[0], R);


	gemm(R, Ori, 1, ptvecs[0], 1, pO, GEMM_3_T);

	Mat Mnz(Nz);


	gemm(R, Mnz, 1, NULL, 0, Rz);


	wz[0] = Rz.at<double>(0, 0);
	wz[1] = Rz.at<double>(1, 0);
	wz[2] = Rz.at<double>(2, 0);
	wz[3] = wz[0] * pO.at<double>(0, 0) +
		wz[1] * pO.at<double>(1, 0) +
		wz[2] * pO.at<double>(2, 0);



	for (int i = 0; i < nsize; i++) {
		Point2f pd = cam_undist_points.at<Point2f>(i);



		float norm = (float)sqrt(pow(pd.x, 2) + pow(pd.y, 2) + 1.0);
		v[0] = (float)pd.x / norm;
		v[1] = (float)pd.y / norm;
		v[2] = (float)1.0 / norm;



		intersectLineWithPlane3D(q, v, wz, ipoint, depth);



		cout << ipoint[0] << ipoint[1] << ipoint[2] << endl;


		//double sum = 0 ; 
		for (k = 0; k < 3; k++)
		{
			//	sum +=pow( old[k]-ipoint[k],2);

			pm.at<double>(k, 0) = (double)ipoint[k] - ptvecs[0].at<double>(0, k);


		}


		Mat Rinv = R.inv();
		Mat ipm = Rinv * pm;


		//check 
		cv::Mat tt = cv::Mat::zeros(3, 1, CV_64F);
		for (k = 0; k < 3; k++)
			tt.at<double>(k) = ipm.at<double>(k);
		gemm(R, tt, 1, ptvecs[0], 1, pO, GEMM_3_T);

		float x = pO.at<double>(0);
		float y = pO.at<double>(1);
		float z = pO.at<double>(2);

		cout << x / z << "," << y / z << endl;



		newpoints.push_back(Point3f(ipm.at<double>(0, 0), ipm.at<double>(1, 0), ipm.at<double>(2, 0)));


	}
}


void prepare3D(std::vector<cv::Point2f> bookPoints, float book)
{
	prepared3DPoints.clear();
	pose2d3d(book, bookPoints, prepared3DPoints, drvecs, dtvecs, dcameraMatrix, ddistCoeffs);

}


void drawIR(Mat src)
{


	/*
	
	std::vector<cv::Point2f> dprojectedPoints;
	if (prepared3DPoints.size() > 0)
	{

		dprojectedPoints.clear();
		try
		{
			cv::projectPoints(prepared3DPoints, drvecs[0], dtvecs[0], dcameraMatrix, ddistCoeffs, dprojectedPoints);
		}
		catch (exception e)
		{
			cout << e.what();
		}

		for (int k = 0; k < dprojectedPoints.size(); k++)
		{
			circle(src, dprojectedPoints.at(k), 5, Scalar(255));
		}

		imshow("mask", src);
	}
*/
}

Ptr<BackgroundSubtractor> pBackSub;
//Ptr<BackgroundSubtractor> pTrainedSub=NULL;

bool                      binitmog = false;
Mat                       mogMask;
//Mat                       mogBMask;


void initFrame(const rs2::video_frame& frame, int n)
{
	int h = frame.get_height();
	int w = frame.get_width();

	Mat colormat(Size(w, h), CV_8UC4, (void*)frame.get_data(), Mat::AUTO_STEP);

	cvtColor(colormat, finit, cv::COLOR_BGRA2BGR);
	//cvtColor(finit, finit, cv::COLOR_BGR2HSV);
	
	
	//finit = colormat.clone();
	if (!binitmog)
	{

		
		pBackSub = cv::bgsegm::createBackgroundSubtractorCNT(1,true,10,true);
		binitmog = true;

	}
	/*else if (n == 0)
	{
		memcpy( pBackSub, pTrainedSub, sizeof(cv::bgsegm::BackgroundSubtractorCNT));
		mogMask = mogBMask.clone();


	}*/


	//blur(finit, finit, Size(4, 4));
	//for(int i=0; i < 11; i++)
	//pBackSub->apply(finit, mogMask);

	/*if (n == 19)
	{
		if (pTrainedSub == (Ptr<BackgroundSubtractor>)NULL )
			pTrainedSub = cv::bgsegm::createBackgroundSubtractorCNT();

		memcpy(pTrainedSub, pBackSub, sizeof(cv::bgsegm::BackgroundSubtractorCNT));
		mogBMask = mogMask.clone();
	}*/

	/*Mat check = mogMask.clone();
	char retstr[1024];
	sprintf(retstr, "%d", n);
	putText(check, retstr, Point(100, 100), 0, 1, Scalar(255, 0, 0), 3);
	imshow("mask", check);*/

}


bool findCorner(Mat img, Mat mask, Point input, Point& output, int cornerType)
{

	Rect _rect(input.x - 50, input.y - 50, 100, 100);
	Mat subimg,submask;
	Mat backgroundModel = Mat(Size(65, 1), CV_64FC1);
	Mat foregroundModel = Mat(Size(65, 1), CV_64FC1);
	Rect rectangle(0, 0, 100, 100);

	img(_rect).copyTo(subimg);
	mask(_rect).copyTo(submask);

	grabCut(subimg, submask, rectangle,
		backgroundModel, foregroundModel,
		1, GC_INIT_WITH_MASK);

	Mat outputm = Mat::zeros(submask.rows, submask.cols, CV_8U);

	//cv::compare(submask, cv::GC_PR_FGD, outputm, cv::CMP_EQ);
	//cv::compare(submask, cv::GC_FGD, outputm, cv::CMP_EQ);


	for (int i = 0; i < submask.rows; ++i) {
		for (int j = 0; j < submask.cols; ++j) {
			// choose pixels that are certainly or likely background
			if (submask.at<uchar>(i, j) == GC_PR_FGD || submask.at<uchar>(i, j) == GC_FGD) {
				outputm.at<uchar>(i, j) = 255;
			}
		}
	}


	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> appros;
	std::vector<cv::Vec4i> hierarchy;
	int thresh = 100;
	Mat dst1, dst2, canny_output;

	Canny(outputm, canny_output, thresh, thresh * 2, 3);


	findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));
	std::vector<cv::Point> cnt = contours[0];
	//auto epsilon = 0.5 * arcLength(cnt, true);
	//appros.clear();
	//appros.resize(1);
	//approxPolyDP(cnt, appros[0], epsilon, false);
	//ApproxChains(cnt, appros[0], CHAIN_APPROX_SIMPLE );

	//Point2f vtx[4];
	vector<Point2f> vtx;
	if (cnt.size() > 0)
	{
		//RotatedRect box = minAreaRect(cnt);
		
		minEnclosingTriangle(cnt, vtx);
		//box.points(vtx);
		for (int i = 0; i < 3; i++)
			line(canny_output, vtx[i], vtx[(i + 1) % 3], Scalar(128), 1, LINE_AA);
		
	}
	

	

	//if (vtx.size()> 0 && appros[0].size() > 0)
	{
		float dis = FLT_MAX;
		for (int i = 0; i < 3; i++)
		{
			Point p =Point( vtx[i].x, vtx[i].y);
			Point del = p - Point(50, 50);

			float dd = del.dot(del);
			if (dd < dis)
			{
				dis = dd;
				output = p;
			}

			
		}


		circle(canny_output, output, 5, (250));
		
		imshow("grabCut", canny_output);
		return true;
	}

	return false;

}


bool colorBound(const rs2::video_frame& frame, float dep, bool bfinal)
{

	std::vector<cv::Point2f> projectedPoints;
	

	int h = frame.get_height();
	int w = frame.get_width();
	//frame.get_profile().

	bool bret = bfinal;








	Mat colormat(Size(w, h), CV_8UC4, (void*)frame.get_data(), Mat::AUTO_STEP),mf;

	cvtColor(colormat, mf, cv::COLOR_BGRA2BGR);

	auto rr = ProcessEdgeImage(mf, true);

	imshow("img1", std::get<2>(rr)[0]);
	imshow("img2", std::get<2>(rr)[1]);
	imshow("img3", std::get<2>(rr)[2]);
	imshow("img4", std::get<2>(rr)[3]);

	imwrite("out.png", std::get<2>(rr)[1]);


	if (!binit)
	{
		imwrite("C:\\james\\images\\f0.bmp", colormat);
	}


	if (!binit)
	{
		if (colormat.dims > 0 && colormat.rows > 0)
		{
			finit = colormat.clone();
			binit = true;
		}
		else
			std::cout << "mat error " << std::endl;


		//runcalib(0);
		//runcalib(1);

		const std::string outputFileName = "c:\\james\\images\\d_camera_data.xml";
		const std::string outputFileNamergb = "c:\\james\\images\\rgb_camera_data.xml";
		Size dimageSize;
		Size imageSize;


		if (!readCameraParams(outputFileName, dimageSize, dcameraMatrix, ddistCoeffs, drvecs, dtvecs))
		{
			std::cout << "error";

		}
		if (!readCameraParams(outputFileNamergb, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs))
		{
			std::cout << "error";

		}

	}
	else if (dep > 0 )
	{


		//	Mat  pm = cv::Mat::zeros(3, 1, CV_64F);
		//	Mat R;
		//	Rodrigues(drvecs[0], R);
		//
		//	//gemm(R, Ori, 1, tvecs[0], 1, O, GEMM_3_T);	
		//	Mat Rinv = R.inv();
		//	std::vector<cv::Point3f> cPoints;
		//	cPoints.push_back(Point3f(0, 0, 0));
		//	cPoints.push_back(Point3f(210, 0, 0));
		//	for (int i = 0; i < prepared3DPoints.size(); i++)
		//	{
		//
		//		pm.at<double>(0, 0) = (double)prepared3DPoints[i].x  - dtvecs[0].at<double>(0, 0);
		//		pm.at<double>(1, 0) = (double)prepared3DPoints[i].y  - dtvecs[0].at<double>(0, 1);
		//		pm.at<double>(2, 0) = (double)prepared3DPoints[i].z  - dtvecs[0].at<double>(0, 2);
		//
		//		Mat ipm = Rinv * pm;
		//
		//		cPoints.push_back(Point3f(ipm.at<double>(0),
		//			ipm.at<double>(1), -ipm.at<double>(2)));
		//		cPoints.push_back(Point3f(ipm.at<double>(0),
		//			ipm.at<double>(1), ipm.at<double>(2)));
		//		cPoints.push_back(Point3f(ipm.at<double>(0),
		//			ipm.at<double>(1),0));
		//
		//		/*cv::Mat tt = cv::Mat::zeros(3, 1, CV_64F);
		//		for (int k = 0; k < 3; k++)
		//			tt.at<double>(k) = ipm.at<double>(k);
		//		gemm(cR, tt, 1, tvecs[0], 1, P, GEMM_3_T);
		//*/
		//	}

		try
		{
			int nsize = prepared3DPoints.size();
			if (nsize ==4 && !bfinal)
			{

				//使用灰度图像进行角点检测	
				//cv::Mat image_gray;	cv::cvtColor(colormat, image_gray, cv::COLOR_BGR2GRAY);

				projectedPoints.clear();


				cv::projectPoints(prepared3DPoints, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, projectedPoints);


				Mat mf , fgMask, src;
				
				
				cvtColor(colormat, mf, cv::COLOR_BGRA2BGR);
				//src = mf.clone();
				auto rr =ProcessEdgeImage(mf, true);

				imshow("img1", std::get<2>(rr)[0]);
				imshow("img2", std::get<2>(rr)[1]);
				imshow("img3", std::get<2>(rr)[2]);
				imshow("img4", std::get<2>(rr)[3]);


				//blur(mf, mf, Size(4, 4));

				//if (pTrainedSub != (Ptr<BackgroundSubtractor>)NULL)
				//{
				//	memcpy(pBackSub, pTrainedSub, sizeof(cv::bgsegm::BackgroundSubtractorCNTImpl));
				//	mogMask = mogBMask.clone();
				//}

			/*	pBackSub->apply(src, mogMask);

				fgMask = mogMask.clone();
				Mat dst1, dst2, canny_output;
				Mat ele = getStructuringElement(cv::MORPH_DILATE, Size(50, 50));
				Mat ele2 = getStructuringElement(cv::MORPH_ERODE, Size(50, 50));				
				cv::erode(fgMask, dst2, ele2);
				cv:dilate(dst2, fgMask, ele);
				int thresh = 100;		

				Canny(fgMask, canny_output, thresh, thresh * 2, 3);
				imshow("colordist", canny_output);*/

				//cvtColor(mf, mf, cv::COLOR_BGR2HSV);
				
				//fgMask = mf.clone();
				//imwrite("curr.png", mf);
				//imwrite("bg.png", finit);

				//absdiff(mf, finit, fgMask);
				//for ( int i = 0 ; i < w; i++)
				//	for (int j = 0; j < h; j++)
				//	{
				//		Vec3b c1=mf.at<Vec3b>(j, i);
				//		Vec3b c2 = finit.at<Vec3b>(j, i);


				//		uchar d1 = c2[0] - c1[0];
				//		uchar d2 = c2[1] - c1[1];
				//		uchar d3 = c2[2] - c1[2];

				//		if ( (d1 < 50 && d2 < 50) || (d1 < 50 && d3 < 50 ) || (d2 < 50 && d3 < 50))
				//			fgMask.at<Vec3b>(j, i)=Vec3b(0,0,0);
				//		else
				//			fgMask.at<Vec3b>(j, i) = Vec3b(255,255,255);
				//		//{
				//		//	if (d1 < 128)
				//		//		d1 = d1 * 2;
				//		//	else
				//		//		d1 = 255;
				//		//	if (d2 < 128)
				//		//		d2 = d2 * 2;
				//		//	else
				//		//		d2 = 255;
				//		//	if (d3 < 128)
				//		//		d3 = d3 * 2;
				//		//	else
				//		//		d3 = 255;
				//		//	fgMask.at<Vec3b>(j, i) = Vec3b(d1, d2, d3);
				//		//	//cout << i << "," << j << endl;
				//		//}


				//	}



				//cvtColor(fgMask, fgMask, cv::COLOR_HSV2BGR);
				//imwrite("diff.png", fgMask);

				//fastNlMeansDenoisingColored(fgMask, mf, 10, 10, 7, 21);
				
				
				Mat mask = Mat(Size(w,h), CV_8UC1, Scalar(GC_BGD));
				Mat backgroundModel = Mat(Size(65, 1), CV_64FC1);
				Mat foregroundModel = Mat(Size(65, 1), CV_64FC1);
				


				Point root[1][4];
				int minx= w, maxx=0;
				int miny=h, maxy=0;

				for (int i = 0; i < projectedPoints.size(); i++)
				{
					root[0][i] =Point( projectedPoints[i].x, projectedPoints[i].y);
					if (projectedPoints[i].x < minx)
						minx = projectedPoints[i].x;
					if (projectedPoints[i].x > maxx)
						maxx = projectedPoints[i].x;
					if (projectedPoints[i].y < miny)
						miny = projectedPoints[i].y;
					if (projectedPoints[i].y > maxy)
						maxy = projectedPoints[i].y;



				}
				const Point* ppt[1] = { root[0] };
				int npt[] = { 4 };

				//polylines(mask, ppt, npt, 1, 1, Scalar(100), 100, 8, 0);
				fillPoly(mask, ppt, npt, 1, Scalar(cv::GC_FGD));
				polylines(mask, ppt, npt, 1, 1, Scalar(GC_PR_FGD), 30, 8, 0);
				//polylines(src, ppt, npt, 1, 1, Scalar(128,128,0), 30, 8, 0);
				//addWeighted(colormat, 0.5, src, 0.5, 0, colormat);
				Rect rectangle = Rect(minx, miny, maxx- minx, maxy - miny);

				std::vector<cv::Point2f> outPoints;
				for (int k = 0; k < projectedPoints.size(); k++)
				{
					Point out;
					bool bfind = findCorner(mf, mask, projectedPoints.at(k), out, k);

					if (bfind)
					{
						Point in = Point(projectedPoints.at(k).x, projectedPoints.at(k).y);
						Point res = in + out - Point(50, 50);
						outPoints.push_back(res);
					}
				}




				/*Mat imMat3f;
				double maxWeight = 2;
				mf.convertTo(imMat3f, CV_32FC3, 1 / 255.0);
				GrabCutMF  cutMF(imMat3f, mf, ".", 6, 10, 2, 20, 33, 3, 41);
				cutMF.initialize(rectangle, mask, (float)maxWeight, true);
				cutMF.refine();
				Mat ret = cutMF.drawResult();*/

				/*grabCut(mf, mask, rectangle,					
					backgroundModel, foregroundModel,
					1, GC_INIT_WITH_MASK);*/

				//Mat output =Mat::zeros(mask.rows, mask.cols, CV_8U);

				//for (int i = 0; i < mask.rows; ++i) {
				//	for (int j = 0; j < mask.cols; ++j) {
				//		// choose pixels that are certainly or likely background
				//		if (mask.at<uchar>(i, j) == GC_PR_FGD || mask.at<uchar>(i, j) == GC_FGD) {
				//			output.at<uchar>(i, j) = 255;
				//		}
				//	}
				//}
				//cv::compare(mask, cv::GC_PR_FGD, output, cv::CMP_EQ);
				//cv::compare(mask, cv::GC_FGD, output, cv::CMP_EQ);
				//fillPoly(mask, ppt, npt, 1, Scalar(255));
				//result = result & 1 ;	
				//cv::Mat foreground(mf.size() , CV_8UC3 ,		cv::Scalar(128 , 128 , 128)) ;	
				//mf.copyTo(foreground , mask) ;
				

				//imshow("grabCut", foreground);


				/*std::vector<std::vector<cv::Point>> contours;
				std::vector<std::vector<cv::Point>> appros;
				std::vector<cv::Vec4i> hierarchy;
				int thresh = 100;
				Mat dst1, dst2, canny_output;

				Canny(output, canny_output, thresh, thresh * 2, 3);

				
				findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

				if (contours.size() > 0)
				{
					std::vector<cv::Point> cnt = contours[0];
					auto epsilon = 0.1 * arcLength(cnt, true);
					appros.clear();
					appros.resize(1);
					approxPolyDP(cnt, appros[0], epsilon, true);
				}*/

				for (int i = 0; i < projectedPoints.size(); i++)
				{

					Point p = Point(projectedPoints[i].x, projectedPoints[i].y);
					Point pA = p + Point(-50, -50);
					Point pB = p + Point(50, 50);
					cv::rectangle(colormat, pA, pB,
						cv::Scalar(128, 128, 0), 2, 8, 0);




				}

				if (outPoints.size() > 0)// && appros[0].size() == 4)
				{
					/*vector<Point2f>  inputs;

					for (int i = 0; i < 4; i++)
					{
						inputs.push_back(Point2f(appros[0][i].x, appros[0][i].y));
					}
*/
					prepared3DPoints.clear();
					pose2d3d(dep, outPoints, prepared3DPoints, rvecs, tvecs, cameraMatrix, distCoeffs);

				}

				int nsize = prepared3DPoints.size();

				float bw = FLT_MAX;
				float bh = FLT_MAX;
				float bd = dep;

				for (int i = 0; i < nsize; i++)
				{
					Point3f a = prepared3DPoints.at(i);
					Point3f b = prepared3DPoints.at((i + 1) % nsize);

					Point3f c = a - b;
					float dis = sqrtf(c.dot(c));
					if (i % 2)
					{
						if (dis < bw)
						{
							bw = dis;
						}
					}
					else
					{
						if (dis < bh)
							bh = dis;

					}




				}



				


				for (int i = 0; i < outPoints.size(); i++)
				{

					Point p = outPoints[i];
					
					circle(colormat, p, 10, cv::Scalar(0, 128, 128));
						

				}

				projectedPoints.clear();


				cv::projectPoints(prepared3DPoints, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, projectedPoints);



				//Mat fgMask, d1, d2, c1, c2;
				////cvtColor(colormat, c1, cv::COLOR_BGR2HSV);
				////cvtColor(finit, c2, cv::COLOR_BGR2HSV);
				//absdiff(colormat, finit, fgMask);
				////cvtColor(fgMask, c1, cv::COLOR_HSV2BGR);
				//cvtColor(fgMask, d1, cv::COLOR_BGR2GRAY);
				////threshold(d2, d1, 50, 255.0, cv::THRESH_BINARY);
				////Mat ele = getStructuringElement(cv::MORPH_DILATE, Size(10, 10));
				////Mat ele2 = getStructuringElement(cv::MORPH_ERODE, Size(10, 10));
				////cv::erode(d1, d2, ele2);
				////cv:dilate(d2, d1, ele);


				////设置角点检测参数	
				//std::vector<cv::Point2f> corners;
				//int max_corners = 200;
				//double quality_level = 0.01;
				//double min_distance = 3.0;
				//int block_size = 3;
				//bool use_harris = false;
				//double k = 0.04;
				////角点检测	
				//cv::goodFeaturesToTrack(d1,
				//	corners, max_corners, quality_level, min_distance,
				//	cv::Mat(), block_size, use_harris, k);
				////将检测到的角点绘制到原图上	
				//for (int i = 0; i < corners.size(); i++)
				//{
				//	cv::circle(colormat, corners[i], 1, cv::Scalar(0, 0, 255), 2, 8, 0);
				//}

				for (int i = 0; i < projectedPoints.size(); i++)
				{
					cv::circle(colormat, projectedPoints[i], 1, cv::Scalar(128, 0, 255), 2, 8, 0);
				}

				

				char retstr[10240];


				snprintf(retstr, sizeof(retstr), "W: %f H: %f D: %f", bw, bh, bd);

				putText(colormat, retstr, Point(280, 280), 0, 1, Scalar(0, 0, 0), 3);


				if (nsize == 4)
					bret = true;

				imshow("mask", mogMask);
				imshow("color", colormat);
			}
		}
		catch (exception e)
		{
			cout << e.what();

		}

	}

	return bret;
}



