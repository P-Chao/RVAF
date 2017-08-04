/* SurfDetect
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1. The program use surf to detect feature points
// and generate feature descriptor. You can also directly sample image from video
// or camera, a webcamera is necessary if you run the program on camera mode.
// The program can mosaic two image and draw the match point, you can see usage
// and input the parameters by command line. The surf detect part referenced on
// OpenSurt(https://github.com/amarburg/opencv-ffi-ext/tree/master/ext/opensurf)
// project, and the core of code write by C. The test shows the program is more
// efficiency than OpenSurf. Enjoy it!
// Created at: 26 Aug. 2016, all rights reserved.*/


#include "common.h"
#include <opencv2\opencv.hpp>
#include <time.h>
#include <string>

using namespace std;
using namespace cv;

#include <Windows.h>

namespace pc{

LARGE_INTEGER m_liPerfFreq_;
LARGE_INTEGER m_liPerfStart_;
LARGE_INTEGER liPerfNow_;
double dfTim_;

void StartWatchTimer_(){
	QueryPerformanceFrequency(&m_liPerfFreq_);
	QueryPerformanceCounter(&m_liPerfStart_);
}

void ReadWatchTimer_(){
	QueryPerformanceCounter(&liPerfNow_);
	dfTim_ = (((liPerfNow_.QuadPart - m_liPerfStart_.QuadPart) * 1000.0f) / m_liPerfFreq_.QuadPart);
	printf("%f ms\n", dfTim_);
}

extern AlgParam algparam;
void imsave(string basename, cv::Mat image){
	string dir("./imgs/");
	if (algparam.saveimage){
		char filename[21];
		time_t rawtime;
		struct tm * t_tm;
		time(&rawtime);
		t_tm = localtime(&rawtime);
		int it = clock() % 1000;

		sprintf(filename, "_%02d%02d%02d%02d%02d%02d%03d.jpg",
			t_tm->tm_year - 100, t_tm->tm_mon + 1, t_tm->tm_mday,
			t_tm->tm_hour, t_tm->tm_min, t_tm->tm_sec, it);
		string strFileName(filename);
		string writefile = dir + basename + strFileName;
		cv::imwrite(writefile, image);
	}
	return;
}

void surfDraw(cv::Mat& img, vector<Ipoint>& ipts){
	for (int i = 0; i < ipts.size(); ++i){
		float s = 2.5f * ipts[i].s;
		float o = ipts[i].orientation;
		int lap = ipts[i].laplacian;
		int r1 = round(ipts[i].y);
		int c1 = round(ipts[i].x);
		int r2 = round(s * sin(o)) + r1;
		int c2 = round(s * cos(o)) + c1;
		
		if (o){
			line(img, cv::Point(c1, r1), cv::Point(c2, r2), cv::Scalar(0, 255, 0));
		} else{
			circle(img, cv::Point(c1, r1), 1, cv::Scalar(0, 255, 0), -1);
		}

		if (lap == 1){
			circle(img, cv::Point(c1, r1), round(s), cv::Scalar(255, 0, 0), 1);
		} else if (lap == 0){
			circle(img, cv::Point(c1, r1), round(s), cv::Scalar(0, 0, 255), 1);
		} else if (lap == 9){
			circle(img, cv::Point(c1, r1), round(s), cv::Scalar(0, 255, 0), 1);
		}
	}
}

cv::Mat drawMatches(cv::Mat& image1, cv::Mat& image2, vector<pair<Ipoint, Ipoint>>& matches){
	int width = image1.cols + image2.cols;
	int height = max(image1.rows, image2.rows);
	cv::Rect roi1(0, 0, image1.cols, image1.rows);
	cv::Rect roi2(image1.cols, 0, image2.cols, image2.rows);

	cv::Mat image(height, width, image1.type());
	image1.copyTo(image(roi1));
	image2.copyTo(image(roi2));

	for (int i = 0; i < matches.size(); ++i){
		cv::Point pt1(matches[i].first.x, matches[i].first.y);
		cv::Point pt2(matches[i].second.x + image1.cols, matches[i].second.y);
		cv::line(image, pt1, pt2, cv::Scalar(255, 128, 0));
	}
	return image;
}

void print3Dpoint(vector<cv::Point3d>& point){
	for (int i = 0; i < point.size(); ++i){
		cout << "  " << point[i].x << ", " << point[i].y << ", " << point[i].z << endl;
	}
	cout << endl;
	return;
}


}