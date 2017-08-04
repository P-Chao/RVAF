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

using namespace std;
using namespace cv;

namespace pc{


extern AlgParam algparam;
extern SurfParam suparam;

void iptMatch(const vector<Ipoint>& ipts1, const vector<Ipoint>& ipts2, 
	vector<pair<Ipoint, Ipoint>>& matches){
	float d1, d2, dist;
	const Ipoint *match;

	for (int i = 0; i < ipts1.size(); ++i){
		d1 = d2 = FLT_MAX;
		for (int j = 0; j < ipts2.size(); ++j){
			dist = 0.0f;
			for (int k = 0; k < 64; ++k){
				float diff = (ipts1[i].descriptor[k] - ipts2[j].descriptor[k]);
				dist += diff * diff;
			}
			dist = sqrt(dist);

			if (dist < d1){
				d2 = d1;
				d1 = dist;
				match = &ipts2[j];
			} else if (dist < d2){
				d2 = dist;
			}
		}

		if (d1 / d2 < 0.65){
			//ipts1[i].dx = match->x - ipts1[i].x;
			//ipts1[i].dy = match->y - ipts1[i].y;
			matches.push_back(make_pair(ipts1[i], *match));
		}
	}
}

Mat computeHomography(vector<pair<Ipoint, Ipoint>>& matches){
	vector<cv::Point2f> pt1, pt2;
	vector<uchar> mask(matches.size());
	for (int i = 0; i < matches.size(); ++i){
		pt1.push_back(Point(matches[i].first.x, matches[i].first.y));
		pt2.push_back(Point(matches[i].second.x, matches[i].second.y));
	}
	Mat homography = findHomography(pt2, pt1, mask, CV_RANSAC, suparam.homothresh);

	vector<pair<Ipoint, Ipoint>> inliers;
	for (int i = 0; i < matches.size(); ++i){
		if (mask[i]){
			inliers.push_back(matches[i]);
		}
	}
	matches.swap(inliers);
	return homography;
}

inline static void computeCorner(const Mat& homography, const Point& src, Point& dst){
	const double a = homography.at<double>(0, 0) * src.x + 
		homography.at<double>(0, 1) * src.y + homography.at<double>(0, 2);
	const double b = homography.at<double>(1, 0) * src.x + 
		homography.at<double>(1, 1) * src.y + homography.at<double>(1, 2);
	const double c = homography.at<double>(2, 0) * src.x + 
		homography.at<double>(2, 1) * src.y + homography.at<double>(2, 2);
	dst.x = round(a / c);
	dst.y = round(b / c);
}

static void imageFusion(const Mat& image1, Mat& xformed, const int start, const int end){
	if (start > xformed.rows || start > end)
		return;
	addWeighted(image1(Rect(0, 0, start, xformed.rows)), 1, xformed(Rect(0, 0, start, xformed.rows)), 
		0, 0, xformed(Rect(0, 0, start, xformed.rows)));
	dimshow("Before Fusion", xformed);
	
	double alpha = 1;
	const int processCols = min(xformed.cols, image1.cols);
	const double processWidth = end - start;
	for (int i = 0; i < xformed.rows; ++i){
		for (int j = start; j < end; ++j){
			const Vec3b bgr = xformed.at<Vec3b>(i, j);
			if (10 >(int)bgr[0] + bgr[1] + bgr[2]){
				alpha = 1;
			} else{
				alpha = min((processWidth - (j - start)) / processWidth + 0.02, (double)1.0f);
			}
			xformed.at<Vec3b>(i, j)[0] = alpha * image1.at<Vec3b>(i, j)[0] 
				+ (1 - alpha) * xformed.at<Vec3b>(i, j)[0];
			xformed.at<Vec3b>(i, j)[1] = alpha * image1.at<Vec3b>(i, j)[1] 
				+ (1 - alpha) * xformed.at<Vec3b>(i, j)[1];
			xformed.at<Vec3b>(i, j)[2] = alpha * image1.at<Vec3b>(i, j)[2] 
				+ (1 - alpha) * xformed.at<Vec3b>(i, j)[2];
		}
	}

}

Mat imageMosaic(const Mat& image1, const Mat& image2, const Mat& homography){
	Point srcCorner[4], dstCorner[4];
	srcCorner[0].x = 0;
	srcCorner[0].y = 0;
	srcCorner[1].x = 0;
	srcCorner[1].y = image2.rows;
	srcCorner[2].x = image2.cols;
	srcCorner[2].y = 0;
	srcCorner[3].x = image2.cols;
	srcCorner[3].y = image2.rows;
	for (int i = 0; i < 4; ++i){
		computeCorner(homography, srcCorner[i], dstCorner[i]);
	}

	Mat xformed(Size(min(dstCorner[2].x, dstCorner[3].x), min(image1.rows, image2.rows)), image2.type());
	warpPerspective(image2, xformed, homography, xformed.size(), CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);
	imageFusion(image1, xformed, max(0, min(dstCorner[0].x, dstCorner[1].x)), min(image1.cols, xformed.cols));
	return xformed;
}

}