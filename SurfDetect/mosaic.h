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


#pragma once
#include "common.h"

using namespace std;

namespace pc{

void iptMatch(const vector<Ipoint>& ipts1, const vector<Ipoint>& ipts2, 
	vector<pair<Ipoint, Ipoint>>& matches);
cv::Mat computeHomography(vector<pair<Ipoint, Ipoint>>& matches);
cv::Mat imageMosaic(const cv::Mat& image1, const cv::Mat& image2, 
	const cv::Mat& homography);

}
