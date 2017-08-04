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

#define STRIDE		2
#define OCTAVES		5
#define INTERVALS	4
#define THRES 0.0004f

#ifndef PI
#define PI	3.1415926535897932384626433
#endif
#define imcElem(__img, __row, __col, __chns)										\
		(__img).data[(__img.chns) * ((__img).cols * (__row)) + (__col)]
#define imgElem(__img, __row, __col)												\
		(__img).data[(__img).cols * (__row) + (__col)]
#define layerResponse(__layer, __row, __col)										\
		(__layer).responses[(__row)*((__layer).width) + (__col)]
#define layerLaplacian(__layer, __row, __col)										\
		(__layer).laplacian[(__row)*((__layer).width) + (__col)]
#define dimshow(__name, __mat) if(algparam.showprocess) cv::imshow((__name), (__mat))

#include <opencv2\opencv.hpp>

using namespace std;

namespace pc{

typedef unsigned int uint;

typedef struct _Image_uchar{
	uchar*	data;
	int		rows;
	int		cols;
	int		chns;
} uimg;

typedef struct _Image_float{
	float*	data;
	int		rows;
	int		cols;
	int		chns;
} fimg;

typedef struct _Ipoint{
	float	x, y, s;
	float	dx, dy;
	float	orientation;
	float	descriptor[64];

	uchar	laplacian;
	int		clusterIndex;
} Ipoint;

typedef struct _ResponseLayer{
	int		width;
	int		height;
	int		step;
	int		filter;

	float*	responses;
	uchar*	laplacian;
} ResponseLayer;

struct SurfParam{
	bool	upright = false;
	int		stride = 2;//2
	int		octaves = 5;//5
	int		intervals = 4;//4
	int		homothresh = 5;//5
	float	thresh = 0.0004f;//0.0004f
};

struct AlgParam{
	char	runtype = 'A';
	int		camnum = 0;
	bool	savelog = false;
	bool	saveimage = false;
	bool	showprocess = false;
	bool	usedetector = false;
	std::string	videoname = "res/1.mp4";
	std::string	leftimage = "res/left.jpg";
	std::string	rightimage = "res/right.jpg";
};

void SurfPoint(cv::Mat& image, vector<cv::Point2f>& points, vector<float>& scales, vector<int>& label);
void SurfDescriptor(cv::Mat& image, vector<cv::Point2f>& points, vector<float>& scales,
	vector<vector<float>>& descriptors);

}
