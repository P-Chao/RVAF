/*
 *	chnsCompute.h
 *		for VisualStudio2012 & opencv3.0.0
 *
 *	Created on:	Aug 2,	2015
 *		Author:	Peng Chao
 *
 */

#ifndef CHNSCOMPUTE_H_
#define CHNSCOMPUTE_H_

#include <opencv2/opencv.hpp>

#include <stdint.h>
#include <vector>

#include "gradient.h"
#include "convTri.h"

using namespace std;
using namespace cv;

namespace pc{

typedef struct
{
	Mat		lChn;
	Mat		uChn;
	Mat		vChn;
	Mat		gradMag;
	vector<Mat> gradHist;
	float*	data;
	int32_t width;
	int32_t height;
	int32_t size;
	int32_t gradHistChns;
} Channels;

typedef struct
{
	int32_t shrink;
	int32_t colorSmooth;
	int32_t gradMagNormRad;
	float   gradMagNormConst;
	int32_t gradMagFull;
	int32_t gradHistBinSize;
	int32_t gradHistOrients;
	int32_t gradHistSoftBin;
	int32_t gradHistUseHog;
	float gradHistClipHog;
} ChannelsOpt;

void Bgr2Luv(Mat& src, Mat& dst);
void ChnsCompute(Mat& lChn, Mat& uChn, Mat& vChn, Channels& chns, ChannelsOpt& opt);
void ChnsDataGather(Channels& chns);
void ChnsDataRelease(Channels& chns);
void ChnsComputeInit();

}


#endif /* CHNSCOPUTE_H_ */
