/*
 *	chnsCompute.cpp
 *		for VisualStudio2012 & opencv3.0.0
 *
 *	Created on:	Aug 2,	2015
 *		Author:	Peng Chao
 *
 */

#include <vector>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "chnsCompute.h"
#include "convTri.h"
#include "gradient.h"
#include "disp.h"
#include <glog\logging.h>

using namespace std;
using namespace cv;

namespace pc{


float bgr2luvData [256*256*256*3];

void Bgr2Luv(Mat& src, Mat& dst)
{
	int32_t pixelCnt_3 = src.size().width * src.size().height * 3;
	int32_t index;
	float* dstData;
	dst.create(src.size(), CV_32FC3);
	dstData = (float*)dst.data;

	for(int32_t i = 0; i < pixelCnt_3; i += 3 * 4)
	{
		index = (((int32_t)src.data[i] << 16 | (int32_t)src.data[i+1] << 8 | (int32_t)src.data[i+2]));
		index = (index << 2) - index;
		memcpy(dstData, &bgr2luvData[index], 12);
		dstData += 3;

		index = (((int32_t)src.data[i+3] << 16 | (int32_t)src.data[i+4] << 8 | (int32_t)src.data[i+5]));
		index = (index << 2) - index;
		memcpy(dstData, &bgr2luvData[index], 12);
		dstData += 3;

		index = (((int32_t)src.data[i+6] << 16 | (int32_t)src.data[i+7] << 8 | (int32_t)src.data[i+8]));
		index = (index << 2) - index;
		memcpy(dstData, &bgr2luvData[index], 12);
		dstData += 3;

		index = (((int32_t)src.data[i+9] << 16 | (int32_t)src.data[i+10] << 8 | (int32_t)src.data[i+11]));
		index = (index << 2) - index;
		memcpy(dstData, &bgr2luvData[index], 12);
		dstData += 3;
	}
}

void ChnsCompute(Mat& lChn, Mat& uChn, Mat& vChn, Channels& chns, ChannelsOpt& opt)
{
	Channels tmpChns;
	Mat ori;

	ConvConst(lChn, lChn, opt.colorSmooth);
	ConvConst(uChn, uChn, opt.colorSmooth);
	ConvConst(vChn, vChn, opt.colorSmooth);

	GradientMag(lChn, tmpChns.gradMag, ori, 1, opt.gradMagNormRad, opt.gradMagNormConst, opt.gradMagFull);
	GradientHist(tmpChns.gradMag, ori, tmpChns.gradHist, tmpChns.gradHistChns, opt.gradHistBinSize, opt.gradHistOrients,
				 opt.gradHistSoftBin, opt.gradHistUseHog, opt.gradHistClipHog, opt.gradMagFull);

	//Down sampling the channels
	Size sampleSize;
	sampleSize.width  = lChn.size().width / opt.shrink;
	sampleSize.height = lChn.size().height / opt.shrink;

	resize(lChn           , chns.lChn   , sampleSize, 0, 0, INTER_NEAREST);
	resize(uChn           , chns.uChn   , sampleSize, 0, 0, INTER_NEAREST);
	resize(vChn           , chns.vChn   , sampleSize, 0, 0, INTER_NEAREST);
	resize(tmpChns.gradMag, chns.gradMag, sampleSize, 0, 0, INTER_CUBIC);

	chns.gradHist = tmpChns.gradHist;
	chns.gradHistChns = tmpChns.gradHistChns;
}

void ChnsDataGather(Channels& chns)
{
	chns.width = chns.lChn.size().width;
	chns.height = chns.lChn.size().height;
	chns.size = chns.width * chns.height;
	chns.data = new float[chns.size * 10];
	memcpy(chns.data                , chns.lChn.data       , chns.size * 4);
	memcpy(chns.data + chns.size    , chns.uChn.data       , chns.size * 4);
	memcpy(chns.data + chns.size * 2, chns.vChn.data       , chns.size * 4);
	memcpy(chns.data + chns.size * 3, chns.gradMag.data    , chns.size * 4);
	memcpy(chns.data + chns.size * 4, chns.gradHist[0].data, chns.size * 4);
	memcpy(chns.data + chns.size * 5, chns.gradHist[1].data, chns.size * 4);
	memcpy(chns.data + chns.size * 6, chns.gradHist[2].data, chns.size * 4);
	memcpy(chns.data + chns.size * 7, chns.gradHist[3].data, chns.size * 4);
	memcpy(chns.data + chns.size * 8, chns.gradHist[4].data, chns.size * 4);
	memcpy(chns.data + chns.size * 9, chns.gradHist[5].data, chns.size * 4);
}

void ChnsDataRelease(Channels& chns)
{
	delete [] chns.data;
}

void ChnsComputeInit()
{
	FILE *fp = fopen("./bgr2luv.dat","r");
	if(fp == NULL)
	{
		LOG(FATAL) << "Open bgr2luv.dat error!";
		fprintf(stderr, "Open bgr2luv.dat error");
		exit(1);
	}
	fread(bgr2luvData, 256*256*256*3*4, 1, fp);
	fclose(fp);
}

}

