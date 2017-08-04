/*
 *	chnsPyramid.h
 *		for VisualStudio2012 & opencv3.0.0
 *
 *	Created on:	Aug 2,	2015
 *		Author:	Peng Chao
 *
 */

#ifndef CHNSPYRAMID_H_
#define CHNSPYRAMID_H_

#include <stdint.h>
#include <vector>

#include <opencv2\opencv.hpp>

#include "chnsCompute.h"

using namespace std;
using namespace cv;

namespace pc{


typedef struct
{
	ChannelsOpt	chnsOpt;
	int32_t		nPerOct;
	int32_t		nOctUp;
	int32_t		nApprox;
	vector<float> lambdas;
	Size		pad;
	Size		minDs;
	int32_t		smooth;
} PyramidOpt;

typedef struct
{
	int32_t		nTypes;
	int32_t		nScales;
	vector<Channels> data;
	vector<float> lambdas;
	vector<float> scales;
	vector<float> scalesHt;
	vector<float> scalesWd;
} Pyramid;

void ChnsPyramid(Mat& src, PyramidOpt& opt, Pyramid& pyramid);
void ChnsPyramidScale(Mat& src, PyramidOpt& opt, Pyramid& pyramid, int scaleindex);
}


#endif /* CHNSPYRAMID_H_ */