/*
 *	disp.h
 *		for VisualStudio2012 & opencv3.0.0
 *
 *	Created on:	Aug 2,	2015
 *		Author:	Peng Chao
 *
 */

#ifndef DISP_H_
#define DISP_H_

#include <opencv2\opencv.hpp>

#include "acfDetect.h"
#include "chnsPyramid.h"

using namespace cv;

namespace pc{

void __StartWatchTimer();
void __ReadWatchTimer();
void DispResult(Mat& dst, vector<DetectResult>& result);

}

#endif /* DISP_H_ */
