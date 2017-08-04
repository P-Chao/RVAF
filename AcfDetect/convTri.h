/*
 * convTri.h
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#ifndef CONVTRI_H_
#define CONVTRI_H_

#include <stdint.h>

#include <opencv2\opencv.hpp>

using namespace cv;

namespace pc{

void ConvTri(float* src, float* dst, int32_t h, int32_t w, int32_t d, int32_t r, int32_t s);
void ConvTrix(Mat& src, Mat& dst, int32_t r);
void ConvConst(Mat& src, Mat& dst, int32_t r, int32_t s = 1);

}
#endif /* CONVTRI_H_ */
