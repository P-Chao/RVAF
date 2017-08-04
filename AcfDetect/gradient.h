/*
 * gradient.h
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#ifndef GRADIENT_H_
#define GRADIENT_H_

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

namespace pc{

void GradientMag(Mat& src, Mat& mag, Mat& ori, int32_t channel, int32_t normRad, float normConst, int32_t full);
void GradientHist(Mat& mag, Mat& ori, vector<Mat>& hist, int32_t& nChns, int32_t binSize, int32_t nOrients, int32_t softBin, int32_t useHog, float clipHog, int32_t full);

}
#endif /* GRADIENT_H_ */
