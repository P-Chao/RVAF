/*
 * chnsPyramid.cpp
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#include <stdio.h>
#include <math.h>
#include <vector>

#include <opencv2\opencv.hpp>

#include "chnsPyramid.h"
#include "convTri.h"
#include "disp.h"

using namespace std;
using namespace cv;

namespace pc{


float avgLuminance = 0.2f;

inline double round(double r)
{
	return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

void GetScales(vector<float>& scales, vector<float>& scalesHt, vector<float>&scalesWd, 
	int32_t nPerOct, int32_t nOctUp, Size minDs, int32_t shrink, Size sz)
{
	int32_t nScales = floor(nPerOct * (nOctUp + log(min(
			(float)sz.width/(float)minDs.width, (float)sz.height/(float)minDs.height)))+1);
	int32_t d0, d1;
	float ss[101], em[101];

	for(int32_t i = 0; i < nScales; i++)
	{
		float s = pow(2, -(float)i / (float)nPerOct + nOctUp);
		scales.push_back(s);
	}

	if(sz.height < sz.width)
	{
		d0 = sz.height;
		d1 = sz.width;
	}
	else
	{
		d0 = sz.width;
		d1 = sz.height;
	}

	for(int32_t i = 0; i < nScales; i++)
	{
		float s = scales[i];
		float s0 = (round(d0*s/shrink)*shrink - 0.25f*shrink) / d0;
		float s1 = (round(d0*s/shrink)*shrink + 0.25f*shrink) / d0;
		float sd = s1 - s0;
		float es0, es1;
		for(int32_t j = 0; j < 101; j++)
		{
			ss[j] = j * 0.01f * sd + s0;
			es0 = d0 * ss[j] ;
			es0 = abs(es0 - round(es0 / shrink) * shrink);
			es1 = d1 * ss[j] ;
			es1 = abs(es1 - round(es1 / shrink) * shrink);
			em[j] = max(es0, es1);
		}

		float minEs = em[0];
		int32_t minI = 0;
		for(int32_t j = 1; j < 101; j++)
		{
			if(minEs > em[j])
			{
				minEs = em[j];
				minI = j;
			}
		}
		scales[i] = ss[minI];
	}

	vector<float>::iterator iter;
	iter = std::unique(scales.begin(), scales.end());
	scales.erase(iter, scales.end());

	scalesHt.clear();
	scalesWd.clear();
	for(uint32_t i = 0; i < scales.size(); i++)
	{
		float wd = round(sz.width  * scales[i] / shrink) * shrink / sz.width;
		float ht = round(sz.height * scales[i] / shrink) * shrink / sz.height;
		scalesHt.push_back(ht);
		scalesWd.push_back(wd);
	}
}

void ChnsPyramid(Mat& src, PyramidOpt& opt, Pyramid& pyramid)
{
	vector<int32_t> isR, isA, isN;
	vector<Mat> colorChannels;
	Mat I, lChn, uChn, vChn;
	Size sz;
	int32_t nScales;

	// Convert I to luv color space
	Bgr2Luv(src, I);

	split(I, colorChannels);
	lChn = colorChannels.at(0);
	uChn = colorChannels.at(1);
	vChn = colorChannels.at(2);

	//计算图像平均亮度
	avgLuminance = mean(lChn).val[0];

	sz = I.size();

	// Get scales at which to compute features and list of real/approx scales
	GetScales(pyramid.scales, pyramid.scalesHt, pyramid.scalesWd, opt.nPerOct, 
		opt.nOctUp, opt.minDs, opt.chnsOpt.shrink, sz);
	nScales = pyramid.scales.size();
	pyramid.nScales = nScales;
	for(int32_t i = 1; i <= nScales; i++)
	{
		if(i % (opt.nApprox+1) == 1)
		{
			isR.push_back(i);
		}
		else
		{
			isA.push_back(i);
		}
		isN.push_back(i);
	}

	vector<int32_t> j;
	j.push_back(0);
	for(uint32_t i = 0; i < isR.size()-1; i++)
	{
		int32_t tmp = (isR[i] + isR[i+1])/2;
		j.push_back(tmp);
	}
	j.push_back(nScales);

	for(uint32_t i = 0; i < isR.size(); i++)
	{
		for(int32_t k = j[i]; k < j[i+1]; k++)
		{
			isN[k] = isR[i];
		}
	}

	pyramid.nTypes = 0;

	// Compute image pyramid [real scales]
	for(int32_t i = 0; i < nScales; i++)
	{
		Channels chns;
		pyramid.data.push_back(chns);
	}

//	StartWatchTimer();
	for(uint32_t i = 0; i < isR.size(); i++)
	{
		Mat lChn1, uChn1, vChn1;
		Channels chns;
		float s = pyramid.scales[isR[i]-1];
		Size sz1;
		sz1.width = round(sz.width * s / opt.chnsOpt.shrink) * opt.chnsOpt.shrink;
		sz1.height = round(sz.height * s / opt.chnsOpt.shrink) * opt.chnsOpt.shrink;

		// Crop sz1 so divisible by shrink and get target dimensions
		Size cropSz = sz1;
		cropSz.width = (cropSz.width - cropSz.width % opt.chnsOpt.shrink);
		cropSz.height = (cropSz.height - cropSz.height % opt.chnsOpt.shrink);

		if(sz.width == cropSz.width && sz.height == cropSz.height)
		{
			lChn1 = lChn;
			uChn1 = uChn;
			vChn1 = vChn;
		}
		else
		{
			resize(lChn, lChn1, cropSz, 0, 0);
			resize(uChn, uChn1, cropSz, 0, 0);
			resize(vChn, vChn1, cropSz, 0, 0);
		}
		if(s == 0.5f && (opt.nApprox > 0 || opt.nPerOct == 1))
		{
			lChn = lChn1;
			uChn = uChn1;
			vChn = vChn1;
		}
		ChnsCompute(lChn1, uChn1, vChn1, pyramid.data[isR[i]-1], opt.chnsOpt);
	}

	// Compute image pyramid [approximated scales]
	for(uint32_t i = 0; i < isA.size(); i++)
	{
		int32_t iA = isA[i]-1;
		int32_t iR = isN[iA]-1;
		Size sz1;
		sz1.width = round(sz.width * pyramid.scales[iA] / opt.chnsOpt.shrink);
		sz1.height = round(sz.height * pyramid.scales[iA] / opt.chnsOpt.shrink);

		float ratio;

		resize(pyramid.data[iR].lChn, pyramid.data[iA].lChn, sz1, INTER_CUBIC);
		resize(pyramid.data[iR].uChn, pyramid.data[iA].uChn, sz1, INTER_CUBIC);
		resize(pyramid.data[iR].vChn, pyramid.data[iA].vChn, sz1, INTER_CUBIC);

		ratio = pow((pyramid.scales[iA] / pyramid.scales[iR]), -opt.lambdas[1]);
		resize(pyramid.data[iR].gradMag, pyramid.data[iA].gradMag, sz1, INTER_CUBIC);
		pyramid.data[iA].gradMag *= ratio;
		pyramid.data[iA].gradHist.clear();
		for(uint32_t k = 0; k < pyramid.data[iR].gradHist.size(); k++)
		{
			Mat tmp;
			pyramid.data[iA].gradHist.push_back(tmp);
			resize(pyramid.data[iR].gradHist[k], pyramid.data[iA].gradHist[k], sz1, INTER_CUBIC);
			pyramid.data[iA].gradHist[k] *= ratio;
		}
	}

	// Smooth channels, optionally pad and concatenate channels
	for(int32_t i = 0; i < nScales; i++)
	{
		ConvConst(pyramid.data[i].lChn       , pyramid.data[i].lChn       , opt.smooth);
		ConvConst(pyramid.data[i].uChn       , pyramid.data[i].uChn       , opt.smooth);
		ConvConst(pyramid.data[i].vChn       , pyramid.data[i].vChn       , opt.smooth);
		ConvConst(pyramid.data[i].gradMag    , pyramid.data[i].gradMag    , opt.smooth);
		ConvConst(pyramid.data[i].gradHist[0], pyramid.data[i].gradHist[0], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[1], pyramid.data[i].gradHist[1], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[2], pyramid.data[i].gradHist[2], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[3], pyramid.data[i].gradHist[3], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[4], pyramid.data[i].gradHist[4], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[5], pyramid.data[i].gradHist[5], opt.smooth);
	}

	if(opt.pad.width > 0 || opt.pad.height > 0)
	{
		for(int32_t i = 0; i < nScales; i++)
		{
			int32_t padHeight = opt.pad.height / opt.chnsOpt.shrink;
			int32_t padWidth  = opt.pad.width  / opt.chnsOpt.shrink;
			copyMakeBorder(pyramid.data[i].lChn,        pyramid.data[i].lChn,       padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].uChn,        pyramid.data[i].uChn,       padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].vChn,        pyramid.data[i].vChn,       padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradMag,     pyramid.data[i].gradMag,     padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[0], pyramid.data[i].gradHist[0], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[1], pyramid.data[i].gradHist[1], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[2], pyramid.data[i].gradHist[2], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[3], pyramid.data[i].gradHist[3], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[4], pyramid.data[i].gradHist[4], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[5], pyramid.data[i].gradHist[5], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);

			/*imshow("tmp/l.bmp", pyramid.data[i].lChn);
			imshow("tmp/u.bmp", pyramid.data[i].uChn);
			imshow("tmp/v.bmp", pyramid.data[i].vChn);
			imshow("tmp/m.bmp", pyramid.data[i].gradMag);
			imshow("tmp/h0.bmp", pyramid.data[i].gradHist[0]);
			imshow("tmp/h1.bmp", pyramid.data[i].gradHist[1]);
			imshow("tmp/h2.bmp", pyramid.data[i].gradHist[2]);
			imshow("tmp/h3.bmp", pyramid.data[i].gradHist[3]);
			imshow("tmp/h4.bmp", pyramid.data[i].gradHist[4]);
			imshow("tmp/h5.bmp", pyramid.data[i].gradHist[5]);
			waitKey();*/
		}
	}

	for(int32_t i = 0; i < nScales; i++)
	{
		ChnsDataGather(pyramid.data[i]);
	}

	
	

}

void ChnsPyramidScale(Mat& src, PyramidOpt& opt, Pyramid& pyramid, int scaleindex)
{
	vector<int32_t> isR, isA, isN;
	vector<Mat> colorChannels;
	Mat I, lChn, uChn, vChn;
	Size sz;
	int32_t nScales;

	// Convert I to luv color space
	Bgr2Luv(src, I);

	split(I, colorChannels);
	lChn = colorChannels.at(0);
	uChn = colorChannels.at(1);
	vChn = colorChannels.at(2);

	//计算图像平均亮度
	avgLuminance = mean(lChn).val[0];

	sz = I.size();

	// Get scales at which to compute features and list of real/approx scales
	GetScales(pyramid.scales, pyramid.scalesHt, pyramid.scalesWd, opt.nPerOct,
		opt.nOctUp, opt.minDs, opt.chnsOpt.shrink, sz);
	
	nScales = 1;
	pyramid.nScales = nScales;
	/*nScales = pyramid.scales.size();
	pyramid.nScales = nScales;*/
	//isR.push_back(scaleindex);
	/*for (int32_t i = 1; i <= nScales; i++)
	{
		if (i % (opt.nApprox + 1) == 1)
		{
			isR.push_back(i);
		}
		else
		{
			isA.push_back(i);
		}
		isN.push_back(i);
	}*/

	/*vector<int32_t> j;
	j.push_back(0);
	for (uint32_t i = 0; i < isR.size() - 1; i++)
	{
		int32_t tmp = (isR[i] + isR[i + 1]) / 2;
		j.push_back(tmp);
	}
	j.push_back(nScales);

	for (uint32_t i = 0; i < isR.size(); i++)
	{
		for (int32_t k = j[i]; k < j[i + 1]; k++)
		{
			isN[k] = isR[i];
		}
	}*/

	pyramid.nTypes = 0;

	// Compute image pyramid [real scales]
	/*for (int32_t i = 0; i < nScales; i++)
	{*/
		Channels chns;
		pyramid.data.push_back(chns);
	/*}*/

	//	StartWatchTimer();
	/*for (uint32_t i = 0; i < isR.size(); i++)
	{*/
		Mat lChn1, uChn1, vChn1;
		//Channels chns;
		float s = pyramid.scales[scaleindex];
		Size sz1;
		sz1.width = round(sz.width * s / opt.chnsOpt.shrink) * opt.chnsOpt.shrink;
		sz1.height = round(sz.height * s / opt.chnsOpt.shrink) * opt.chnsOpt.shrink;

		// Crop sz1 so divisible by shrink and get target dimensions
		Size cropSz = sz1;
		cropSz.width = (cropSz.width - cropSz.width % opt.chnsOpt.shrink);
		cropSz.height = (cropSz.height - cropSz.height % opt.chnsOpt.shrink);



		if (sz.width == cropSz.width && sz.height == cropSz.height)
		{
			lChn1 = lChn;
			uChn1 = uChn;
			vChn1 = vChn;
		}
		else
		{
			resize(lChn, lChn1, cropSz, 0, 0);
			resize(uChn, uChn1, cropSz, 0, 0);
			resize(vChn, vChn1, cropSz, 0, 0);
		}
		if (s == 0.5f && (opt.nApprox > 0 || opt.nPerOct == 1))
		{
			lChn = lChn1;
			uChn = uChn1;
			vChn = vChn1;
		}
		ChnsCompute(lChn1, uChn1, vChn1, pyramid.data[0], opt.chnsOpt);
	/*}*/

	// Compute image pyramid [approximated scales]
	/*for (uint32_t i = 0; i < isA.size(); i++)
	{
		int32_t iA = isA[i] - 1;
		int32_t iR = isN[iA] - 1;

		Size sz1;
		sz1.width = round(sz.width * pyramid.scales[iA] / opt.chnsOpt.shrink);
		sz1.height = round(sz.height * pyramid.scales[iA] / opt.chnsOpt.shrink);

		float ratio;

		resize(pyramid.data[iR].lChn, pyramid.data[iA].lChn, sz1, INTER_CUBIC);
		resize(pyramid.data[iR].uChn, pyramid.data[iA].uChn, sz1, INTER_CUBIC);
		resize(pyramid.data[iR].vChn, pyramid.data[iA].vChn, sz1, INTER_CUBIC);

		ratio = pow((pyramid.scales[iA] / pyramid.scales[iR]), -opt.lambdas[1]);
		resize(pyramid.data[iR].gradMag, pyramid.data[iA].gradMag, sz1, INTER_CUBIC);
		pyramid.data[iA].gradMag *= ratio;
		pyramid.data[iA].gradHist.clear();
		for (uint32_t k = 0; k < pyramid.data[iR].gradHist.size(); k++)
		{
			Mat tmp;
			pyramid.data[iA].gradHist.push_back(tmp);
			resize(pyramid.data[iR].gradHist[k], pyramid.data[iA].gradHist[k], sz1, INTER_CUBIC);
			pyramid.data[iA].gradHist[k] *= ratio;
		}
	}*/

	// Smooth channels, optionally pad and concatenate channels
	for (int32_t i = 0; i < nScales; i++)
	{
		ConvConst(pyramid.data[i].lChn, pyramid.data[i].lChn, opt.smooth);
		ConvConst(pyramid.data[i].uChn, pyramid.data[i].uChn, opt.smooth);
		ConvConst(pyramid.data[i].vChn, pyramid.data[i].vChn, opt.smooth);
		ConvConst(pyramid.data[i].gradMag, pyramid.data[i].gradMag, opt.smooth);
		ConvConst(pyramid.data[i].gradHist[0], pyramid.data[i].gradHist[0], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[1], pyramid.data[i].gradHist[1], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[2], pyramid.data[i].gradHist[2], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[3], pyramid.data[i].gradHist[3], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[4], pyramid.data[i].gradHist[4], opt.smooth);
		ConvConst(pyramid.data[i].gradHist[5], pyramid.data[i].gradHist[5], opt.smooth);
	}

	if (opt.pad.width > 0 || opt.pad.height > 0)
	{
		for (int32_t i = 0; i < nScales; i++)
		{
			int32_t padHeight = opt.pad.height / opt.chnsOpt.shrink;
			int32_t padWidth = opt.pad.width / opt.chnsOpt.shrink;
			copyMakeBorder(pyramid.data[i].lChn, pyramid.data[i].lChn, padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].uChn, pyramid.data[i].uChn, padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].vChn, pyramid.data[i].vChn, padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradMag, pyramid.data[i].gradMag, padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[0], pyramid.data[i].gradHist[0], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[1], pyramid.data[i].gradHist[1], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[2], pyramid.data[i].gradHist[2], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[3], pyramid.data[i].gradHist[3], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[4], pyramid.data[i].gradHist[4], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
			copyMakeBorder(pyramid.data[i].gradHist[5], pyramid.data[i].gradHist[5], padHeight, padHeight, padWidth, padWidth, BORDER_REPLICATE);
		}
	}

	for (int32_t i = 0; i < nScales; i++)
	{
		ChnsDataGather(pyramid.data[i]);
	}

}

}
