/*
 * acfDetect.cpp
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#include <stdio.h>
#include <vector>

#include <opencv2\opencv.hpp>

#include "acfDetect.h"
#include "chnsPyramid.h"
#include "disp.h"
#include <glog\logging.h>

using namespace std;

/// One of the @link comparison_functors comparison functors@endlink.
template<typename _Tp>
struct greater : public binary_function<_Tp, _Tp, bool>{
	bool operator()(const _Tp& __x, const _Tp& __y) const{ 
		return __x > __y; }
};

namespace pc{


static void SuppressResult(vector<DetectResult>& ri, vector<DetectResult>& ro, float nms = 0.65f);

AcfDetector::AcfDetector(char* path)
{
	uint32_t size;
	FILE *fp = fopen(path, "rb+");
	
	if(fp == NULL )
	{
		LOG(FATAL) << "Open detector error!";
		fprintf(stderr, "Open detector error\n");
		exit(1);
	}

	fread(&this->nTreeNodes, 4, 1, fp);
	fread(&this->nTrees, 4, 1, fp);
	size = this->nTreeNodes * this->nTrees;

	this->fids    = new uint32_t[size];
	this->thrs    = new float[size];
	this->child   = new uint32_t[size];
	this->hs      = new float[size];
	this->weights = new float[size];
	this->depth   = new uint32_t[size];

	fread(this->fids, size * 4, 1, fp);
	fread(this->thrs, size * 4, 1, fp);
	fread(this->child, size * 4, 1, fp);
	fread(this->hs, size * 4, 1, fp);
	fread(this->weights, size * 4, 1, fp);
	fread(this->depth, size * 4, 1, fp);
	fread(&this->treeDepth, 4, 1, fp);
	fread(&this->stride, 4, 1, fp);
	fread(&this->cascThr, 4, 1, fp);
	fread(&this->modelHt, 4, 1, fp);
	fread(&this->modelWd, 4, 1, fp);
	fread(&this->modelPadHt, 4, 1, fp);
	fread(&this->modelPadWd, 4, 1, fp);
	fclose(fp);
}

void AcfDetector::Open(char* path)
{
	uint32_t size;
	FILE *fp = fopen(path, "rb+");

	if (fp == NULL)
	{
		LOG(FATAL) << "Open detector error!";
		fprintf(stderr, "Open detector error\n");
		exit(1);
	}

	fread(&this->nTreeNodes, 4, 1, fp);
	fread(&this->nTrees, 4, 1, fp);
	size = this->nTreeNodes * this->nTrees;

	this->fids = new uint32_t[size];
	this->thrs = new float[size];
	this->child = new uint32_t[size];
	this->hs = new float[size];
	this->weights = new float[size];
	this->depth = new uint32_t[size];

	fread(this->fids, size * 4, 1, fp);
	fread(this->thrs, size * 4, 1, fp);
	fread(this->child, size * 4, 1, fp);
	fread(this->hs, size * 4, 1, fp);
	fread(this->weights, size * 4, 1, fp);
	fread(this->depth, size * 4, 1, fp);
	fread(&this->treeDepth, 4, 1, fp);
	fread(&this->stride, 4, 1, fp);
	fread(&this->cascThr, 4, 1, fp);
	fread(&this->modelHt, 4, 1, fp);
	fread(&this->modelWd, 4, 1, fp);
	fread(&this->modelPadHt, 4, 1, fp);
	fread(&this->modelPadWd, 4, 1, fp);
	fclose(fp);
}

AcfDetector::~AcfDetector()
{
	delete [] this->fids;
	delete [] this->thrs;
	delete [] this->child;
	delete [] this->hs;
	delete [] this->weights;
	delete [] this->depth;
}

inline void GetChild(float* chns1, uint32_t* cids, uint32_t* fids,
					 float* thrs, uint32_t offset, uint32_t& k0, uint32_t& k)
{
	float ftr = chns1[cids[fids[k]]];
	k = (ftr < thrs[k]) ? 1 : 2;
	k0 = k += k0 * 2;
	k += offset;
}

void AcfDetector::Detect(Channels& chns, int32_t shrink, vector<DetectResult>& result, float epipolarLine)
{
	// Get dimensions and constants
	const int32_t height = chns.gradMag.size().height;
	const int32_t width  = chns.gradMag.size().width;
	const int32_t nChns  = 10;
	const int32_t height1 = (int32_t)ceil(float(height * shrink - modelPadHt + 1) / stride);
	const int32_t width1  = (int32_t)ceil(float(width  * shrink - modelPadWd + 1) / stride);

	// Construct cids array
	int32_t nFtrs = modelPadHt / shrink * modelPadWd / shrink * nChns;
	uint32_t *cids = new uint32_t[nFtrs];
	int32_t m = 0;

	for(int32_t z = 0; z < nChns; z++)
	{
		for(int32_t c = 0; c < modelPadWd / shrink; c++)
		{
			for(int32_t r = 0; r < modelPadHt / shrink; r++)
			{
				cids[m++] = z * width * height + r * width + c;
			}
		}
	}

	int SearchLine = -1;
	if (epipolarLine > 0){
		SearchLine = (epipolarLine + 0.5) / stride;
		if (SearchLine < 0 && SearchLine <= height1){
			SearchLine = -1;
		}
	}
	
	
	

	// Apply classifier to each patch
#if(USE_OPENMP == -1)
	int32_t c;
	#pragma omp parallel for private(c) num_threads(3)
	for(c = 0; c < width1; c++)
#else
	for(int32_t c = 0; c < width1; c++)
#endif
	{
		for(int32_t r = 0; r < height1; r++)
		{
			if (SearchLine > 0){
				if (r < SearchLine){
					continue;
				}
				if (r > SearchLine){
					break;
				}
			}

			float h = 0, *chns1 = chns.data + (r * stride / shrink) * width + (c * stride / shrink);
			if(treeDepth == 1)
			{
				// specialized case for treeDepth==1
				for(uint32_t t = 0; t < nTrees; t++)
				{
					uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
					GetChild(chns1, cids, fids, thrs, offset, k0, k);
					h += hs[k];
					if(h <= cascThr)
					{
						break;
					}
				}
			}
			else if(treeDepth == 2)
			{	// specialized case for treeDepth==2
				for(uint32_t t = 0; t < nTrees; t++)
				{
					uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
					GetChild(chns1, cids, fids, thrs, offset, k0, k);
					GetChild(chns1, cids, fids, thrs, offset, k0, k);
					h += hs[k];
					if(h <= cascThr)
					{
						break;
					}
				}
			}
			else if(treeDepth > 2)
			{
				// specialized case for treeDepth>2
				for(uint32_t t = 0; t < nTrees; t++)
				{
					uint32_t offset = t * nTreeNodes, k = offset, k0 = 0;
					for(uint32_t i = 0; i < treeDepth; i++)
					{
						GetChild(chns1, cids, fids, thrs, offset, k0, k);
					}
					h += hs[k];
					if(h <= cascThr)
					{
						break;
					}
				}
			}
			else
			{
				// general case (variable tree depth)
				for(uint32_t t = 0; t < nTrees; t++)
				{
					uint32_t offset = t * nTreeNodes, k = offset, k0 = k;
					while(child[k])
					{
						float ftr = chns1[cids[fids[k]]];
						k = (ftr < thrs[k]) ? 1 : 0;
						k0 = k = child[k0] - k + offset;
					}
					h += hs[k];
					if(h <= cascThr)
					{
						break;
					}
				}
			}
			if(h > cascThr)
			{
				DetectResult res;
				res.cs = c * stride;
				res.modelWd = modelWd;
				res.modelHt = modelHt;
				res.rs = r * stride;
				res.hs = h;
#if(USE_OPENMP == -1)
				#pragma omp critical (result)
				{
					result.push_back(res);
				}
#else
				result.push_back(res);
#endif
			}
		}
	}

	delete [] cids;
}

void AcfDetectImg(Mat& img, PyramidOpt& opt, AcfDetector& detector, vector<DetectResult>& result, float nms)
{
	Pyramid pyramid;
	vector<DetectResult> res;
	int32_t shiftX, shiftY;

	ChnsPyramid(img, opt, pyramid);

	result.clear();

#if(USE_OPENMP == 1)
	int32_t i;
	#pragma omp parallel for private(i) num_threads(3)
	for(i = 0; i < pyramid.nScales; i++)
#else
	for(int32_t i = 0; i < pyramid.nScales; i++)
#endif
	{
		vector<DetectResult> r0;
		detector.Detect(pyramid.data[i], opt.chnsOpt.shrink, r0);
		shiftX = (detector.modelPadWd - detector.modelWd) / 2 - opt.pad.width;
		shiftY = (detector.modelPadHt - detector.modelHt) / 2 - opt.pad.height;
		for(uint32_t j = 0; j < r0.size(); j++)
		{
			DetectResult r = r0[j];
			r.cs = (r.cs + shiftX) / pyramid.scalesWd[i];
			r.rs = (r.rs + shiftY) / pyramid.scalesHt[i];
			r.modelWd = detector.modelWd / pyramid.scales[i];
			r.modelHt = detector.modelHt / pyramid.scales[i];
			r.scaleindex = i;
#if(USE_OPENMP == 1)
			#pragma omp critical (result)
#endif
			{
				
				res.push_back(r);
				
			}
		}
	}

	for(uint32_t i = 0; i < pyramid.data.size(); i++)
	{
		ChnsDataRelease(pyramid.data[i]);
	}

	SuppressResult(res, result, nms);
}


// AcfDetectImg Function For Stereo Vision Application

void AcfDetectImgScale(Mat& img, PyramidOpt& opt, AcfDetector& detector, vector<DetectResult>& result, 
	int scaleindex, float epipolarLine, float nms)
{
	Pyramid pyramid;
	vector<DetectResult> res;
	int32_t shiftX, shiftY;

	ChnsPyramidScale(img, opt, pyramid, scaleindex);

	result.clear();

#if(USE_OPENMP == 1)
	int32_t i;
#pragma omp parallel for private(i) num_threads(3)
	for (i = 0; i < pyramid.nScales; i++)
#else
	for (int32_t i = 0; i < pyramid.nScales; i++)
#endif
	{
		if (epipolarLine > 0){
			shiftY = (detector.modelPadHt - detector.modelHt) / 2 - opt.pad.height;
			epipolarLine = pyramid.scalesHt[scaleindex] * epipolarLine - shiftY;
		}

		vector<DetectResult> r0;
		detector.Detect(pyramid.data[i], opt.chnsOpt.shrink, r0, epipolarLine);
		shiftX = (detector.modelPadWd - detector.modelWd) / 2 - opt.pad.width;
		shiftY = (detector.modelPadHt - detector.modelHt) / 2 - opt.pad.height;
		for (uint32_t j = 0; j < r0.size(); j++)
		{
			DetectResult r = r0[j];
			r.cs = (r.cs + shiftX) / pyramid.scalesWd[scaleindex];
			r.rs = (r.rs + shiftY) / pyramid.scalesHt[scaleindex];
			r.modelWd = detector.modelWd / pyramid.scales[scaleindex];
			r.modelHt = detector.modelHt / pyramid.scales[scaleindex];
			r.scaleindex = scaleindex;
#if(USE_OPENMP == 1)
#pragma omp critical (result)
#endif
			{
				res.push_back(r);
			}
		}
	}

	for (uint32_t i = 0; i < pyramid.data.size(); i++)
	{
		ChnsDataRelease(pyramid.data[i]);
	}

	SuppressResult(res, result, nms);
}

void SuppressResult(vector<DetectResult>& ri, vector<DetectResult>& ro, float nms)
{
	bool* kp = new bool[ri.size()];
	float* as = new float[ri.size()];
	float* xs = new float[ri.size()];
	float* xe = new float[ri.size()];
	float* ys = new float[ri.size()];
	float* ye = new float[ri.size()];

	// For each i suppress all j st j > i and area-overlap > overlap
	std::sort(ri.begin(), ri.end(), greater<DetectResult>());

	for(uint32_t i = 0; i < ri.size(); i++)
	{
		kp[i] = true;
		as[i] = ri[i].modelWd * ri[i].modelHt;
		xs[i] = ri[i].cs;
		xe[i] = xs[i] + ri[i].modelWd;
		ys[i] = ri[i].rs;
		ye[i] = ys[i] + ri[i].modelHt;
	}

	for(uint32_t i = 0; i < ri.size(); i++)
	{
		if(kp[i] == 0)
		{
			continue;
		}
		for(uint32_t j = i + 1; j < ri.size(); j++)
		{
			if(kp[j] == false)
			{
				continue;
			}
			float iw = min(xe[i], xe[j]) - max(xs[i], xs[j]);
			if(iw <= 0)
			{
				continue;
			}
			float ih = min(ye[i], ye[j]) - max(ys[i], ys[j]);
			if(ih <= 0)
			{
				continue;
			}
			float u = min(as[i], as[j]);
			float o = iw * ih / u;
			if(o > nms)
			{
				kp[j] = false;
			}
		}
	}

	for(uint32_t i = 0; i < ri.size(); i++)
	{
		if(kp[i] == true)
		{
			ro.push_back(ri[i]);
		}
	}

	delete [] kp;
	delete [] as;
}

}