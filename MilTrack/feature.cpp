/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1, and see useage before run the program to track
// the target by MIL or OBA method. The code referenced Boris's paper and code
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#include "common.h"

namespace pc{

float sumRect_1channel(fimg ii_img, pc::Rect rect){
	assert(ii_img.chns == 1);
	const float tl = ii_img.data[rect.y * ii_img.cols + rect.x];
	const float tr = ii_img.data[rect.y * ii_img.cols + rect.x + rect.width];
	const float br = ii_img.data[(rect.y + rect.height) * ii_img.cols + rect.x + rect.width];
	const float bl = ii_img.data[(rect.y + rect.height) * ii_img.cols + rect.x];
	return br + tl - tr - bl;
}

void computeintegral_1channel(uimg& img, fimg& ii_img){
	ii_img.chns = img.chns;
	const int cols = ii_img.cols = img.cols;
	const int rows = ii_img.rows = img.rows;
	ii_img.data[0] = (float)img.data[0];
	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
			if (i == 0 && j == 0)
				continue;
			if (i == 0 && j != 0)
				ii_img.data[j] = ii_img.data[j - 1] + img.data[j];
			if (i != 0 && j == 0)
				ii_img.data[i*cols] = ii_img.data[(i - 1)*cols] + img.data[i*cols];
			if (i != 0 && j != 0){
				ii_img.data[i*cols + j] = 
					ii_img.data[(i - 1)*cols + j] + ii_img.data[i*cols + j - 1]
					- ii_img.data[(i - 1)*cols + j - 1] + img.data[i*cols + j];
			}
		}
	}
}

void compute_integral(uimg& img, fimg& ii_img){
	if (img.chns == 1){
		computeintegral_1channel(img, ii_img);
	}
	else{
		assert(false);
	}
}

//extern FeatureParam ftrparam;
vector<Haar*> generatefeature(int numFeat, FeatureParam &ftrparam){
	// 初始化参数
	ftrparam.useChannels[0] = 0;

	vector<Haar*> haars;
	haars.resize(numFeat);
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < numFeat; i++){
		haars[i] = new Haar;

		haars[i]->width = ftrparam.width;
		haars[i]->height = ftrparam.height;
		const int numrects = randint(ftrparam.minRectNum, ftrparam.maxRectNum);
		haars[i]->rects.resize(numrects);
		haars[i]->weights.resize(numrects);
		haars[i]->rsums.resize(numrects);
		haars[i]->maxSum = 0.0f;

		haars[i]->channel = 0;

		for (int j = 0; j < numrects; j++){
			haars[i]->weights[j] = randfloat() * 2 - 1;   
			haars[i]->rects[j].x = randint(0, (uint)(ftrparam.width - 3));
			haars[i]->rects[j].y = randint(0, (uint)(ftrparam.height - 3));
			haars[i]->rects[j].width = randint(1, ftrparam.width - haars[i]->rects[j].x - 2);
			haars[i]->rects[j].height = randint(1, ftrparam.height - haars[i]->rects[j].y - 2);
			haars[i]->rsums[j] = abs(255 * haars[i]->weights[j] * 
				(haars[i]->rects[j].width + 1) * (haars[i]->rects[j].height + 1));
		}

		if (ftrparam.numCh < 0){
			ftrparam.numCh = 0;
			for (int j = 0; j < 1024; j++){
				ftrparam.numCh += ftrparam.useChannels >= 0;
			}
		}
	}

	return haars;
}

vector<Haar*> generatefeature_mc(int numFeat, FeatureParam &ftrparam, int channels){
	// 初始化参数
	ftrparam.useChannels[0] = 0;
	const int numfeat = numFeat;

	vector<Haar*> haars;
	haars.resize(numfeat);
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < numfeat; i++){
		haars[i] = new Haar;

		haars[i]->width = ftrparam.width;
		haars[i]->height = ftrparam.height;
		const int numrects = randint(ftrparam.minRectNum, ftrparam.maxRectNum);
		haars[i]->rects.resize(numrects);
		haars[i]->weights.resize(numrects);
		haars[i]->rsums.resize(numrects);
		haars[i]->maxSum = 0.0f;

		haars[i]->channel = randint(0, channels-1);

		for (int j = 0; j < numrects; j++){
			haars[i]->weights[j] = randfloat() * 2 - 1;
			haars[i]->rects[j].x = randint(0, (uint)(ftrparam.width - 3));
			haars[i]->rects[j].y = randint(0, (uint)(ftrparam.height - 3));
			haars[i]->rects[j].width = randint(1, ftrparam.width - haars[i]->rects[j].x - 2);
			haars[i]->rects[j].height = randint(1, ftrparam.height - haars[i]->rects[j].y - 2);
			haars[i]->rsums[j] = abs(255 * haars[i]->weights[j] *
				(haars[i]->rects[j].width + 1) * (haars[i]->rects[j].height + 1));
		}

		if (ftrparam.numCh < 0){
			ftrparam.numCh = 0;
			for (int j = 0; j < 1024; j++){
				ftrparam.numCh += ftrparam.useChannels >= 0;
			}
		}
	}

	return haars;
}

float computefeature(uimg img, fimg ii_img, Haar* haar, pc::Rect rect){
	float sum = 0.0f; 
	pc::Rect r;
	
	for (int k = 0; k < (int)haar->rects.size(); k++){
		r.x = haar->rects[k].x;
		r.y = haar->rects[k].y;
		r.width = haar->rects[k].width;
		r.height = haar->rects[k].height;

		r.x += rect.x;
		r.y += rect.y;
		sum += haar->weights[k] * sumRect_1channel(ii_img, r);
	}

	return sum;
}

float computefeature_mc(vector<uimg>& img, vector<fimg>& ii_img, Haar* haar, pc::Rect rect){
	float sum = 0.0f;
	pc::Rect r;

	const int rectsize = haar->rects.size();
	for (int k = 0; k < rectsize; k++){
		r.x = haar->rects[k].x;
		r.y = haar->rects[k].y;
		r.width = haar->rects[k].width;
		r.height = haar->rects[k].height;

		r.x += rect.x;
		r.y += rect.y;
		sum += haar->weights[k] * sumRect_1channel(ii_img[haar->channel], r);
	}

	return sum;
}

vector<vector<float>> computefeature_set(uimg img, fimg ii_img, vector<pc::Rect>& samples, vector<Haar*>& ftrs){
	const int numftrs = ftrs.size();
	const int numsamples = samples.size();

	vector<vector<float>> ftrVals;
	if (numsamples == 0) return ftrVals;
	ftrVals.resize(numftrs);
	for (int k = 0; k < numftrs; k++)
		ftrVals[k].resize(numsamples);

#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < numftrs; i++){
		for (int j = 0; j < numsamples; j++){
			ftrVals[i][j] = computefeature(img, ii_img, ftrs[i], samples[j]);
		}
	}

	return ftrVals;
}

vector<vector<float>> computefeature_set_mc(vector<uimg>& img, vector<fimg>& ii_img, vector<pc::Rect>& samples, vector<Haar*>& ftrs){
	const int numftrs = ftrs.size();
	const int numsamples = samples.size();

	vector<vector<float>> ftrVals;
	if (numsamples == 0) return ftrVals;
	ftrVals.resize(numftrs);
#ifdef _OPENMP
#pragma omp parallel for
#endif	
	for (int k = 0; k < numftrs; k++)
		ftrVals[k].resize(numsamples);

#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < numftrs; i++){
		for (int j = 0; j < numsamples; j++){
			ftrVals[i][j] = computefeature_mc(img, ii_img, ftrs[i], samples[j]);
		}
	}

	return ftrVals;
}

}

