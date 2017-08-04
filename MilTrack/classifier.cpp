/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1, and see useage before run the program to track
// the target by MIL or OBA method. The code referenced Boris's paper and code
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#include "classifier.h"
#include "feature.h"

namespace pc{

extern TrackParam trparam;

vector<float> online_classifysetf(Weak& weakclf, vector<pc::Rect> sampleset, 
	vector<vector<float>> ftrVal, fimg ii_img, bool ftrsComputed){

	const int sampsize = sampleset.size();
	vector<float> res(sampsize);
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int k = 0; k < sampsize; k++){
		bool lftrsComputed = ftrsComputed && sampleset.size() > 0 && ftrVal[0].size() > 0;
		float xx;
		if (ftrsComputed){
			xx = ftrVal[weakclf.index][k];
		}
		else{
			uimg u; u.chns = u.cols = u.rows = 0; u.data = NULL;
			xx = computefeature(u, ii_img, weakclf.ftrs, sampleset[k]);
		}
		
		const double p0 = exp((xx - weakclf.mu0)*(xx - weakclf.mu0)*weakclf.e0) * weakclf.n0;
		const double p1 = exp((xx - weakclf.mu1)*(xx - weakclf.mu1)*weakclf.e1) * weakclf.n1;
		res[k] = (float)(log(1e-5 + p1) - log(1e-5 + p0));
	}
	return res;
}

vector<float> online_classifysetf_mc(Weak& weakclf, vector<pc::Rect> sampleset,
	vector<vector<float>> ftrVal, vector<fimg>& ii_img, bool ftrsComputed){

	const int sampsize = sampleset.size();
	vector<float> res(sampsize);
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int k = 0; k < sampsize; k++){
		const bool lftrsComputed = ftrsComputed && sampleset.size() > 0 && ftrVal[0].size() > 0;
		float xx;
		if (ftrsComputed){
			xx = ftrVal[weakclf.index][k];
		} else{
			vector<uimg> u;
			xx = computefeature_mc(u, ii_img, weakclf.ftrs, sampleset[k]);
		}

		const double p0 = exp((xx - weakclf.mu0)*(xx - weakclf.mu0)*weakclf.e0) * weakclf.n0;
		const double p1 = exp((xx - weakclf.mu1)*(xx - weakclf.mu1)*weakclf.e1) * weakclf.n1;
		res[k] = (float)(log(1e-5 + p1) - log(1e-5 + p0));
	}
	return res;
}

void online_trainupdate(Weak& weakclf, int posxsize, int negxsize, 
	vector<vector<float>>& posftrVal, vector<vector<float>>& negftrVal){

	float posmu = 0.0f, negmu = 0.0f;
	if (posxsize > 0) posmu = vecmean(posftrVal[weakclf.index]);
	if (negxsize > 0) negmu = vecmean(negftrVal[weakclf.index]);

	if (weakclf.trained){
		if (posxsize > 0){
			weakclf.mu1 = weakclf.lRate * weakclf.mu1 + (1 - weakclf.lRate) * posmu;
			vector<float> tmpp = vecsub(posftrVal[weakclf.index], weakclf.mu1);
			vector<float> tmp = vecsqr(tmpp);
			weakclf.sig1 = weakclf.lRate * weakclf.sig1 + (1 - weakclf.lRate) * vecmean(tmp);
		}
		if (negxsize > 0){
			weakclf.mu0 = weakclf.lRate * weakclf.mu0 + (1 - weakclf.lRate) * negmu;
			vector<float> tmpp = vecsub(posftrVal[weakclf.index], weakclf.mu0);
			vector<float> tmp = vecsqr(tmpp);
			weakclf.sig0 = weakclf.lRate * weakclf.sig0 + (1 - weakclf.lRate) * vecmean(tmp);
		}
	}
	else{
		weakclf.trained = true;
		if (posxsize > 0){
			weakclf.mu1 = posmu;
			weakclf.sig1 = vecvar(posftrVal[weakclf.index]) + espf;
		}
		if (negxsize > 0){
			weakclf.mu0 = negmu;
			weakclf.sig0 = vecvar(negftrVal[weakclf.index]) + espf;
		}
	}

	weakclf.q = (weakclf.mu1 - weakclf.mu0) / 2;
	weakclf.s = sign(weakclf.mu1 - weakclf.mu0);
	weakclf.n0 = 1.0f / pow(weakclf.sig0, 0.5f);
	weakclf.n1 = 1.0f / pow(weakclf.sig1, 0.5f);
	weakclf.e1 = -1.0f / (2.0f * weakclf.sig1 + esp);
	weakclf.e0 = -1.0f / (2.0f * weakclf.sig0 + esp);
}

vector<bool> online_classifyset(Weak& weakclf, vector<pc::Rect> sampleset, 
	vector<vector<float>> ftrVal, fimg ii_img, bool ftrsComputed){

	vector<bool> res(sampleset.size());
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int k = 0; k < (int)res.size(); k++){
		bool lftrsComputed = ftrsComputed && sampleset.size() > 0 && ftrVal[0].size() > 0;
		float xx;
		if (ftrsComputed){
			xx = ftrVal[weakclf.index][k];
		} else{
			uimg u; u.chns = u.cols = u.rows = 0; u.data = NULL;
			xx = computefeature(u, ii_img, weakclf.ftrs, sampleset[k]);
		}
		
		double p0 = exp((xx - weakclf.mu0)*(xx - weakclf.mu0)*weakclf.e0) * weakclf.n0;
		double p1 = exp((xx - weakclf.mu1)*(xx - weakclf.mu1)*weakclf.e1) * weakclf.n1;
		res[k] = (p1 > p0);
	}
	return res;
}

vector<bool> online_classifyset_mc(Weak& weakclf, vector<pc::Rect> sampleset,
	vector<vector<float>> ftrVal, vector<fimg>& ii_img, bool ftrsComputed){

	const int sampsize = sampleset.size();
	vector<bool> res(sampsize);
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int k = 0; k < (int)sampsize; k++){
		const bool lftrsComputed = ftrsComputed && sampleset.size() > 0 && ftrVal[0].size() > 0;
		float xx;
		if (ftrsComputed){
			xx = ftrVal[weakclf.index][k];
		} else{
			vector<uimg> u;
			xx = computefeature_mc(u, ii_img, weakclf.ftrs, sampleset[k]);
		}

		const double p0 = exp((xx - weakclf.mu0)*(xx - weakclf.mu0)*weakclf.e0) * weakclf.n0;
		const double p1 = exp((xx - weakclf.mu1)*(xx - weakclf.mu1)*weakclf.e1) * weakclf.n1;
		res[k] = (p1 > p0);
	}
	return res;
}

}
