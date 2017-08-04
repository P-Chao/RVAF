/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build 
// this work with opencv > 2.3.1, and see useage before run the program to track  
// the target by MIL or OBA method. The code referenced Boris's paper and code 
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal 
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#include "common.h"
#include "feature.h"
#include "train.h"
#ifdef _OPENMP
#include <omp.h>
#endif

using namespace cv;

namespace pc{

vector<pc::Rect> getsample(uimg& img, pc::Rect box, float inrad,
					float outrad, int maxnum){
	vector<pc::Rect> samples;
	int rowsz = img.rows - box.height - 1;
	int colsz = img.cols - box.width - 1;
	float inradsq = inrad*inrad;
	float outradsq = outrad*outrad;

	const uint minrow = max(0, (int)box.y - (int)inrad);
	const uint maxrow = min((int)rowsz - 1, (int)box.y + (int)inrad);
	const uint mincol = max(0, (int)box.x - (int)inrad);
	const uint maxcol = min((int)colsz - 1, (int)box.x + (int)inrad);

	samples.resize((maxrow - minrow + 1)*(maxcol - mincol + 1));
	int i = 0;

	float prob = ((float)(maxnum)) / samples.size();
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int r = minrow; r <= maxrow; r++)
		for (int c = mincol; c <= maxcol; c++){
		const int dist = (box.y - r)*(box.y - r) + (box.x - c)*(box.x - c);
		if (randfloat()<prob && dist < inradsq && dist >= outradsq){
			samples[i].x = c;
			samples[i].y = r;
			samples[i].height = box.height;
			samples[i].width = box.width;
			i++;
		}
		}

	samples.resize(min(i, maxnum));
	return samples;
}

vector<pc::Rect> getsamplenum(uimg& img, uint num, int w, int h){
	int rowsz = img.rows - h - 1;
	int colsz = img.cols - w - 1;

	vector<pc::Rect> samples(num);
	for (int i = 0; i < num; i++){
		samples[i].x = randint(0, colsz);
		samples[i].y = randint(0, rowsz);
		samples[i].width = w;
		samples[i].height = h;
	}
	return samples;
}

//FeatureParam ftrparam;
//TrackParam trparam;
void miltrack_firstframe(Mat& frame, pc::Rect& init_rect, TrackParam& trparam, FeatureParam& ftrparam){
	// 初始化变量
	for (int i = 0; i < 1024; i++)
		ftrparam.useChannels[i] = -1;
	ftrparam.useChannels[0] = 0;
	
	//图像装入容器
	uimg img;
	img.chns = frame.channels();
	img.cols = frame.cols;
	img.rows = frame.rows;
	img.data = frame.data;

	// 初始时设置当前实例窗口
	ftrparam.width = init_rect.width;
	ftrparam.height = init_rect.height;

	// 设置当前窗口
	pc::Rect curt_rect;
	curt_rect.x = init_rect.x;
	curt_rect.y = init_rect.y;
	curt_rect.width = init_rect.width;
	curt_rect.height = init_rect.height;

	// 保存当前窗口到全局
	trparam.curt_rect.x = curt_rect.x;
	trparam.curt_rect.y = curt_rect.y;
	trparam.curt_rect.width = curt_rect.width;
	trparam.curt_rect.height = curt_rect.height;

	// 计算积分图
	fimg ii_img;
	ii_img.data = (float *)calloc(img.cols * img.rows * img.chns,sizeof(float));
	compute_integral(img, ii_img);

	// 生成特征
	int numFeat = trparam.numFeat;		//250;
	trparam.haars = generatefeature(numFeat, ftrparam);
	assert(numFeat == trparam.haars.size());

	// 弱分类器
	float lRate = trparam.lRate;//0.85f;
	trparam.weakclf.resize(numFeat);

#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int k = 0; k < numFeat; k++){
		trparam.weakclf[k].trained = false;
		trparam.weakclf[k].index = k;
		trparam.weakclf[k].ftrs = trparam.haars[k];
		trparam.weakclf[k].lRate = lRate;
	}

	// 采集正样本窗口
	float init_postrainrad	= trparam.init_postrainrad; //3.0f;
	int init_negnumtrain	= trparam.init_negnumtrain;	//65;
	int negnumtrain			= trparam.negnumtrain;		//65;
	int srchwinsz			= trparam.srchwinsz;		//25;
	vector<pc::Rect> posx;
	posx = getsample(img, curt_rect, init_postrainrad, 0, 1000000);

	// 采集负样本窗口
	vector<pc::Rect> negx;
	negx = getsample(img, curt_rect, 2.0f*srchwinsz, 1.5f*init_postrainrad, init_negnumtrain);
	if (posx.size() < 1 || negx.size() < 1){
		assert(false);
		return;
	}

	// 初始的训练
	mil_trainupdate(img, ii_img, trparam.weakclf, trparam.haars, posx, negx, trparam);
	negx.clear();
	
	// 清理内存
	free(ii_img.data);
}

void miltrack_frame(Mat& frame, pc::Rect& curt_rect, TrackParam& trparam, FeatureParam& ftrparam){
	//图像装入容器
	uimg img;
	img.chns = frame.channels();
	img.cols = frame.cols;
	img.rows = frame.rows;
	img.data = frame.data;

	// 初始化当前窗口
	//pc::Rect curt_rect;
	trparam.curt_rect.x = curt_rect.x;
	trparam.curt_rect.y = curt_rect.y;
	trparam.curt_rect.width = curt_rect.width;
	trparam.curt_rect.height = curt_rect.height;

	// 计算积分图
	fimg ii_img;
	ii_img.data = (float *)calloc(img.cols * img.rows * img.chns, sizeof(float));
	compute_integral(img, ii_img);
	

	// 在search window运行当前分类器，并获取概率图
	vector<pc::Rect> posx, negx, detectx;
	int srchwinsz = trparam.srchwinsz;//25;
	detectx = getsample(img, curt_rect, srchwinsz, 0.0f, 1000000);

	vector<float> prob;
	vector<vector<float>> ftrValn;
	ftrValn.clear();
	prob = milclassify(trparam.weakclf, detectx, ftrValn, ii_img, trparam.uselogR, trparam);

	// 寻找最优位置
	int bestidx = vecmaxindex(prob);
	double resp = prob[bestidx];

	curt_rect.x = detectx[bestidx].x;
	curt_rect.y = detectx[bestidx].y;

	// 训练局部分类器
	if (trparam.negsamplestrat == 0){
		negx = getsamplenum(img, trparam.negnumtrain, curt_rect.width, curt_rect.height);
	} else{
		negx = getsample(img, curt_rect, 1.5f*trparam.srchwinsz,
			trparam.posradtrain + 5, trparam.negnumtrain);
	}
	
	if (trparam.posradtrain == 1){
		posx.push_back(curt_rect);
	} else {
		posx = getsample(img, curt_rect, trparam.posradtrain, 0, trparam.posmaxtrain);
	}
	
	mil_trainupdate(img, ii_img, trparam.weakclf, trparam.haars, posx, negx, trparam);
	trparam.count++;

	// 保存当前窗口到全局
	trparam.curt_rect.x = curt_rect.x;
	trparam.curt_rect.y = curt_rect.y;
	trparam.curt_rect.width = curt_rect.width;
	trparam.curt_rect.height = curt_rect.height;

	// 清理内存
	posx.clear(); negx.clear(); detectx.clear();
	free(ii_img.data);

}

void adatrack_firstframe(Mat& frame, pc::Rect& init_rect, TrackParam& trparam, FeatureParam& ftrparam){
	uimg img;
	img.chns = frame.channels();
	img.cols = frame.cols;
	img.rows = frame.rows;
	img.data = frame.data;

	// 初始时设置当前实例窗口
	ftrparam.width = init_rect.width;
	ftrparam.height = init_rect.height;

	pc::Rect curt_rect;
	curt_rect.x = init_rect.x;
	curt_rect.y = init_rect.y;
	curt_rect.width = init_rect.width;
	curt_rect.height = init_rect.height;

	// 保存当前窗口到全局
	trparam.curt_rect.x = curt_rect.x;
	trparam.curt_rect.y = curt_rect.y;
	trparam.curt_rect.width = curt_rect.width;
	trparam.curt_rect.height = curt_rect.height;

	// 计算积分图
	fimg ii_img;
	ii_img.data = (float *)calloc(img.cols * img.rows * img.chns, sizeof(float));
	compute_integral(img, ii_img);

	// 根据特征数量， 创建特征矩阵
	trparam.numsamples = 0;
	int numSel = trparam.numSel;
	int numFeat = trparam.numFeat;
	if (numSel > numFeat || numSel < 1){
		numSel = numFeat / 2;
	}

	trparam.countFPv.resize(numSel);
	trparam.countFNv.resize(numSel);
	trparam.countTPv.resize(numSel);
	trparam.countTNv.resize(numSel);
	for (int i = 0; i < numSel; i++){
		trparam.countFPv[i].resize(numFeat, 1.0f);
		trparam.countFNv[i].resize(numFeat, 1.0f);
		trparam.countTPv[i].resize(numFeat, 1.0f);
		trparam.countTNv[i].resize(numFeat, 1.0f);
	}
	trparam.alphas.resize(numSel, 0);
	
	// 生成特征，初始化弱分类器
	trparam.haars = generatefeature(numFeat, ftrparam);
	trparam.selectors.resize(numSel, 0);
	trparam.weakclf.resize(numFeat);
	for (int k = 0; k < numFeat; k++){
		trparam.weakclf[k].trained = false;
		trparam.weakclf[k].index = k;
		trparam.weakclf[k].ftrs = trparam.haars[k];
		trparam.weakclf[k].lRate = trparam.lRate;
	}

	// 采集正样本窗口
	float init_postrainrad = trparam.init_postrainrad; //3.0f;
	int init_negnumtrain = trparam.init_negnumtrain;	//65;
	int negnumtrain = trparam.negnumtrain;		//65;
	int srchwinsz = trparam.srchwinsz;		//25;
	vector<pc::Rect> posx;
	posx = getsample(img, curt_rect, init_postrainrad, 0, 1000000);

	// 采集负样本窗口
	vector<pc::Rect> negx;
	negx = getsample(img, curt_rect, 2.0f*srchwinsz, 1.5f*init_postrainrad, init_negnumtrain);
	if (posx.size() < 1 || negx.size() < 1){
		assert(false);
		return;
	}

	// 初始的训练
	ada_trainupdate(img, ii_img, trparam.weakclf, trparam.haars, posx, negx, trparam);
	negx.clear();

	// 清理内存
	free(ii_img.data);
}

void adatrack_frame(Mat& frame, pc::Rect& curt_rect, TrackParam& trparam, FeatureParam& ftrparam){

	//图像装入容器
	uimg img;
	img.chns = frame.channels();
	img.cols = frame.cols;
	img.rows = frame.rows;
	img.data = frame.data;

	// 初始化当前窗口
	//::Rect curt_rect;
	trparam.curt_rect.x = curt_rect.x;
	trparam.curt_rect.y = curt_rect.y;
	trparam.curt_rect.width = curt_rect.width;
	trparam.curt_rect.height = curt_rect.height;

	// 计算积分图
	fimg ii_img;
	ii_img.data = (float *)calloc(img.cols * img.rows * img.chns, sizeof(float));
	compute_integral(img, ii_img);

	// 在search window运行当前分类器，并获取概率图
	vector<pc::Rect> posx, negx, detectx;
	int srchwinsz = trparam.srchwinsz;//25;
	detectx = getsample(img, curt_rect, srchwinsz, 0.0f, 1000000);

	vector<float> prob;
	vector<vector<float>> ftrValn;
	ftrValn.clear();
	prob = adaclassify(trparam.weakclf, detectx, ftrValn, ii_img, trparam.uselogR, trparam);

	// 寻找最优位置
	int bestidx = vecmaxindex(prob);
	double resp = prob[bestidx];

	curt_rect.x = detectx[bestidx].x;
	curt_rect.y = detectx[bestidx].y;

	// 训练局部分类器
	if (trparam.negsamplestrat == 0){
		negx = getsamplenum(img, trparam.negnumtrain, curt_rect.width, curt_rect.height);
	}
	else{
		negx = getsample(img, curt_rect, 1.5f*trparam.srchwinsz,
			trparam.posradtrain + 5, trparam.negnumtrain);
	}

	if (trparam.posradtrain == 1){
		posx.push_back(curt_rect);
	}
	else {
		posx = getsample(img, curt_rect, trparam.posradtrain, 0, trparam.posmaxtrain);
	}

	ada_trainupdate(img, ii_img, trparam.weakclf, trparam.haars, posx, negx, trparam);
	trparam.count++;

	// 保存当前窗口到全局
	trparam.curt_rect.x = curt_rect.x;
	trparam.curt_rect.y = curt_rect.y;
	trparam.curt_rect.width = curt_rect.width;
	trparam.curt_rect.height = curt_rect.height;

	// 清理内存
	posx.clear(); negx.clear(); detectx.clear();
	free(ii_img.data);
}

/***********************************For SVAF**************************************/

Binotrack_param binotrack_param;

//void miltrack_firstframe_sync(Mat& frame, pc::Rect& init_rect, TrackParam& trparam, FeatureParam& ftrparam){
void miltrack_firstframe_sync(Mat& frame0, pc::Rect& init_rect0, TrackParam& trparam0, FeatureParam& ftrparam0,
	Mat& frame1, pc::Rect& init_rect1, TrackParam& trparam1, FeatureParam& ftrparam1){
	// 初始化变量
	for (int i = 0; i < 1024; i++){
		ftrparam0.useChannels[i] = -1;
		ftrparam1.useChannels[i] = -1;
	}
		
	ftrparam0.useChannels[0] = 0;
	ftrparam1.useChannels[0] = 0;

	//图像装入容器
	uimg img0, img1;
	img0.chns = frame0.channels();
	img0.cols = frame0.cols;
	img0.rows = frame0.rows;
	img0.data = frame0.data;
	img1.chns = frame1.channels();
	img1.cols = frame1.cols;
	img1.rows = frame1.rows;
	img1.data = frame1.data;

	// 初始时设置当前实例窗口
	ftrparam0.width = init_rect0.width;
	ftrparam0.height = init_rect0.height;
	ftrparam1.width = init_rect1.width;
	ftrparam1.height = init_rect1.height;

	// 设置当前窗口
	pc::Rect curt_rect0, curt_rect1;
	curt_rect0.x = init_rect0.x;
	curt_rect0.y = init_rect0.y;
	curt_rect0.width = init_rect0.width;
	curt_rect0.height = init_rect0.height;
	curt_rect1.x = init_rect1.x;
	curt_rect1.y = init_rect1.y;
	curt_rect1.width = init_rect1.width;
	curt_rect1.height = init_rect1.height;

	// 保存当前窗口到全局
	trparam0.curt_rect.x = curt_rect0.x;
	trparam0.curt_rect.y = curt_rect0.y;
	trparam0.curt_rect.width = curt_rect0.width;
	trparam0.curt_rect.height = curt_rect0.height;
	trparam1.curt_rect.x = curt_rect1.x;
	trparam1.curt_rect.y = curt_rect1.y;
	trparam1.curt_rect.width = curt_rect1.width;
	trparam1.curt_rect.height = curt_rect1.height;

	// 计算积分图
	fimg ii_img0, ii_img1;
	ii_img0.data = (float *)calloc(img0.cols * img0.rows * img0.chns, sizeof(float));
	compute_integral(img0, ii_img0);
	ii_img1.data = (float *)calloc(img1.cols * img1.rows * img1.chns, sizeof(float));
	compute_integral(img1, ii_img1);

	// 生成特征
	int numFeat0 = trparam0.numFeat;		//250;
	trparam0.haars = generatefeature(numFeat0, ftrparam0);
	assert(numFeat0 == trparam0.haars.size());
	int numFeat1 = trparam1.numFeat;		//250;
	trparam1.haars = generatefeature(numFeat1, ftrparam1);
	assert(numFeat1 == trparam1.haars.size());

	// 弱分类器
	float lRate0 = trparam0.lRate;//0.85f;
	trparam0.weakclf.resize(numFeat0);
	float lRate1 = trparam1.lRate;//0.85f;
	trparam1.weakclf.resize(numFeat1);

	for (int k = 0; k < numFeat0; k++){
		trparam0.weakclf[k].trained = false;
		trparam0.weakclf[k].index = k;
		trparam0.weakclf[k].ftrs = trparam0.haars[k];
		trparam0.weakclf[k].lRate = lRate0;
	}

	for (int k = 0; k < numFeat1; k++){
		trparam1.weakclf[k].trained = false;
		trparam1.weakclf[k].index = k;
		trparam1.weakclf[k].ftrs = trparam1.haars[k];
		trparam1.weakclf[k].lRate = lRate1;
	}

	// 采集正样本窗口
	float init_postrainrad0 = trparam0.init_postrainrad; //3.0f;
	int init_negnumtrain0 = trparam0.init_negnumtrain;	//65;
	int negnumtrain0 = trparam0.negnumtrain;		//65;
	int srchwinsz0 = trparam0.srchwinsz;		//25;
	vector<pc::Rect> posx0;
	posx0 = getsample(img0, curt_rect0, init_postrainrad0, 0, 1000000);

	float init_postrainrad1 = trparam1.init_postrainrad; //3.0f;
	int init_negnumtrain1 = trparam1.init_negnumtrain;	//65;
	int negnumtrain1 = trparam1.negnumtrain;		//65;
	int srchwinsz1 = trparam1.srchwinsz;		//25;
	vector<pc::Rect> posx1;
	posx1 = getsample(img1, curt_rect1, init_postrainrad1, 0, 1000000);

	// 采集负样本窗口
	vector<pc::Rect> negx0;
	negx0 = getsample(img0, curt_rect0, 2.0f*srchwinsz0, 1.5f*init_postrainrad0, init_negnumtrain0);
	if (posx0.size() < 1 || negx0.size() < 1){
		assert(false);
		return;
	}

	vector<pc::Rect> negx1;
	negx1 = getsample(img1, curt_rect1, 2.0f*srchwinsz1, 1.5f*init_postrainrad1, init_negnumtrain1);
	if (posx1.size() < 1 || negx1.size() < 1){
		assert(false);
		return;
	}

	// 初始的训练
	mil_trainupdate(img0, ii_img0, trparam0.weakclf, trparam0.haars, posx0, negx0, trparam0);
	if (!binotrack_param.pool){
		mil_trainupdate(img1, ii_img1, trparam1.weakclf, trparam1.haars, posx1, negx1, trparam1);
	}
	if (binotrack_param.mixfeat || binotrack_param.pool){
		if (!binotrack_param.pool){
			mil_trainupdate(img1, ii_img1, trparam0.weakclf, trparam0.haars, posx1, negx1, trparam0);
		}
		mil_trainupdate(img0, ii_img0, trparam1.weakclf, trparam1.haars, posx0, negx0, trparam1);
	}

	negx0.clear();
	negx1.clear();

	// 清理内存
	free(ii_img0.data);
	free(ii_img1.data);
}



void miltrack_frame_sync(Mat& frame0, pc::Rect& curt_rect0, TrackParam& trparam0, FeatureParam& ftrparam0,
	Mat& frame1, pc::Rect& curt_rect1, TrackParam& trparam1, FeatureParam& ftrparam1){
	//图像装入容器
	uimg img0, img1;
	img0.chns = frame0.channels();
	img0.cols = frame0.cols;
	img0.rows = frame0.rows;
	img0.data = frame0.data;
	img1.chns = frame1.channels();
	img1.cols = frame1.cols;
	img1.rows = frame1.rows;
	img1.data = frame1.data;

	// 初始化当前窗口
	//pc::Rect curt_rect;
	trparam0.curt_rect.x = curt_rect0.x;
	trparam0.curt_rect.y = curt_rect0.y;
	trparam0.curt_rect.width = curt_rect0.width;
	trparam0.curt_rect.height = curt_rect0.height;
	trparam1.curt_rect.x = curt_rect1.x;
	trparam1.curt_rect.y = curt_rect1.y;
	trparam1.curt_rect.width = curt_rect1.width;
	trparam1.curt_rect.height = curt_rect1.height;

	// 计算积分图
	fimg ii_img0, ii_img1;
	ii_img0.data = (float *)calloc(img0.cols * img0.rows * img0.chns, sizeof(float));
	ii_img1.data = (float *)calloc(img1.cols * img1.rows * img1.chns, sizeof(float));
	compute_integral(img0, ii_img0);
	compute_integral(img1, ii_img1);


	// 在search window运行当前分类器，并获取概率图
	vector<pc::Rect> posx0, negx0, detectx0, posx1, negx1, detectx1;
	int srchwinsz0 = trparam0.srchwinsz;//25;
	int srchwinsz1 = trparam1.srchwinsz;//25;
	detectx0 = getsample(img0, curt_rect0, srchwinsz0, 0.0f, 1000000);
	detectx1 = getsample(img1, curt_rect1, srchwinsz1, 0.0f, 1000000);

	vector<float> prob0, prob1;
	vector<vector<float>> ftrValn0, ftrValn1;
	ftrValn0.clear();
	ftrValn1.clear();
	prob0 = milclassify(trparam0.weakclf, detectx0, ftrValn0, ii_img0, trparam0.uselogR, trparam0);
	if (binotrack_param.pool){
		prob1 = milclassify(trparam0.weakclf, detectx1, ftrValn1, ii_img1, trparam0.uselogR, trparam0);
	} else{
		prob1 = milclassify(trparam1.weakclf, detectx1, ftrValn1, ii_img1, trparam1.uselogR, trparam1);
	}
	//
	

	// 显示真实的概率图
	if (binotrack_param.showprob){
		float pmin0, pmax0, pmin1, pmax1, pmin, pmax;
		vecminmax(prob0, pmin0, pmax0);
		vecminmax(prob1, pmin1, pmax1);
		pmin = ((pmin0 < pmin1) ? pmin0 : pmin1);
		pmax = ((pmax0 > pmax1) ? pmax0 : pmax1);

		Mat pimg0 = convert2img(prob0, detectx0, img0.rows, img0.cols, pmin0, pmax0);
		Mat pimg1 = convert2img(prob1, detectx1, img1.rows, img1.cols, pmin1, pmax1);
		resize(pimg0, pimg0, Size(640, 480));
		resize(pimg1, pimg1, Size(640, 480));
		imshow("prob0", pimg0);
		imshow("prob1", pimg1);
	}

	// 寻找最优位置
	int bestidx0 = vecmaxindex(prob0);
	double resp0 = prob0[bestidx0];
	curt_rect0.x = detectx0[bestidx0].x;
	curt_rect0.y = detectx0[bestidx0].y;

	int bestidx1 = vecmaxindex(prob1);
	double resp1 = prob1[bestidx1];
	curt_rect1.x = detectx1[bestidx1].x;
	curt_rect1.y = detectx1[bestidx1].y;

	if (binotrack_param.sync){
		if (resp0 > resp1){
			curt_rect1.y = curt_rect0.y;
		} else{
			curt_rect0.y = curt_rect1.y;
		}
	}
	
	//vecmaxindex_sync(curt_rect0, curt_rect1, prob0, prob1, detectx0, detectx1);

	// 训练局部分类器
	if (trparam0.negsamplestrat == 0){
		negx0 = getsamplenum(img0, trparam0.negnumtrain, curt_rect0.width, curt_rect0.height);
	} else{
		negx0 = getsample(img0, curt_rect0, 1.5f*trparam0.srchwinsz,
			trparam0.posradtrain + 5, trparam0.negnumtrain);
	}
	if (trparam1.negsamplestrat == 0){
		negx1 = getsamplenum(img1, trparam1.negnumtrain, curt_rect1.width, curt_rect1.height);
	} else{
		negx1 = getsample(img1, curt_rect1, 1.5f*trparam1.srchwinsz,
			trparam1.posradtrain + 5, trparam1.negnumtrain);
	}

	if (trparam0.posradtrain == 1){
		posx0.push_back(curt_rect0);
	} else {
		posx0 = getsample(img0, curt_rect0, trparam0.posradtrain, 0, trparam0.posmaxtrain);
	}
	if (trparam1.posradtrain == 1){
		posx1.push_back(curt_rect1);
	} else {
		posx1 = getsample(img1, curt_rect1, trparam1.posradtrain, 0, trparam1.posmaxtrain);
	}

	mil_trainupdate(img0, ii_img0, trparam0.weakclf, trparam0.haars, posx0, negx0, trparam0);
	if (!binotrack_param.pool){
		mil_trainupdate(img1, ii_img1, trparam1.weakclf, trparam1.haars, posx1, negx1, trparam1);
	}
	if (binotrack_param.mixfeat || binotrack_param.pool){
		if (!binotrack_param.pool){
			mil_trainupdate(img1, ii_img1, trparam0.weakclf, trparam0.haars, posx1, negx1, trparam0);
		}
		mil_trainupdate(img0, ii_img0, trparam1.weakclf, trparam1.haars, posx0, negx0, trparam1);
	}

	trparam0.count++;
	trparam1.count++;

	// 保存当前窗口到全局
	trparam0.curt_rect.x = curt_rect0.x;
	trparam0.curt_rect.y = curt_rect0.y;
	trparam0.curt_rect.width = curt_rect0.width;
	trparam0.curt_rect.height = curt_rect0.height;
	trparam1.curt_rect.x = curt_rect1.x;
	trparam1.curt_rect.y = curt_rect1.y;
	trparam1.curt_rect.width = curt_rect1.width;
	trparam1.curt_rect.height = curt_rect1.height;

	// 清理内存
	posx0.clear(); negx0.clear(); detectx0.clear();
	posx1.clear(); negx1.clear(); detectx1.clear();
	free(ii_img0.data);
	free(ii_img1.data);

}


void miltrack_firstframe_sync_mc(Mat& frame0, pc::Rect& init_rect0, TrackParam& trparam0, FeatureParam& ftrparam0,
	Mat& frame1, pc::Rect& init_rect1, TrackParam& trparam1, FeatureParam& ftrparam1){
	// 初始化变量
	for (int i = 0; i < 1024; i++){
		ftrparam0.useChannels[i] = -1;
		ftrparam1.useChannels[i] = -1;
	}

	ftrparam0.useChannels[0] = 0;
	ftrparam1.useChannels[0] = 0;

	// 检查输入
	if (frame0.cols != frame1.cols || frame1.rows != frame1.rows 
		|| frame0.channels() != frame1.channels()){
		abort();
	}

	// 计算图像通道
	const int cols = frame0.cols;
	const int rows = frame0.rows;
	const int chns = frame0.channels();
	
	vector<Mat> colorChannels0, colorChannels1;
	split(frame0, colorChannels0);
	split(frame1, colorChannels1);

	//图像装入容器
	vector<uimg> img0, img1;
	img0.resize(chns);
	img1.resize(chns);
	for (int i = 0; i < chns; ++i){
		img0[i].chns = 1;
		img0[i].cols = colorChannels0[i].cols;
		img0[i].rows = colorChannels0[i].rows;
		img0[i].data = colorChannels0[i].data;
		img1[i].chns = 1;
		img1[i].cols = colorChannels1[i].cols;
		img1[i].rows = colorChannels1[i].rows;
		img1[i].data = colorChannels1[i].data;
	}

	// 初始时设置当前实例窗口
	ftrparam0.width = init_rect0.width;
	ftrparam0.height = init_rect0.height;
	ftrparam1.width = init_rect1.width;
	ftrparam1.height = init_rect1.height;

	// 设置当前窗口
	pc::Rect curt_rect0, curt_rect1;
	curt_rect0.x = init_rect0.x;
	curt_rect0.y = init_rect0.y;
	curt_rect0.width = init_rect0.width;
	curt_rect0.height = init_rect0.height;
	curt_rect1.x = init_rect1.x;
	curt_rect1.y = init_rect1.y;
	curt_rect1.width = init_rect1.width;
	curt_rect1.height = init_rect1.height;

	// 保存当前窗口到全局
	trparam0.curt_rect.x = curt_rect0.x;
	trparam0.curt_rect.y = curt_rect0.y;
	trparam0.curt_rect.width = curt_rect0.width;
	trparam0.curt_rect.height = curt_rect0.height;
	trparam1.curt_rect.x = curt_rect1.x;
	trparam1.curt_rect.y = curt_rect1.y;
	trparam1.curt_rect.width = curt_rect1.width;
	trparam1.curt_rect.height = curt_rect1.height;

	// 计算积分图
	vector<fimg> ii_img0, ii_img1;
	ii_img0.resize(chns);
	ii_img1.resize(chns);
	for (int i = 0; i < chns; ++i){
		ii_img0[i].data = (float*)calloc(cols * rows, sizeof(float));
		ii_img1[i].data = (float*)calloc(cols * rows, sizeof(float));
		compute_integral(img0[i], ii_img0[i]);
		compute_integral(img1[i], ii_img1[i]);
	}

	// 生成特征
	int numFeat0 = trparam0.numFeat;		//250;
	trparam0.haars = generatefeature_mc(numFeat0, ftrparam0, chns);
	assert(numFeat0 == trparam0.haars.size());
	int numFeat1 = trparam1.numFeat;		//250;
	trparam1.haars = generatefeature_mc(numFeat1, ftrparam1, chns);
	assert(numFeat1 == trparam1.haars.size());

	// 弱分类器
	float lRate0 = trparam0.lRate;//0.85f;
	trparam0.weakclf.resize(numFeat0);
	float lRate1 = trparam1.lRate;//0.85f;
	trparam1.weakclf.resize(numFeat1);

	for (int k = 0; k < numFeat0; k++){
		trparam0.weakclf[k].trained = false;
		trparam0.weakclf[k].index = k;
		trparam0.weakclf[k].ftrs = trparam0.haars[k];
		trparam0.weakclf[k].lRate = lRate0;
	}

	for (int k = 0; k < numFeat1; k++){
		trparam1.weakclf[k].trained = false;
		trparam1.weakclf[k].index = k;
		trparam1.weakclf[k].ftrs = trparam1.haars[k];
		trparam1.weakclf[k].lRate = lRate1;
	}

	// 采集正样本窗口
	float init_postrainrad0 = trparam0.init_postrainrad; //3.0f;
	int init_negnumtrain0 = trparam0.init_negnumtrain;	//65;
	int negnumtrain0 = trparam0.negnumtrain;		//65;
	int srchwinsz0 = trparam0.srchwinsz;		//25;
	vector<pc::Rect> posx0;
	posx0 = getsample(img0[0], curt_rect0, init_postrainrad0, 0, 1000000);

	float init_postrainrad1 = trparam1.init_postrainrad; //3.0f;
	int init_negnumtrain1 = trparam1.init_negnumtrain;	//65;
	int negnumtrain1 = trparam1.negnumtrain;		//65;
	int srchwinsz1 = trparam1.srchwinsz;		//25;
	vector<pc::Rect> posx1;
	posx1 = getsample(img1[0], curt_rect1, init_postrainrad1, 0, 1000000);

	// 采集负样本窗口
	vector<pc::Rect> negx0;
	negx0 = getsample(img0[0], curt_rect0, 2.0f*srchwinsz0, 1.5f*init_postrainrad0, init_negnumtrain0);
	if (posx0.size() < 1 || negx0.size() < 1){
		assert(false);
		return;
	}

	vector<pc::Rect> negx1;
	negx1 = getsample(img1[0], curt_rect1, 2.0f*srchwinsz1, 1.5f*init_postrainrad1, init_negnumtrain1);
	if (posx1.size() < 1 || negx1.size() < 1){
		assert(false);
		return;
	}

	// 初始的训练
	mil_trainupdate_mc(img0, ii_img0, trparam0.weakclf, trparam0.haars, posx0, negx0, trparam0);
	if (!binotrack_param.pool){
		mil_trainupdate_mc(img1, ii_img1, trparam1.weakclf, trparam1.haars, posx1, negx1, trparam1);
	}
	if (binotrack_param.mixfeat || binotrack_param.pool){
		if (!binotrack_param.pool){
			mil_trainupdate_mc(img1, ii_img1, trparam0.weakclf, trparam0.haars, posx1, negx1, trparam0);
		}
		mil_trainupdate_mc(img0, ii_img0, trparam1.weakclf, trparam1.haars, posx0, negx0, trparam1);
	}
	
	negx0.clear();
	negx1.clear();

	// 清理内存
	for (int i = 0; i < chns; ++i){
		free(ii_img0[i].data);
		free(ii_img1[i].data);
	}
	
}

void miltrack_frame_sync_mc(Mat& frame0, pc::Rect& curt_rect0, TrackParam& trparam0, FeatureParam& ftrparam0,
	Mat& frame1, pc::Rect& curt_rect1, TrackParam& trparam1, FeatureParam& ftrparam1){

	// 检查输入
	if (frame0.cols != frame1.cols || frame1.rows != frame1.rows
		|| frame0.channels() != frame1.channels()){
		abort();
	}

	// 计算图像通道
	const int cols = frame0.cols;
	const int rows = frame0.rows;
	const int chns = frame0.channels();

	vector<Mat> colorChannels0, colorChannels1;
	split(frame0, colorChannels0);
	split(frame1, colorChannels1);

	//图像装入容器
	vector<uimg> img0, img1;
	img0.resize(chns);
	img1.resize(chns);
	for (int i = 0; i < chns; ++i){
		img0[i].chns = 1;
		img0[i].cols = colorChannels0[i].cols;
		img0[i].rows = colorChannels0[i].rows;
		img0[i].data = colorChannels0[i].data;
		img1[i].chns = 1;
		img1[i].cols = colorChannels1[i].cols;
		img1[i].rows = colorChannels1[i].rows;
		img1[i].data = colorChannels1[i].data;
	}

	// 初始化当前窗口
	//pc::Rect curt_rect;
	trparam0.curt_rect.x = curt_rect0.x;
	trparam0.curt_rect.y = curt_rect0.y;
	trparam0.curt_rect.width = curt_rect0.width;
	trparam0.curt_rect.height = curt_rect0.height;
	trparam1.curt_rect.x = curt_rect1.x;
	trparam1.curt_rect.y = curt_rect1.y;
	trparam1.curt_rect.width = curt_rect1.width;
	trparam1.curt_rect.height = curt_rect1.height;

	// 计算积分图
	vector<fimg> ii_img0, ii_img1;
	ii_img0.resize(chns);
	ii_img1.resize(chns);
	for (int i = 0; i < chns; ++i){
		ii_img0[i].data = (float*)calloc(cols * rows, sizeof(float));
		ii_img1[i].data = (float*)calloc(cols * rows, sizeof(float));
		compute_integral(img0[i], ii_img0[i]);
		compute_integral(img1[i], ii_img1[i]);
	}

	// 在search window运行当前分类器，并获取概率图
	vector<pc::Rect> posx0, negx0, detectx0, posx1, negx1, detectx1;
	int srchwinsz0 = trparam0.srchwinsz;//25;
	int srchwinsz1 = trparam1.srchwinsz;//25;
	detectx0 = getsample(img0[0], curt_rect0, srchwinsz0, 0.0f, 1000000);
	detectx1 = getsample(img1[0], curt_rect1, srchwinsz1, 0.0f, 1000000);

	vector<float> prob0, prob1;
	vector<vector<float>> ftrValn0, ftrValn1;
	ftrValn0.clear();
	ftrValn1.clear();
	prob0 = milclassify_mc(trparam0.weakclf, detectx0, ftrValn0, ii_img0, trparam0.uselogR, trparam0);
	if (binotrack_param.pool){
		prob1 = milclassify_mc(trparam0.weakclf, detectx1, ftrValn1, ii_img1, trparam0.uselogR, trparam0);
	} else{
		prob1 = milclassify_mc(trparam1.weakclf, detectx1, ftrValn1, ii_img1, trparam1.uselogR, trparam1);
	}

	// 显示真实的概率图
	if (binotrack_param.showprob){
		float pmin0, pmax0, pmin1, pmax1, pmin, pmax;
		vecminmax(prob0, pmin0, pmax0);
		vecminmax(prob1, pmin1, pmax1);
		pmin = ((pmin0 < pmin1) ? pmin0 : pmin1);
		pmax = ((pmax0 > pmax1) ? pmax0 : pmax1);

		Mat pimg0 = convert2img(prob0, detectx0, rows, cols, pmin0, pmax0);
		Mat pimg1 = convert2img(prob1, detectx1, rows, cols, pmin1, pmax1);
		resize(pimg0, pimg0, Size(640, 480));
		resize(pimg1, pimg1, Size(640, 480));
		imshow("prob0", pimg0);
		imshow("prob1", pimg1);
	}

	// 寻找最优位置
	int bestidx0 = vecmaxindex(prob0);
	double resp0 = prob0[bestidx0];
	curt_rect0.x = detectx0[bestidx0].x;
	curt_rect0.y = detectx0[bestidx0].y;

	int bestidx1 = vecmaxindex(prob1);
	double resp1 = prob1[bestidx1];
	curt_rect1.x = detectx1[bestidx1].x;
	curt_rect1.y = detectx1[bestidx1].y;

	if (binotrack_param.sync){
		if (resp0 > resp1){
			curt_rect1.y = curt_rect0.y;
		}
		else{
			curt_rect0.y = curt_rect1.y;
		}
	}

	//vecmaxindex_sync(curt_rect0, curt_rect1, prob0, prob1, detectx0, detectx1);

	// 训练局部分类器
	if (trparam0.negsamplestrat == 0){
		negx0 = getsamplenum(img0[0], trparam0.negnumtrain, curt_rect0.width, curt_rect0.height);
	} else{
		negx0 = getsample(img0[0], curt_rect0, 1.5f*trparam0.srchwinsz,
			trparam0.posradtrain + 5, trparam0.negnumtrain);
	}
	if (trparam1.negsamplestrat == 0){
		negx1 = getsamplenum(img1[0], trparam1.negnumtrain, curt_rect1.width, curt_rect1.height);
	} else{
		negx1 = getsample(img1[0], curt_rect1, 1.5f*trparam1.srchwinsz,
			trparam1.posradtrain + 5, trparam1.negnumtrain);
	}

	if (trparam0.posradtrain == 1){
		posx0.push_back(curt_rect0);
	} else{
		posx0 = getsample(img0[0], curt_rect0, trparam0.posradtrain, 0, trparam0.posmaxtrain);
	}
	if (trparam1.posradtrain == 1){
		posx1.push_back(curt_rect1);
	} else{
		posx1 = getsample(img1[0], curt_rect1, trparam1.posradtrain, 0, trparam1.posmaxtrain);
	}

	mil_trainupdate_mc(img0, ii_img0, trparam0.weakclf, trparam0.haars, posx0, negx0, trparam0);
	if (!binotrack_param.pool){
		mil_trainupdate_mc(img1, ii_img1, trparam1.weakclf, trparam1.haars, posx1, negx1, trparam1);
	}
	if (binotrack_param.mixfeat || binotrack_param.pool){
		if (!binotrack_param.pool){
			mil_trainupdate_mc(img1, ii_img1, trparam0.weakclf, trparam0.haars, posx1, negx1, trparam0);
		}
		mil_trainupdate_mc(img0, ii_img0, trparam1.weakclf, trparam1.haars, posx0, negx0, trparam1);
	}

	trparam0.count++;
	trparam1.count++;

	// 保存当前窗口到全局
	trparam0.curt_rect.x = curt_rect0.x;
	trparam0.curt_rect.y = curt_rect0.y;
	trparam0.curt_rect.width = curt_rect0.width;
	trparam0.curt_rect.height = curt_rect0.height;
	trparam1.curt_rect.x = curt_rect1.x;
	trparam1.curt_rect.y = curt_rect1.y;
	trparam1.curt_rect.width = curt_rect1.width;
	trparam1.curt_rect.height = curt_rect1.height;

	// 清理内存
	posx0.clear(); negx0.clear(); detectx0.clear();
	posx1.clear(); negx1.clear(); detectx1.clear();
	for (int i = 0; i < chns; ++i){
		free(ii_img0[i].data);
		free(ii_img1[i].data);
	}

}



}

