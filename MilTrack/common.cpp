/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1, and see useage before run the program to track
// the target by MIL or OBA method. The code referenced Boris's paper and code
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#include "common.h"
#include <stdio.h>

//using namespace cv;

namespace pc{


static CvRNG rng_state = cvRNG(1);

int		randint(const int min, const int max){
	return cvRandInt(&rng_state) % (max - min + 1) + min;
}

float	randfloat(){
	return (float)cvRandReal(&rng_state);
}

#define _WINDOWS
#ifdef _WINDOWS
#include <Windows.h>
static LARGE_INTEGER _m_liPerfFreq;
static LARGE_INTEGER _m_liPerfStart;
static LARGE_INTEGER _liPerfNow;
static double _dfTim;

void _StartWatchTimer(){
	QueryPerformanceFrequency(&_m_liPerfFreq);
	QueryPerformanceCounter(&_m_liPerfStart);
}

void _ReadWatchTimer(){
	QueryPerformanceCounter(&_liPerfNow);
	_dfTim = (((_liPerfNow.QuadPart - _m_liPerfStart.QuadPart) * 1000.0f) / _m_liPerfFreq.QuadPart);
	printf("%f ms\n", _dfTim);
}

#else
#include <stack>
#include <sys\time.h>
stack<timeval> ts;
void StartWatchTimer(){
	timeval t;
	gettimeofday(&t, NULL);
	ts.push(t);
}

void ReadWatchTimer(){
	timeval t1, t2;
	float elapsed;
	gettimeofday(&t2, NULL);
	if (!ts.empty())
	{
		t1 = ts.top();
		ts.pop();
		elapsed = (t2.tv_sec - t1.tv_sec) + (t2.tv_usec - t1.tv_usec) / 1000000.f;
		printf("%fs\n", elapsed);
	}
}
#endif

vector<float> vecsqr(vector<float> v){
	for (int i = 0; i < v.size(); i++){
		v[i] = v[i] * v[i];
	}
	return v;
}

vector<float> vecsub(vector<float> v, const float a){
	for (int i = 0; i < v.size(); i++){
		v[i] -= a;
	}
	return v;
}

double vecvar(vector<float> v){
	double vmean = vecmean(v);
	double var = 0;
	for (int i = 0; i < v.size(); i++){
		var += (v[i] - vmean) * (v[i] - vmean);
	}
	return var / v.size();
}

double vecmean(vector<float>& v){
	double avg = 0;
	for (auto value : v)
		avg += value;
	return avg / v.size();
}

double sigmoid(double x){
	return 1.0f / (1.0f + exp(-x));
}

int vecmaxindex(vector<float>& v){
	float max = v[0];
	int maxidx = 0;
	for (int i = 1; i < v.size(); i++){
		if (v[i] > max){
			max = v[i];
			maxidx = i;
		}
	}
	return maxidx;
}

void vecminmax(vector<float>& v, float& vmin, float& vmax){
	const int vsize = v.size();
	if (vsize == 0)
		return;

	vmin = v[0];
	vmax = v[0];

	//int s = (vsize % 2) ? 1 : 0;
	for (int i = (vsize & 1); i < vsize; i+=2){
		if (v[i] > v[i + 1]){
			if (v[i] > vmax)
				vmax = v[i];
			if (v[i + 1] < vmin)
				vmin = v[i + 1];
		}else{
			if (v[i+1] > vmax)
				vmax = v[i+1];
			if (v[i] < vmin)
				vmin = v[i];
		}
	}
		
	return;
}

void vecmaxindex_sync(pc::Rect& curt_rect0, pc::Rect& curt_rect1,
	vector<float>& p0, vector<float>& p1, vector<pc::Rect>& dx0, vector<pc::Rect>& dx1){
	map<int, int> hash0, hash1;
	const int size0 = dx0.size();
	const int size1 = dx1.size();
	for (int i = 0; i < size0; ++i){
		if (p0[i] > hash0[dx0[i].y]){
			hash0[dx0[i].y] = i;
		}
	}
	for (int i = 0; i < size1; ++i){
		if (p1[i] > hash1[dx1[i].y]){
			hash1[dx1[i].y] = i;
		}
	}

	float max = FLT_MIN;
	int maxidx0 = 0, maxidx1 = 0;
	auto it0 = hash0.begin();
	auto it1 = hash1.begin();
	const int mapsize0 = hash0.size();
	for (int i = 0; i < mapsize0; ++i){
		float sc = p0[it0->second] + p1[it1->second];
		if (sc > max){
			max = sc;
			maxidx0 = it0->second;
			maxidx1 = it1->second;
		}
		it0++;
		it1++;
	}

	curt_rect0.x = dx0[maxidx0].x;
	curt_rect0.y = dx0[maxidx0].y;
	curt_rect1.x = dx1[maxidx1].x;
	curt_rect1.y = dx1[maxidx1].y;
	
	if (curt_rect0.y != curt_rect1.y){
		//abort();
	}

	return;
}

cv::Mat convert2img(vector<float>& v, vector<pc::Rect>& idx, int rows, int cols, float min, float max){
	cv::Mat mat(rows, cols, CV_8UC1, cv::Scalar::all(0));
	const int vsize = v.size();
	const float df = 255.0 / (max - min);
	for (int i = 0; i < vsize; ++i){
		mat.ptr<uchar>(idx[i].y + idx[i].height / 2)[idx[i].x + idx[i].width / 2] 
			= (uchar)((v[i] - min) * df);
	}
	return mat;
}

cv::Mat g_org, g_img;
void on_mouse(int eventt, int x, int y, int flags, void *ustc){
	cv::Mat tmp;
	pc::Rect* r = (pc::Rect*)ustc;
	static cv::Point pre_pt = (-1, -1); 
	static cv::Point cur_pt = (-1, -1); 
	char temp[16];

	if (eventt == CV_EVENT_LBUTTONDOWN){
		g_org.copyTo(g_img);  
		sprintf(temp, "(%d,%d)", x, y);
		pre_pt = cv::Point(x, y);
		cv::putText(g_img, temp, pre_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255), 1, 8);
		cv::circle(g_img, pre_pt, 2, cv::Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		cv::imshow("Select Track Target", g_img);
	} else if (eventt == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON)){
		g_img.copyTo(tmp);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cv::Point(x, y);
		cv::putText(tmp, temp, cur_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255));
		cv::imshow("Select Track Target", tmp);
	} else if (eventt == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON)){
		g_img.copyTo(tmp);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cv::Point(x, y);
		cv::putText(tmp, temp, cur_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255));
		cv::rectangle(tmp, pre_pt, cur_pt, cv::Scalar(0, 255, 0, 0), 1, 8, 0);
		cv::imshow("Select Track Target", tmp);
	} else if (eventt == CV_EVENT_LBUTTONUP) {
		g_org.copyTo(g_img);
		sprintf(temp, "(%d,%d)", x, y);
		cur_pt = cv::Point(x, y);
		cv::putText(g_img, temp, cur_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0, 255));
		cv::circle(g_img, pre_pt, 2, cv::Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		cv::rectangle(g_img, pre_pt, cur_pt, cv::Scalar(0, 255, 0, 0), 1, 8, 0);
		cv::imshow("Select Track Target", g_img);
		g_img.copyTo(tmp);

		int width = abs(pre_pt.x - cur_pt.x);
		int height = abs(pre_pt.y - cur_pt.y);
		if (width == 0 || height == 0)
		{
			printf("width == 0 || height == 0");
			return;
		}
		r->x = min(cur_pt.x, pre_pt.x);
		r->y = min(cur_pt.y, pre_pt.y);
		r->width = width;
		r->height = height;
	}
}

}
