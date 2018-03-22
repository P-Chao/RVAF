/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include <opencv2\opencv.hpp>
#include "../src/Circuit.h"
//#include "../svaf/svaf.pb.h"

using namespace std;
using namespace cv;

namespace svaf{

class Circuit;

class Timer{
public:
	void StartWatchTimer(){
		QueryPerformanceFrequency(&m_liPerfFreq);
		QueryPerformanceCounter(&m_liPerfStart);
	}

	float ReadWatchTimer(){
		char temp[64];
		QueryPerformanceCounter(&liPerfNow);
		dfTim = (((liPerfNow.QuadPart - m_liPerfStart.QuadPart) * 1000.0f) / m_liPerfFreq.QuadPart);//单位为ms
		return dfTim;
	}

	float ReadWatchTimer(char *cstr){
		return ReadWatchTimer(string(cstr));
	}

	float ReadWatchTimer(string str){
		char temp[64];
		QueryPerformanceCounter(&liPerfNow);
		dfTim = (((liPerfNow.QuadPart - m_liPerfStart.QuadPart) * 1000.0f) / m_liPerfFreq.QuadPart);//单位为ms
		sprintf(temp, ": %3f ms\n", dfTim);
		DLOG(INFO) << str << temp;
		return dfTim;
	}

	operator float(){
		return dfTim;
	}

protected:
	LARGE_INTEGER m_liPerfFreq;
	LARGE_INTEGER m_liPerfStart;
	LARGE_INTEGER liPerfNow;
	double dfTim;
};

class Layer
{
public:
	Layer();
	explicit Layer(LayerParameter&);
	~Layer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*) = 0;
	void RLOG(std::string);
public:
	static Figures<> *figures;
	static size_t *id;
	static SvafApp task_type;
	static bool gui_mode;
	static Circuit *pCir;
	
protected:
	Timer __t;
	bool  __show;
	bool  __save;
	bool  __bout;
	bool  __logi;
	bool  __logt;
	string __name;
	
};

}
