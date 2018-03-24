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

// 计时器类
class Timer{
public:
	// 打开秒表
	void StartWatchTimer(){
		QueryPerformanceFrequency(&m_liPerfFreq);
		QueryPerformanceCounter(&m_liPerfStart);
	}

	// 读取秒表
	float ReadWatchTimer(){
		char temp[64];
		QueryPerformanceCounter(&liPerfNow);
		dfTim = (((liPerfNow.QuadPart - m_liPerfStart.QuadPart) * 1000.0f) / m_liPerfFreq.QuadPart);//单位为ms
		return dfTim;
	}

	// 读取秒表并输出信息到屏幕
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

	// 可以直接隐式转换该类记录的时间
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
	// 静态成员变量，参见cpp文件
	static Figures<> *figures;
	static size_t *id;
	static SvafApp task_type;
	static bool gui_mode;
	static Circuit *pCir;
	
protected:
	Timer __t;		// 记录的时间
	bool  __show;	// 是否显示结果
	bool  __save;	// 是否保存结果
	bool  __bout;	// 是否输出到进程
	bool  __logi;	// 是否记录信息
	bool  __logt;	// 是否记录时间
	string __name;	// 层的名字
	
};

}
