/*
 *	disp.cpp
 *		for VisualStudio2012 & opencv3.0.0
 *
 *	Created on:	Aug 2,	2015
 *		Author:	Peng Chao
 *
 */

#include <stdio.h>
#include <vector>
#include <stack>
#include <string.h>
#include <stdint.h>
#include <Windows.h>

#include <opencv2\opencv.hpp>

#include "disp.h"

using namespace std;
using namespace cv;

namespace pc{


LARGE_INTEGER m_liPerfFreq;
LARGE_INTEGER m_liPerfStart;
LARGE_INTEGER liPerfNow;
double dfTim;

void __StartWatchTimer()
{
	QueryPerformanceFrequency(&m_liPerfFreq);
	QueryPerformanceCounter(&m_liPerfStart);
}

void __ReadWatchTimer()
{
	QueryPerformanceCounter(&liPerfNow);
	dfTim = (((liPerfNow.QuadPart - m_liPerfStart.QuadPart) * 1000.0f) / m_liPerfFreq.QuadPart);//µ¥Î»Îªms
	printf("%f ms\n",dfTim);
}

float h_thresh = 0;
void DispResult(Mat& src, vector<DetectResult>& result)
{
	char str[20];
	
	for(uint32_t i=0; i < result.size(); i++)
	{
		if (result[i].hs < h_thresh){
			continue;
		}

		Rect rect;
		rect.x = result[i].cs;
		rect.y = result[i].rs;
		rect.width = result[i].modelWd;
		rect.height = result[i].modelHt;
		rectangle(src, rect, Scalar(255,255,255), 2);

		Point focus;
		focus.x = rect.x + rect.width / 2;
		focus.y = rect.y + rect.height / 2;
		sprintf(str, "%8.3f\n", result[i].hs);
		putText(src, str, focus, FONT_HERSHEY_COMPLEX, 0.7, Scalar(255,255,255), 2);
	}
}

}