/*
 *	main.cpp
 *		for VisualStudio2012 & opencv3.0.0
 *
 *	Created on:	Aug 2,	2015
 *		Author:	Peng Chao
 *
 */

#include <iostream>
#include <string>
#include <stdio.h>
#include <errno.h>
#include <Windows.h>
#include <omp.h>

#include <opencv2\opencv.hpp>

#include "acfDetect.h"
#include "chnsCompute.h"
#include "disp.h"

#define RUN_TYPE		1		//0:image
								//1:camera
								//2:
								//3:databaseTest
								//4:video
#define RUN_CHASSIS		0		//0:not connect to other device

using namespace std;
using namespace cv;
using namespace pc;

const string DataBaseDir = ".";

bool cameraOn = false;
bool procExit = false;
bool detectExit = false;

HANDLE	camera_sem;					//同步摄像头线程
const int width  = 640;				//摄像头图像宽度
const int height = 480;				//摄像头图像高度
int		device	 = 0;				//摄像头设备地址
vector<DetectResult> gResult;		//检测结果
uint8_t *rawData = NULL;
Mat		camFrame;					//摄像头采集得到的图像


void ImageDetect(char* detectorfile, char* imagefile)
{
	Channels chns;
	string ImgDir;
	ImgDir = DataBaseDir + "/a.jpg";


	Mat img = imread(imagefile);


	if(img.empty())
	{
		fprintf(stderr, "Can not load image %s\n", imagefile);
		exit(1);
	}

	PyramidOpt	opt;
	//Pyramid		pyramid;
	opt.minDs	= Size(60, 60);
	opt.nPerOct = 8;
	opt.nApprox = opt.nPerOct - 1;
	opt.nOctUp	= 0;
	opt.smooth	= 1;
	opt.pad		= Size(4, 4);
	opt.lambdas.push_back(0);
	opt.lambdas.push_back(0.2259f);
	opt.lambdas.push_back(0.2259f);
	opt.chnsOpt.shrink			= 4;
	opt.chnsOpt.colorSmooth		= 1;
	opt.chnsOpt.gradMagNormRad	= 5;
	opt.chnsOpt.gradMagNormConst = 0.005f;
	opt.chnsOpt.gradMagFull		= 0;
	opt.chnsOpt.gradHistBinSize = opt.chnsOpt.shrink;
	opt.chnsOpt.gradHistOrients = 6;
	opt.chnsOpt.gradHistSoftBin = 0;
	opt.chnsOpt.gradHistUseHog	= 0;
	opt.chnsOpt.gradHistClipHog = 0.2f;


	AcfDetector detector((char*)"facedetector.dat");
	

	vector<DetectResult> result;

	for(int i=0;i<10;i++)
	{
	__StartWatchTimer();
	AcfDetectImg(img, opt, detector, result);
	__ReadWatchTimer();
	}

	DispResult(img, result);
	imshow("result", img);

	while(waitKey(0) != 113);
}

void VideoDetect(char *detectorfile, char *videofile, bool sample)
{
	//Initialize detector
	PyramidOpt opt;
	Pyramid pyramid;
	opt.minDs = Size(60, 60);
	opt.nPerOct = 8;
	opt.nApprox = opt.nPerOct - 1;
	opt.nOctUp = 0;
	opt.smooth = 1;
	opt.pad = Size(40, 40);
	opt.lambdas.push_back(0);
	opt.lambdas.push_back(0.2259f);
	opt.lambdas.push_back(0.2259f);
	opt.chnsOpt.shrink = 4;
	opt.chnsOpt.colorSmooth = 1;
	opt.chnsOpt.gradMagNormRad = 5;
	opt.chnsOpt.gradMagNormConst = 0.005f;
	opt.chnsOpt.gradMagFull = 0;
	opt.chnsOpt.gradHistBinSize = opt.chnsOpt.shrink;
	opt.chnsOpt.gradHistOrients = 6;
	opt.chnsOpt.gradHistSoftBin = 0;
	opt.chnsOpt.gradHistUseHog = 0;
	opt.chnsOpt.gradHistClipHog = 0.2f;

	AcfDetector detector(detectorfile);
	vector<DetectResult> result;

	int i = 10000;
	Mat frame;
	Size sz(640,360);
	VideoCapture cap(videofile);
	char name[100];
	while (1){
		i++;
		cap >> frame;
		if (!frame.data){
			break;
		}
		resize(frame, frame, sz);
		StartWatchTimer();
		AcfDetectImg(frame, opt, detector, result);
		ReadWatchTimer();
		DispResult(frame, result);
		imshow("result", frame);
		if (!sample){
			switch (waitKey(1)){
			case 'q':
				return;
			}
		} else
		switch (waitKey(20))
		{
		case 'q':
			return;
		case 'p':
			sprintf(name, "./pos/I%d.jpg", i);
			imwrite(name, frame);
			break;
		case 'n':
			sprintf(name, "./neg/N%d.jpg", i);
			imwrite(name, frame);
			break;
		default:
			break;
		}
		
	}

	return;
}

void CameraDetect(char *detectorfile, int camindex, bool sample)
{
	//Initialize detector
	PyramidOpt opt;
	Pyramid pyramid;
	opt.minDs = Size(60, 60);
	opt.nPerOct = 8;
	opt.nApprox = opt.nPerOct - 1;
	opt.nOctUp = 0;
	opt.smooth = 1;
	opt.pad = Size(40, 40);
	opt.lambdas.push_back(0);
	opt.lambdas.push_back(0.2259f);
	opt.lambdas.push_back(0.2259f);
	opt.chnsOpt.shrink = 4;
	opt.chnsOpt.colorSmooth = 1;
	opt.chnsOpt.gradMagNormRad = 5;
	opt.chnsOpt.gradMagNormConst = 0.005f;
	opt.chnsOpt.gradMagFull = 0;
	opt.chnsOpt.gradHistBinSize = opt.chnsOpt.shrink;
	opt.chnsOpt.gradHistOrients = 6;
	opt.chnsOpt.gradHistSoftBin = 0;
	opt.chnsOpt.gradHistUseHog = 0;
	opt.chnsOpt.gradHistClipHog = 0.2f;

	AcfDetector detector(detectorfile);
	vector<DetectResult> result;

	int i = 10000;
	Mat frame;
	Size sz(640, 360);
	VideoCapture cap(camindex);
	if (!cap.isOpened()){
		printf("%d camera is not open\n", camindex);
		return;
	}
	
	char name[100];
	while (1){
		i++;
		cap >> frame;
		if (!frame.data){
			break;
		}
		resize(frame, frame, sz);
		StartWatchTimer();
		AcfDetectImg(frame, opt, detector, result);
		ReadWatchTimer();
		DispResult(frame, result);
		imshow("result", frame);
		if (!sample){
			switch (waitKey(1)){
			case 'q':
				return;
			}
		}
		else
			switch (waitKey(20))
		{
			case 'q':
				return;
			case 'p':
				sprintf(name, "./pos/I%d.jpg", i);
				imwrite(name, frame);
				break;
			case 'n':
				sprintf(name, "./neg/N%d.jpg", i);
				imwrite(name, frame);
				break;
			default:
				break;
		}

	}

	return;
}

void PrintUsage(){
	printf("AcfDetect USAGE:\n");
	printf("  acf.exe --imagesource --detector --filename --thresh --sample\n");
	printf("      --imagesource\n");
	printf("          =camera, frap image from camere\n");
	printf("          =video, frap image from video file\n");
	printf("          =image, frap image from image file\n");
	printf("      --detector\n");
	printf("          =detector.dat, choose the detector by filename\n");
	printf("      --filename\n");
	printf("          =0, camera index\n");
	printf("          =*.bmp|*.png...*.mp4|*.avi, image or video file\n");
	printf("      --thresh\n");
	printf("          default=0, show bounding box according to threshold\n");
	printf("      --sample\n");
	printf("          default=false, you can press 'p'|'r' to save positive|negtive image to      ./neg|./pos, the two path should be created by ourself.\n");
	
	printf("    when detected, you can press 'q' to exit the program. Good Luck :)\n");
}

namespace pc{
	extern float h_thresh;
}

int main(int argc, char *argv[])
{
	printf("Copyright 2016, Peng Chao(c). All rights reserved.\n\n");
	ChnsComputeInit();
	if (argc < 3){
		PrintUsage();
	}
	else if (argc == 4){
		h_thresh = 0;
		if (strcmp(argv[1], "camera") == 0){
			CameraDetect(argv[2], atoi(argv[3]), false);
		}
		else if (strcmp(argv[1], "video") == 0){
			VideoDetect(argv[2], argv[3], false);
		}
		else if (strcmp(argv[1], "image") == 0){
			ImageDetect(argv[2], argv[3]);
		}
	}
	else if (argc == 5){
		h_thresh = atof(argv[4]);
		if (strcmp(argv[1], "camera") == 0){
			int t = atoi(argv[3]);
			CameraDetect(argv[2], atoi(argv[3]), false);
		}
		else if (strcmp(argv[1], "video") == 0){
			VideoDetect(argv[2], argv[3], false);
		}
		else if (strcmp(argv[1], "image") == 0){
			ImageDetect(argv[2], argv[3]);
		}
	}
	else if (argc > 5){
		h_thresh = atof(argv[4]);
		if (strcmp(argv[1], "camera") == 0){
			CameraDetect(argv[2], atoi(argv[3]), true);
		}
		else if (strcmp(argv[1], "video") == 0){
			VideoDetect(argv[2], argv[3], true);
		}
		else if (strcmp(argv[1], "image") == 0){
			ImageDetect(argv[2], argv[3]);
		}
	}

	return 0;
}


