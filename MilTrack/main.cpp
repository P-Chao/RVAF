/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1, and see useage before run the program to track
// the target by MIL or OBA method. The code referenced Boris's paper and code
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#include <opencv2\opencv.hpp>
#include "common.h"
using namespace cv;
using namespace pc;

namespace pc{

void Usage(){
	printf("BOOST-TRACK USAGE:\n");
	printf("  track.exe help\n");
	printf("      Look Up the Usage.\n");
	printf("  track.exe -trackmethod\n");
	printf("      Capture Video From Default Camera and Use Default Alg-param\n");
	printf("      -trackmethod=adatrack use AdaBoost Method to train the classifier.\n");
	printf("      -trackmethod=miltrack use MilBoost Method to train the classifier.\n");
	printf("  track.exe -trackmethod -videoname\n");
	printf("      Capture Video From File.\n");
	printf("      -videoname=filename, videofile can be .avi/.mp4/.f4v/.mkv/... formate.\n");
	printf("  track.exe -trackmethod -videoname -seachrad\n");
	printf("      -seachrad, seachrad must be a number, it's a parameter indicate searchsize\n\n");
	printf("    when detected, you can press 'r' to rechoose detect window, and you can also press 'q' to exit the program. Good Luck :)\n");
}

void miltrack_frame		(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void miltrack_firstframe(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void adatrack_frame		(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
void adatrack_firstframe(Mat&, pc::Rect&, TrackParam&, FeatureParam&);
typedef  void(*TrackFun)(Mat&, pc::Rect&, TrackParam&, FeatureParam&);

extern Mat g_org, g_img;
void on_mouse(int, int, int, int, void *);

}

int main(int argc, char* argv[]){
	long totalFrameCount = -1, frameCount = 0;
	TrackFun track_firstframe, track_frame;
	TrackParam trparam;
	FeatureParam ftrparam;
	VideoCapture cap;
	printf("Copyright 2016, Peng Chao(c). All rights reserved.\n\n");
	if (argc > 1){
		if (strcmp(argv[1], "adatrack") == 0){
			track_firstframe = adatrack_firstframe;
			track_frame = adatrack_frame;
			trparam.posradtrain = 1.0f;
		} else if(strcmp(argv[1], "miltrack") == 0){
			track_firstframe = miltrack_firstframe;
			track_frame = miltrack_frame;
			trparam.posradtrain = 4.0f;
		} else{
			Usage();
			exit(0);
		}
		if (argc > 2){
			cap.open(argv[2]);
			totalFrameCount = cap.get(CV_CAP_PROP_FRAME_COUNT);
		} else{
			cap.open(0);
		}
		if (argc > 3){
			trparam.posradtrain == atof(argv[3]);
		}
	} else{
		Usage();
		exit(0);
	}
	
	if (!cap.isOpened())
		return -1;
	Mat		frame, gray;
	Size	sz(240, 180);
	Scalar	color(255, 255, 255);

	cap >> frame;
	cvtColor(frame, gray, CV_RGB2GRAY);
	resize(gray, gray, sz);

	pc::Rect r;
	r.x = 180; r.y = 80; r.width = r.height = 40;
	g_org = gray;
	g_org.copyTo(g_img);
	namedWindow("Select Track Target");
	setMouseCallback("Select Track Target", on_mouse, &r);
	imshow("img", g_img);
	waitKey(0);
	destroyWindow("Select Track Target");
	
	miltrack_firstframe(gray, r, trparam, ftrparam);
	
	while (1){
		frameCount++;
		_StartWatchTimer();
		printf("Frame: %d: x:%d y: %d\n", frameCount, r.x, r.y);
		if (frameCount > totalFrameCount-1 && totalFrameCount > 0)
			break;
		if (frame.empty())
			continue;
		
		switch (waitKey(1))
		{
		case 'r':
			g_org = gray;
			g_org.copyTo(g_img);
			namedWindow("img");
			setMouseCallback("img", on_mouse, &r);
			imshow("img", g_img);
			waitKey(0);
			destroyWindow("img");
			break;
		case 'q':
			exit(0);
		default:
			break;
		}

		miltrack_frame(gray, r, trparam, ftrparam);
		cv::Rect rect; 
		rect.x = r.x;
		rect.y = r.y;
		rect.height = r.height;
		rect.width = r.width; 
		rectangle(gray, rect, color);
		imshow("rst", gray);
		
		cap >> frame;
		cvtColor(frame, gray, CV_RGB2GRAY);
		resize(gray, gray, sz);
		_ReadWatchTimer();
	}
	//waitKey();
}
