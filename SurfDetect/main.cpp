/* SurfDetect
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1. The program use surf to detect feature points
// and generate feature descriptor. You can also directly sample image from video  
// or camera, a webcamera is necessary if you run the program on camera mode.
// The program can mosaic two image and draw the match point, you can see usage
// and input the parameters by command line. The surf detect part referenced on
// OpenSurt(https://github.com/amarburg/opencv-ffi-ext/tree/master/ext/opensurf) 
// project, and the core of code write by C. The test shows the program is more 
// efficiency than OpenSurf. Enjoy it!
// Created at: 26 Aug. 2016, all rights reserved.*/


#include "surfdisp.h"
#include <opencv.hpp>

using namespace std;
using namespace cv;
using namespace pc;

namespace pc{


typedef void(*DetectFun)(Mat&);
typedef Mat(*MosaicFun)(Mat&, Mat&);
void siftImage(Mat&){}
void surfImage(Mat&);
Mat surfMatch(Mat&, Mat&);
Mat surfMosaic(Mat&, Mat&);

extern AlgParam algparam;
extern SurfParam suparam;

static void Usage(){
	printf("MOSAIC USAGE: surfDetect\n");
	printf("  [-I imagename] <detect the surf features of a image>\n");
	printf("  [-H leftimage rightimage] <find the match point of two image>\n");
	printf("  [-M leftimage rightimage] <mosaic the two image based on match>\n");
	printf("  [-U leftimage rightimage] <using stereo triangulation in matlab>");
	printf("  [-V videoname] <open the video and detect surf features>\n");
	printf("  [-C [cameraid]] <opne the camera and detect surf features, id default by 0>\n");
	printf("optional parameter:\n");
	printf("  [-i itervals] <the count of itervals, range from 1 to 4, default value is 4>\n");
	printf("  [-o octaves] <the count of octaves, range from 1 to 4, default value is 4>\n");
	printf("  [-s stride] <the count of stride, range from 1 to 6, default value is 2>\n");
	printf("  [-h threashold] <the value of ransac threshold, default value is 5>\n");
	printf("  [-t threashold] <the value of detect threashold, default value is 0.004>\n");
	printf("  [-u] <upright, compute surf descriptor without rotation>\n");
	printf("  [-d] <debug mode, show more info and result when process>\n");
	printf("  [-S] <save image in dir './imgs/*.jpg'>\n\n");
	printf("    when run code with video or camera, can press 'q' to quit and press 'p' to pause, when paused, you can press anykey to continue.\n\n");
}

static bool getopt(const int argc, const char* const argv[]){
	for (int i = 1; i < argc; ++i){
		if (strcmp(argv[i], "help") == 0)
			return false;
		if (argv[i][0] != '-')
			continue;

		switch (argv[i][1])
		{
		case 'U':
			algparam.runtype = 'U';
			if (i + 2 < argc){
				algparam.leftimage = argv[++i];
				algparam.rightimage = argv[++i];
			}
			printf("  leftimage:\t%s\n", algparam.leftimage);
			printf("  rightimage:\t%s\n", algparam.rightimage);
			break;
		case 'M':
			algparam.runtype = 'M';
			if (i + 2 < argc){
				algparam.leftimage = argv[++i];
				algparam.rightimage = argv[++i];
			}
			printf("  leftimage:\t%s\n", algparam.leftimage);
			printf("  rightimage:\t%s\n", algparam.rightimage);
			break;
		case 'H':
			algparam.runtype = 'H';
			if (i + 2 < argc){
				algparam.leftimage = argv[++i];
				algparam.rightimage = argv[++i];
			}
			printf("  leftimage:\t%s\n", algparam.leftimage);
			printf("  rightimage:\t%s\n", algparam.rightimage);
			break;
		case 'I':
			algparam.runtype = 'I';
			if (i + 1 < argc){
				algparam.leftimage = argv[++i];
			}
			printf("  image:\t%s\n", algparam.leftimage);
			break;
		case 'V':
			algparam.runtype = 'V';
			if (i + 1 < argc){
				algparam.videoname = argv[++i];
			}
			printf("  video:\t%s\n", algparam.videoname);
			break;
		case 'C':
			algparam.runtype = 'C';
			if (i + 1 < argc){
				if (argv[i + 1][0] != '-'){
					algparam.camnum = atoi(argv[++i]);
				}
			}
			printf("  camera:\t%d\n", algparam.camnum);
			break;
		case 'S':
			algparam.saveimage = true;
			break;
		case 'd':
			algparam.showprocess = true;
			break;
		case 'i':
			if (i + 1 < argc){
				suparam.intervals = atoi(argv[++i]);
			}
			break;
		case 'o':
			if (i + 1 < argc){
				suparam.octaves = atoi(argv[++i]);
			}
			break;
		case 's':
			if (i + 1 < argc){
				suparam.stride = atoi(argv[++i]);
			}
			break;
		case 'h':
			if (i + 1 < argc){
				suparam.homothresh = atoi(argv[++i]);
			}
			break;
		case 't':
			if (i + 1 < argc){
				suparam.thresh = atof(argv[++i]);
			}
			break;
		case 'u':
			if (i + 1 < argc){
				suparam.upright = true;
			}
			break;
		default:
			return false;
		}
	}
	printf("  upright:\t%d\n", suparam.upright);
	printf("  stride:\t%d\n", suparam.stride);
	printf("  octave:\t%d\n", suparam.octaves);
	printf("  intervals:\t%d\n", suparam.intervals);
	printf("  ransac:\t%d\n", suparam.homothresh);
	printf("  thresh: \t%f\n", suparam.thresh);
	printf("  debuginfo:\t%d\n\n", algparam.showprocess);
	return true;
}


}

int main(int argc, char* argv[]){
	long totalFrameCount = -1, frameCount = 0;
	DetectFun		pAlg = surfImage;
	MosaicFun		pMos = surfMosaic;
	Mat				frame, image1, image2;
	VideoCapture	cap;

	printf("Copyright 2016, Peng Chao(c). All rights reserved.\n\n");
	if (argc == 1 || !getopt(argc, argv)){
		Usage();
		return 0;
	}

	switch (algparam.runtype)
	{
	case 'V':
		cap.open(algparam.videoname);
		totalFrameCount = cap.get(CV_CAP_PROP_FRAME_COUNT);
		break;
	case 'C':
		cap.open(algparam.camnum);
		break;
	case 'I':
		frame = imread(algparam.leftimage);
		StartWatchTimer_();
		pAlg(frame);
		ReadWatchTimer_();
		imshow(argv[0], frame);
		imsave("SurfImage", frame);
		waitKey(0);
		return 0;
	case 'U':
	case 'H':
		pMos = surfMatch;
		image1 = imread(algparam.leftimage);
		image2 = imread(algparam.rightimage);
		StartWatchTimer_();
		frame = pMos(image1, image2);
		ReadWatchTimer_();
		imshow(argv[0], frame);
		imsave("Homography", frame);
		waitKey(0);
		return 0;
	case 'M':
		pMos = surfMosaic;
		image1 = imread(algparam.leftimage);
		image2 = imread(algparam.rightimage);
		StartWatchTimer_();
		frame = pMos(image1, image2);
		ReadWatchTimer_();
		imshow(argv[0], frame);
		imsave("Mosaic", frame);
		waitKey(0);
		return 0;
	default:
		return -1;
		break;
	}
	
	if (!cap.isOpened())
		return -1;
	while (1){
		cap >> frame;
		frameCount++;
		if (frameCount > totalFrameCount - 1 && totalFrameCount > 0)
			break;
		if (frame.empty())
			continue;
		switch (waitKey(1))
		{
		case 'q':
			exit(0);
		case 'p':
			waitKey(0);
		default:
			break;
		}
		
		StartWatchTimer_();
		pAlg(frame);
		ReadWatchTimer_();
		imshow(argv[0], frame);
	}

	return 0;
}
