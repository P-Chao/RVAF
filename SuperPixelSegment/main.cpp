// SuperPixelSeg.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <opencv.hpp>
#include <opencv.hpp>
#include <Kinect.h>
using namespace cv;

//#pragma comment(lib, "kinect20.lib")

void TestImageMatchEadp(int argc, char** argv);
void TestImageMatchSgm(int argc, char** argv);
void TestImageSupixSeg(int argc, char** argv);
void SuperPixelSegment(IplImage* pIamge);

int main(int argc, char* argv[]){
	//SuperPixelSegment();

#ifndef KINECT
	TestImageMatchEadp(argc, argv);
	TestImageMatchSgm(argc, argv);
	/*IplImage *im = cvLoadImage("0.bmp",CV_LOAD_IMAGE_UNCHANGED);
	SuperPixelSegment(im);*/
#else
	HRESULT hResult = S_OK;
	IKinectSensor *kinect;
	GetDefaultKinectSensor(&kinect);
	kinect->Open();
	IColorFrameSource *colorsource;
	IColorFrameReader *colorreader;
	IFrameDescription *colorde;
	kinect->get_ColorFrameSource(&colorsource);
	colorsource->OpenReader(&colorreader);
	colorsource->get_FrameDescription(&colorde);

	int width = 0;
	int height = 0;
	colorde->get_Height(&height);
	colorde->get_Width(&width);

	Mat a(height, width, CV_8UC4);
	//namedWindow("Capture Image From Kinect2.0");
	while (1){
		IColorFrame *frame;
		hResult = colorreader->AcquireLatestFrame(&frame);
		if (SUCCEEDED(hResult)){
			frame->CopyConvertedFrameDataToArray(height*width * 4,
				reinterpret_cast<BYTE*>(a.data), ColorImageFormat::ColorImageFormat_Bgra);
		}
		if (frame != NULL){
			frame->Release();
			frame = NULL;
		}
		if (waitKey(1) == VK_ESCAPE)
			break;
		//imshow("aaa", a);
		Mat aa;
		cv::resize(a,aa,Size(920,540));
		IplImage ipl_img = aa;
		
		IplImage *input = cvCloneImage(&ipl_img);
		IplImage *output = cvCreateImage(cvGetSize(input), input->depth, 3);
		cvCvtColor(input, output, CV_RGBA2RGB);
		//cvShowImage("aaa", input);
		//cvWaitKey(0);
		SuperPixelSegment(input);
		//cvReleaseImage(&input);
		cvReleaseImage(&output);
		LOG(INFO) << "Processed a frame!";
	}
	if (colorsource != NULL){
		colorsource->Release();
		colorsource = NULL;
	}
	if (colorreader != NULL){
		colorreader->Release();
		colorreader = NULL;
	}
	if (colorde != NULL){
		colorde->Release();
		colorde = NULL;
	}
	if (kinect){
		kinect->Close();
	}
	if (kinect != NULL){
		kinect->Release();
		kinect = NULL;
	}

#endif
	destroyAllWindows();
	system("pause");
	return 0;
}

