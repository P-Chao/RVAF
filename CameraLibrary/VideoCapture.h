#pragma once
#include <Windows.h>
#include <iostream>
#include <vector>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

namespace pc{

class VideoCapture
{
	typedef void* HANDLE;
	static const char HEAD_SIZE = 16;
public:
	VideoCapture();
	VideoCapture(bool);
	~VideoCapture();

	bool getImage();
	bool open();

public:
	//pc::VideoCapture& operator>>(Mat&);
	//pc::VideoCapture& operator>>(pair<Mat&, Mat&>&);
	std::vector<cv::Mat>	images_;

private:
	HANDLE			fileMapping_;
	HANDLE			mutex_;
	LPTSTR			pMsg_;
	char			*fileName_;
	unsigned int	buffersize_;
	unsigned char	*buffer_;
	

};

}



