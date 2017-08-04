#include "VideoCapture.h"

#include <stdio.h>
#include <memory>
#include <Windows.h>
#include <WinBase.h>

namespace pc{

VideoCapture::VideoCapture() : 
buffersize_(0),
buffer_(nullptr),
fileMapping_(nullptr)
{
}

VideoCapture::VideoCapture(bool cameraopen) :
buffersize_(0),
buffer_(nullptr),
fileMapping_(nullptr)
{
	fileName_ = new char[30];
	memcpy(fileName_, "BaseImage", 10);
	mutex_ = OpenEvent(MUTEX_ALL_ACCESS, false, "BaseImageMutex");
	fileMapping_ = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, fileName_);
	pMsg_ = (LPTSTR)MapViewOfFile(fileMapping_, FILE_MAP_ALL_ACCESS, 0, 0, 0);
}

VideoCapture::~VideoCapture(){
	if (mutex_ == nullptr || fileMapping_ == nullptr){
		return;
	}
	if (!UnmapViewOfFile(fileMapping_)){}
	CloseHandle(fileMapping_);
	CloseHandle(mutex_);

	delete fileName_;
	delete buffer_;
}

bool VideoCapture::open(){
	fileName_ = new char[30];
	memcpy(fileName_, "BaseImage", 10);
	mutex_ = OpenEvent(MUTEX_ALL_ACCESS, false, "BaseImageMutex");
	fileMapping_ = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, fileName_);
	pMsg_ = (LPTSTR)MapViewOfFile(fileMapping_, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	if (mutex_ == nullptr || fileMapping_ == nullptr){
		return false;
	}
	return true;
}

bool VideoCapture::getImage(){
	if (mutex_ == nullptr || fileMapping_ == nullptr){
		return false;
	}

	WaitForSingleObject(mutex_, INFINITE);

	// 解析4字节头，并保存数据
	unsigned int frameCount = ((int*)pMsg_)[0];
	unsigned int imageLength = ((int*)pMsg_)[4];
	unsigned int frameLength = (imageLength + HEAD_SIZE) * frameCount + HEAD_SIZE;
	if (buffersize_ < frameLength){
		delete buffer_;
		buffer_ = nullptr;
		buffersize_ = 0;
	}

	if (buffer_ == nullptr){
		buffer_ = new unsigned char[frameLength];
		buffersize_ = frameLength;
	}
	memcpy(buffer_, pMsg_, frameLength);

	// 解析数据包
	images_.clear();
	unsigned char *pImage = buffer_ + HEAD_SIZE;
	for (int i = 0; i < frameCount; ++i){
		int imageSize = ((int*)pImage)[0];
		int imageWidth = ((int*)pImage)[1];
		int imageHeight = ((int*)pImage)[2];
		int imageType = ((int*)pImage)[3];

		int type = 8;
		switch (imageType){
		case 8:
			type = CV_8UC1;
			break;
		case 24:
			type = CV_8UC3;
			break;
		default:
			return false;
			break;
		}

		cv::Mat	img(imageHeight, imageWidth, type, pImage + HEAD_SIZE);
		cv::flip(img, img, 0);
		images_.push_back(img);
		pImage = pImage + imageSize + HEAD_SIZE;
	}
	return true;
}

}

