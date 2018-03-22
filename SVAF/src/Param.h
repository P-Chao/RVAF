/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once

#include <opencv2\opencv.hpp>
#include <glog\logging.h>
#include <hash_map>
#include "..\svaf\svaf.pb.h"

#include "..\..\CameraLibrary\VideoCapture.h"

using namespace std;
using namespace cv;

namespace svaf{


class Param
{
public:
	Param(SvafTask&);
	~Param();

	int		Size();
	bool	IsBinocular() const;
	bool	IsLayerExit(string);

	string					getImage();
	string					getVideo();
	pair<string, string>	getImagePair();
	pair<string, string>	getVideoPair();
	int						getImageCount();
	int						getVideoCount();
	unsigned int			getCurrentFrameCount();
	LayerParameter_LayerType getDataSource() const;

	Param& operator>>(Mat&);
	Param& operator>>(pair<Mat, Mat>&);
	LayerParameter& operator[](int&);
	LayerParameter& operator[](string&);
	
	
protected:
	void	InitDataSource();
	void	InitVideoState();

	cv::VideoCapture	cap_[2];

private:
	int		index_;
	int		frame_;
	int		videoframecount_;

	bool							isbinocular_;
	LayerParameter_LayerType		runtype_;
public:
	vector<pair<string, string>>	imagepairs_;
	vector<string>					images_;
private:
	vector<pair<string, string>>	videopairs_;
	vector<string>					videos_;
	int								camera_[2];
	hash_map<string, LayerParameter> namelayers_;
	hash_map<int, LayerParameter>	layers_;
	
	pc::VideoCapture				dspcamera;
};

}

