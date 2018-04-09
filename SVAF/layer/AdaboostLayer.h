/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"
#include "..\..\AcfDetect\acfDetect.h"

namespace svaf{

class AdaboostLayer :
	public Layer
{
public:
	explicit AdaboostLayer(LayerParameter& layer);
	~AdaboostLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
	bool RunForOneRect(vector<Block>&, LayerParameter&, vector<Rect>&);

protected:
	bool ImageDetect(vector<Block>&, vector<Block>&);
	bool ScaleDetect(vector<Block>&, vector<Block>&);
	bool SelectROI(Block&, int, vector<pc::DetectResult>&, bool isleft = false);
	bool ResultROI(vector<Block>&, vector<Block>&);
	//void ROICenter();

private:
	vector<vector<Rect>> result_rect;
	vector<vector<float>> result_sc;

	pc::AcfDetector detector;
	float thresh;
	int scaleindex;
	bool ksvideo;
	bool ksframe;
	bool kseline;
	float epoLine;
	float nms;

	pc::PyramidOpt	opt;
	bool video_firstframe;
	cv::Rect	roipad;
	int		xshift;
};

}
