/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#include "ICPEstimateLayer.h"

namespace svaf{

ICPEstimateLayer::ICPEstimateLayer(LayerParameter& layer) : StereoLayer(layer)
{

}

ICPEstimateLayer::~ICPEstimateLayer()
{
}

bool ICPEstimateLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	return true;
}

}
