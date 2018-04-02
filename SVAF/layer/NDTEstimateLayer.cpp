/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
NDTµãÔÆÅä×¼
*/

#include "NDTEstimateLayer.h"

namespace svaf{

NDTEstimateLayer::NDTEstimateLayer(LayerParameter& layer) : StereoLayer(layer)
{

}

NDTEstimateLayer::~NDTEstimateLayer()
{
}

bool NDTEstimateLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	return true;
}

}
