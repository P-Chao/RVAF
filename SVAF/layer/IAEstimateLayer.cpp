#include "IAEstimateLayer.h"

namespace svaf{

IAEstimateLayer::IAEstimateLayer(LayerParameter& layer) : StereoLayer(layer)
{
	
}

IAEstimateLayer::~IAEstimateLayer()
{
}

bool IAEstimateLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	return true;
}

}
