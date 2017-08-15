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
