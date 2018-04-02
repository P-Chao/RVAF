/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "StereoLayer.h"

namespace svaf{

class NDTEstimateLayer :
	public StereoLayer{
public:
	explicit NDTEstimateLayer(LayerParameter& layer);
	~NDTEstimateLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
protected:


private:
	World *pWorld_;

};

}

