/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "StereoLayer.h"

namespace svaf{

class ICPEstimateLayer :
	public StereoLayer{
public:
	explicit ICPEstimateLayer(LayerParameter& layer);
	~ICPEstimateLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
protected:


private:
	World *pWorld_;

};

}

