/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"

namespace svaf{

class RansacLayer :
	public Layer
{
public:
	explicit RansacLayer(LayerParameter& layer);
	~RansacLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
};

}


