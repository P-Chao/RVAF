/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"

namespace svaf{

class SurfDescriptorLayer :
	public Layer
{
public:
	explicit SurfDescriptorLayer(LayerParameter& layer);
	~SurfDescriptorLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
	void SetParam(LayerParameter&);
};

}

