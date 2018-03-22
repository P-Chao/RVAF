/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/


#pragma once
#include "Layer.h"

namespace svaf{

class SurfPointLayer :
	public Layer
{
public:
	explicit SurfPointLayer(LayerParameter& layer);
	~SurfPointLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
	void SetParam(LayerParameter&);
};

}

