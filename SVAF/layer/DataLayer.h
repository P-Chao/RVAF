/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/


#pragma once
#include "Layer.h"

namespace svaf{

class DataLayer :
	public Layer
{
public:
	explicit DataLayer(LayerParameter& layer);
	~DataLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

protected:
	int	chns_;
};

}

