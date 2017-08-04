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


