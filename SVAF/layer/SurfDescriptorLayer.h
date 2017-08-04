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

