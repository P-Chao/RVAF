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

