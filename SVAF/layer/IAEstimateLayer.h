#pragma once
#include "StereoLayer.h"

namespace svaf{

class IAEstimateLayer :
	public StereoLayer{
public:
	explicit IAEstimateLayer(LayerParameter& layer);
	~IAEstimateLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
protected:
	

private:
	World *pWorld_;
	
};

}

