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

