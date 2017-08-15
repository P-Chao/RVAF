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

