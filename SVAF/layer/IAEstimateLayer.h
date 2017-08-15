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
	string targetfile;
	int max_iter;
	float min_cors;
	float max_cors;
	float voxel_grid;
	float norm_rad;
	float feat_rad;
	
	float M[3][4];

private:
	World *pWorld_;
	
};

}

