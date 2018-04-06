/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

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
	string targetfile;
	int max_iter;
	float min_cors;
	float max_cors;
	float voxel_grid;
	float norm_rad;
	float feat_rad;

	double a, b, c, x, y, z;

	pcl::PointCloud<pcl::PointXYZ> targetpcd;

private:
	World *pWorld_;

};

}

