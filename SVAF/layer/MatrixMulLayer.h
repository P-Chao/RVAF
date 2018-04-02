/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "StereoLayer.h"

namespace svaf{


class MatrixMulLayer :
	public StereoLayer
{
public:
	explicit MatrixMulLayer(LayerParameter& layer);
	~MatrixMulLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

protected:
	float M[3][4];
};

}

