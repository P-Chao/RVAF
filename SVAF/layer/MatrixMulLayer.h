#pragma once
#include "Layer.h"

namespace svaf{


class MatrixMulLayer :
	public Layer
{
public:
	explicit MatrixMulLayer(LayerParameter& layer);
	~MatrixMulLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

protected:
	float M[3][4];
};

}

