#pragma once
#include "Layer.h"

namespace svaf{

typedef unsigned int uint;

class StereoRectifyLayer :
	public Layer
{
public:
	static void *pTable[14][2];
	static void ReleaseTable();
	void ReadTable(const string&);

public:
	explicit StereoRectifyLayer(LayerParameter& layer);
	~StereoRectifyLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
protected:
	static uint l_length, r_length;
	static uint cols, rows;
};


}

