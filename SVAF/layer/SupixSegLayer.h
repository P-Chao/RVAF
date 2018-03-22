/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"

namespace svaf{

class SupixSegLayer :
	public Layer
{
public:
	explicit SupixSegLayer(LayerParameter& layer);
	~SupixSegLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

private:
	int K;
	int M;
	bool optint;
	bool saveseg;
	string segname;

};

}

