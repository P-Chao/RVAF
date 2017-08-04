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

