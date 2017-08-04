#pragma once
#include "Layer.h"

namespace svaf{


class CenterPointLayer :
	public Layer
{
public:
	explicit CenterPointLayer(LayerParameter& layer);
	~CenterPointLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

protected:
	void ROICenter(vector<Block>&);

private:
	World *pWorld;

};

}

