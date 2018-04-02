/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "StereoLayer.h"

namespace svaf{

class SgmMatchLayer :
	public StereoLayer
{
public:
	explicit SgmMatchLayer(LayerParameter& layer);
	~SgmMatchLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

private:
	int		max_disp;
	int		factor;
	int		dispmr;
	float	r1;
	float	r2;
	string	prefix;
	bool	savetxt;

};

}
