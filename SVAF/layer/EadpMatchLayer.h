#pragma once
#include "Layer.h"

namespace svaf{

class EadpMatchLayer :
	public Layer
{
public:
	explicit EadpMatchLayer(LayerParameter& layer);
	~EadpMatchLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

private:
	int		max_disp;
	int		factor;
	int		guildmr;
	int		dispmr;
	float	sg;
	float	sc;
	float	r1;
	float	r2;
	string	prefix;
	bool	savetxt;

};

}
