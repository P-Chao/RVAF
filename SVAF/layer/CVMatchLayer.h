#pragma once
#include "Layer.h"

namespace svaf{

class CVMatchLayer :
	public Layer
{
public:
	explicit CVMatchLayer(LayerParameter& layer);
	~CVMatchLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

protected:
	bool BFL1(vector<Block>&, vector<Block>&);
	bool BFL2(vector<Block>&, vector<Block>&);
	bool BFH1(vector<Block>&, vector<Block>&);
	bool BFH2(vector<Block>&, vector<Block>&);
	bool FLANN(vector<Block>&, vector<Block>&);

private:
	bool	crossCheck;
	string	layername;
	string	matchname;
	svaf::CVMatchParameter_MatchType type;
	bool(CVMatchLayer::*ptr)(vector<Block>&, vector<Block>&);
};

}


