/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"

namespace svaf{

class CVDesciptorLayer :
	public Layer
{
public:
	explicit CVDesciptorLayer(LayerParameter& layer);
	~CVDesciptorLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

protected:
	bool Sift(vector<Block>&, vector<Block>&);
	bool Surf(vector<Block>&, vector<Block>&);
	bool Brief(vector<Block>&, vector<Block>&);
	bool Brisk(vector<Block>&, vector<Block>&);
	bool ORB(vector<Block>&, vector<Block>&);
	bool Freak(vector<Block>&, vector<Block>&);
	bool OppenentColor(vector<Block>&, vector<Block>&);

private:
	int		brieflength;
	string	layername;
	string	despname;
	svaf::CVDescriptorParameter_DespType type;
	bool(CVDesciptorLayer::*ptr)(vector<Block>&, vector<Block>&);
	
};

}

