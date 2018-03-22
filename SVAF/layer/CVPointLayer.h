/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "Layer.h"

namespace svaf{

class CVPointLayer :
	public Layer
{
public:
	explicit CVPointLayer(LayerParameter& layer);
	~CVPointLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);

protected:
	bool Fast(vector<Block>&, vector<Block>&);
	bool FastX(vector<Block>&, vector<Block>&);
	bool MSER(vector<Block>&, vector<Block>&);
	bool ORB(vector<Block>&, vector<Block>&);
	bool Brisk(vector<Block>&, vector<Block>&);
	bool Freak(vector<Block>&, vector<Block>&);
	bool Star(vector<Block>&, vector<Block>&);
	bool Sift(vector<Block>&, vector<Block>&);
	bool Surf(vector<Block>&, vector<Block>&);
	bool GFTT(vector<Block>&, vector<Block>&);
	bool Harris(vector<Block>&, vector<Block>&);
	bool Dense(vector<Block>&, vector<Block>&);
	bool SimpleBlob(vector<Block>&, vector<Block>&);

	bool Grid(vector<Block>&, vector<Block>&);
	bool Pyramid(vector<Block>&, vector<Block>&);

private:
	string featname;
	svaf::CVPointParameter layerparam;
	svaf::CVPointParameter_PointType type;
	bool(CVPointLayer::*ptr)(vector<Block>&, vector<Block>&);

};

}

