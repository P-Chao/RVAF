#include "CenterPointLayer.h"

namespace svaf{


CenterPointLayer::CenterPointLayer(LayerParameter& layer) : Layer(layer)
{
}

CenterPointLayer::~CenterPointLayer()
{
}

bool CenterPointLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_GE(images.size(), 2) << "";
	pWorld = (World*)param;
	int type = 2;
	switch (type){
	case 2:
		ROICenter(images);
		break;
	default:
		break;
	}
	return true;
}

void CenterPointLayer::ROICenter(vector<Block>& images){
	auto roi0 = images[0].roi;
	auto roi1 = images[1].roi;
	images[0].pMatch = &images[1];

	__t.StartWatchTimer();
	images[0].points.clear();
	images[1].points.clear();
	images[0].points.push_back(Point2f(roi0.width / 2.0, roi0.height / 2.0));
	images[1].points.push_back(Point2f(roi1.width / 2.0, roi1.height / 2.0));
	images[0].ptidx.push_back(0);
	__t.ReadWatchTimer(__name + " Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}

	LOG(INFO) << "ROI Center Has Been Setted.";
	RLOG("ROI CenterHas Been Selected.");
}

}
