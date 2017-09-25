#include "SgmMatchLayer.h"
#include "../../SuperPixelSegment/svafinterface.h"

using namespace pc;

namespace svaf{

SgmMatchLayer::SgmMatchLayer(LayerParameter& layer) : StereoLayer(layer)
{
	max_disp = layer.sgm_param().max_disp();
	factor = layer.sgm_param().factor();
	dispmr = layer.sgm_param().dispmr();
	r1 = layer.sgm_param().r1();
	r2 = layer.sgm_param().r2();
	savetxt = layer.sgm_param().savetxt();
}


SgmMatchLayer::~SgmMatchLayer()
{
}

bool SgmMatchLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_GE(images.size(), 2) << "Need Image Pairs";
	
	prefix = string("tmp/SGM_") + Circuit::time_id_;
	Mat l_disp, r_disp, fill, check;
	__t.StartWatchTimer();
	SgmMatch(images[0].image, images[1].image, l_disp, r_disp, check, fill,
		max_disp, factor, dispmr, r1, r2, prefix, savetxt);
	__t.ReadWatchTimer("SGM Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}

	if (task_type == SvafApp::STEREO_MATCH){
		__bout = true;
	} else {
		__bout = false;
	}

	if (__show || __save){
		disp.push_back(Block("l_disp", l_disp, __show, __save));
		disp.push_back(Block("r_disp", r_disp, __show, __save));
		disp.push_back(Block("check", check, __show, __save, __bout));
		disp.push_back(Block("fill", fill, __show, __save, __bout));
	}
	LOG(INFO) << "disparity map has been computed.";
	CHECK_EQ(l_disp.type(), CV_16U) << "disparity map type error!";

	images[0].pMatch = &images[1];
	images[0].points.clear();
	images[1].points.clear();
	images[0].ptidx.clear();
	Mat &dispmap = fill;
	int idx = 0;
	for (int i = 0; i < fill.rows; ++i){
		ushort *ptr = fill.ptr<ushort>(i);
		for (int j = 0; j < fill.cols; ++j){
			ushort val = ptr[j];
			float dispval = val / (float)factor;
			if (dispval < max_disp){
				images[0].points.push_back(Point2f(i, j));
				images[1].points.push_back(Point2f(i + dispval, j));
				images[0].ptidx.push_back(idx);
				idx++;
			}
		}
	}
	LOG(INFO) << "Dense <" << idx << "> pairs point.";
	return true;
}


}
