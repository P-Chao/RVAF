#include "SupixSegLayer.h"
#include "../../SuperPixelSegment/svafinterface.h"

using namespace pc;

namespace svaf{


SupixSegLayer::SupixSegLayer(LayerParameter& layer) : Layer(layer)
{
	K = layer.supix_param().k();
	M = layer.supix_param().m();
	optint = layer.supix_param().optint();
	saveseg = layer.supix_param().saveseg();
	segname = layer.supix_param().segname();
}

SupixSegLayer::~SupixSegLayer()
{
}

bool SupixSegLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	for (int i = 0; i < images.size(); ++i){
		Mat mat = SuperPixelSegment(images[i].image, K, M, optint, saveseg, segname);

		if (task_type == SvafApp::S_SUPIX || task_type == SvafApp::B_SUPIX){
			__bout = true;
		} else {
			__bout = false;
		}

		if (__show || __save || __bout){
			disp.push_back(Block(images[i].name + " supix", mat, __show, __save, __bout));
		}
	}
	LOG(INFO) << "superpixel map has been computed.";
	RLOG("superpixel map has been computed.");
	return true;
}

}
