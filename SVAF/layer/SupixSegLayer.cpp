/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
超像素分割（By:朱松）
*/

#include "SupixSegLayer.h"
#include "../../SuperPixelSegment/svafinterface.h"

using namespace pc;

namespace svaf{

// 构造函数
SupixSegLayer::SupixSegLayer(LayerParameter& layer) : Layer(layer)
{
	// 超像素分割参数
	K = layer.supix_param().k(); // K参数
	M = layer.supix_param().m(); // 分割块大小
	optint = layer.supix_param().optint();	// 是否整数
	saveseg = layer.supix_param().saveseg(); // 保存分割文件
	segname = layer.supix_param().segname();
}

// 析构函数
SupixSegLayer::~SupixSegLayer()
{
}

// 运行超像素分割算法
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
