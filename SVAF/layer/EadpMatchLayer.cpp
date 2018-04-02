/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
Eadp立体匹配（By：朱松）
*/

#include "EadpMatchLayer.h"
#include "../../SuperPixelSegment/svafinterface.h"

using namespace pc;

namespace svaf{

// 构造函数
EadpMatchLayer::EadpMatchLayer(LayerParameter& layer) : Layer(layer)
{
	// 算法参数
	max_disp = layer.eadp_param().max_disp(); // 最大视差
	factor = layer.eadp_param().factor();
	// 边缘保持滤波器参数
	guildmr = layer.eadp_param().guidmr();
	dispmr = layer.eadp_param().dispmr();
	sg = layer.eadp_param().sg();
	sc = layer.eadp_param().sc();
	r1 = layer.eadp_param().r1();
	r2 = layer.eadp_param().r2();
	// 是否将视差保存为txt文档
	savetxt = layer.eadp_param().savetxt();
}

// 析构函数
EadpMatchLayer::~EadpMatchLayer()
{
}

// 运行
bool EadpMatchLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_GE(images.size(), 2) << "Need Image Pairs";
	prefix = string("tmp/EADP_") + Circuit::time_id_;

	// 执行Eadp立体匹配
	Mat l_disp, r_disp, fill, check;
	__t.StartWatchTimer();
	EadpMatch(images[0].image, images[1].image, l_disp, r_disp, check, fill,
		max_disp, factor, guildmr, dispmr, sg, sc, r1, r2, prefix, savetxt);
	__t.ReadWatchTimer("Eadp Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}

	if (task_type == SvafApp::STEREO_MATCH){
		__bout = true;
	} else {
		__bout = false;
	}

	// 保存视差图结果
	if (__show || __save || __bout){
		disp.push_back(Block("l_disp", l_disp, __show, __save));
		disp.push_back(Block("r_disp", r_disp, __show, __save));
		disp.push_back(Block("check", check, __show, __save, __bout));
		disp.push_back(Block("fill", fill, __show, __save, __bout));
	}
	LOG(INFO) << "disparity map has been computed.";
	RLOG("disparity map has been computed.");
	CHECK_EQ(l_disp.type(), CV_16U) << "disparity map type error!";

	images[0].pMatch = &images[1];
	images[0].points.clear();
	images[1].points.clear();
	images[0].ptidx.clear();
	Mat &dispmap = check;
	int idx = 0;
	for (int i = 0; i < dispmap.rows; ++i){
		ushort *ptr = dispmap.ptr<ushort>(i);
		for (int j = 0; j < dispmap.cols; ++j){
			ushort val = ptr[j];
			float dispval = val / (float)factor;
			if (dispval < max_disp){
				images[0].points.push_back(Point2f(i,j));
				images[1].points.push_back(Point2f(i+dispval,j));
				images[0].ptidx.push_back(idx);
				idx++;
			}
		}
	}
	LOG(INFO) << "Dense <" << idx << "> pairs point.";

	return true;
}

}
