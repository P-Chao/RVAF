/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
Surf特征点检测
*/

#include "SurfPointLayer.h"
#include "../../SurfDetect/common.h"

using namespace pc;

namespace pc{
extern AlgParam algparam;
extern SurfParam suparam;
}

namespace svaf{

// 构造函数
SurfPointLayer::SurfPointLayer(LayerParameter& layer) : Layer(layer)
{
}

// 析构函数
SurfPointLayer::~SurfPointLayer()
{
}

// 运行算法
bool SurfPointLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	SetParam(layer);
	for (int i = 0; i < images.size(); ++i){
		// 调用Surf特征点检测算法
		vector<int> label;
		__t.StartWatchTimer();
		SurfPoint(images[i].image, images[i].points, images[i].points_sc, label);
		__t.ReadWatchTimer("My SURF Time");
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = images[i].points.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
		LOG(INFO) << "Surf Detected <" << images[i].points.size() << "> points";
	}

	if (task_type == SvafApp::S_POINT || task_type == SvafApp::B_POINT || task_type == SvafApp::S_POINTDESP || task_type == SvafApp::B_POINTDESP){
		__bout = true;
	} else {
		__bout = false;
	}

	// 结果显示与保存
	if (__show || __save || __bout){
		for (int i = 0; i < images.size(); ++i){
			Mat mat = images[i].image.clone();
			if (images[i].points.size() > 0){
				if (images[i].points_sc.size() == images[i].points.size()){
					for (int j = 0; j < images[i].points.size(); ++j){
						circle(mat, images[i].points[j],
							round(images[i].points_sc[j]), cv::Scalar(255, 0, 0), 1);
					}
				}
				else{
					for (int j = 0; j < images[i].points.size(); ++j){
						circle(mat, images[i].points[j], 1, cv::Scalar(255, 0, 0), 1);
					}
				}
			}
			disp.push_back(Block(images[i].name + " Surf Point", mat, __show, __save, __bout));
		}
	}
	return true;
}

// 设置Surf算法参数
void SurfPointLayer::SetParam(LayerParameter& layer){
	suparam.upright = layer.surfdescriptor_param().upright();	// 是否旋转不变
	suparam.stride = layer.surfpoint_param().stride();	// 步长
	suparam.octaves = layer.surfpoint_param().octaves();	// 尺度层数
	suparam.intervals = layer.surfpoint_param().intervals();	// 高斯模糊层数
	suparam.thresh = layer.surfpoint_param().thresh();			// 海森判别阈值
}

}
