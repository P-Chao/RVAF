/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
SurfÌØÕ÷µã¼ì²â
*/

#include "SurfPointLayer.h"
#include "../../SurfDetect/common.h"

using namespace pc;

namespace pc{
extern AlgParam algparam;
extern SurfParam suparam;
}

namespace svaf{

SurfPointLayer::SurfPointLayer(LayerParameter& layer) : Layer(layer)
{
}


SurfPointLayer::~SurfPointLayer()
{
}

bool SurfPointLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	SetParam(layer);
	for (int i = 0; i < images.size(); ++i){
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

void SurfPointLayer::SetParam(LayerParameter& layer){
	suparam.upright = layer.surfdescriptor_param().upright();
	suparam.stride = layer.surfpoint_param().stride();
	suparam.octaves = layer.surfpoint_param().octaves();
	suparam.intervals = layer.surfpoint_param().intervals();
	suparam.thresh = layer.surfpoint_param().thresh();
}

}
