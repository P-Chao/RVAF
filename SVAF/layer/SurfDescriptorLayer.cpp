/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
Surf特征描述
*/

#include "SurfDescriptorLayer.h"
#include "../../SurfDetect/common.h"

using namespace pc;

namespace pc{
	extern AlgParam algparam;
	extern SurfParam suparam;
}

namespace svaf{

// 构造函数
SurfDescriptorLayer::SurfDescriptorLayer(LayerParameter& layer) : Layer(layer)
{
}

// 析构函数
SurfDescriptorLayer::~SurfDescriptorLayer()
{
}

// 运行算法
bool SurfDescriptorLayer::Run(vector<Block>& images, vector<Block>& disp, 
	LayerParameter& layer, void* param){
	SetParam(layer);
	for (int i = 0; i < images.size(); ++i){
		// 调用Surf算法生成描述符
		__t.StartWatchTimer();
		SurfDescriptor(images[i].image, images[i].points, images[i].points_sc, images[i].despciptors);
		__t.ReadWatchTimer("My Surf Desp Time");
		if (__logt){
			char alicia[3];
			sprintf(alicia, "%d", i);
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// 设置Surf描述子是否具有旋转不变性
void SurfDescriptorLayer::SetParam(LayerParameter& layer){
	suparam.upright = layer.surfdescriptor_param().upright();
}

}

