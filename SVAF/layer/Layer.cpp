/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
Layer基类
*/

#include "Layer.h"

namespace svaf{

size_t *Layer::id = NULL;					// 帧id，从0开始
Figures<> *Layer::figures = NULL;			// 记录结果的二维数据表
SvafApp Layer::task_type = SvafApp::NONE;	// 用于交互的变量，用于判断界面类型
bool Layer::gui_mode = false;				// 在是否出去GUI模式
Circuit *Layer::pCir = NULL;				// 该指针用于调用发送数据线程

Layer::Layer()
{
}

Layer::Layer(LayerParameter& layer){
	__bout = false;
	__name = layer.name();
	__show = Layer::gui_mode ? false : layer.show();
	__save = layer.save();
	__logi = layer.logi();
	__logt = layer.logt();
}

Layer::~Layer()
{
}

// 进程间通信发送消息
void Layer::RLOG(std::string i){	
	pCir->RLOG(i);
}

}
