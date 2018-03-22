/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
数据流接口
*/


#include "DataLayer.h"

namespace svaf{

// 构造函数
DataLayer::DataLayer(LayerParameter& layer) : chns_(0), Layer(layer)
{
	// 根据脚本判断是否将图像转换为灰度或彩色
	if (layer.data_param().has_color()){
		if (layer.data_param().color()){
			chns_ = 3;
		} else{
			chns_ = 1;
		}
	}
}

// 析构函数
DataLayer::~DataLayer()
{
}

// 运行
bool DataLayer::Run(std::vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	if (chns_ == 1){
		// 转化为单通道图像处理
		for (int i = 0; i < images.size(); ++i){
			if (!images[i].image.empty()){
				if (images[i].image.channels() == 3){
					cvtColor(images[i].image, images[i].image, CV_BGR2GRAY);
				}
			}
		}
	} else{
		// 转化为三通道图像
		for (int i = 0; i < images.size(); ++i){
			if (!images[i].image.empty()){
				if (images[i].image.channels() == 1){
					cvtColor(images[i].image, images[i].image, CV_GRAY2RGB);
				}
			}
		}
	}

	// 是否输出到数据流
	if (task_type == SvafApp::S_SHOW || task_type == SvafApp::B_SHOW || task_type == SvafApp::SITCH ||
		task_type == SvafApp::STEREO_MATCH){
		__bout = true;
	} else {
		__bout = false;
	}

	// 显示与保存
	if (__show || __save || __bout){
		for (int i = 0; i < images.size(); ++i){
			disp.push_back(Block(images[i].name, images[i].image, __show, __save, __bout));
		}
	}
	return true;
}

}

