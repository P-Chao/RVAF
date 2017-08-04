#include "DataLayer.h"

namespace svaf{

DataLayer::DataLayer(LayerParameter& layer) : chns_(0), Layer(layer)
{
	if (layer.data_param().has_color()){
		if (layer.data_param().color()){
			chns_ = 3;
		} else{
			chns_ = 1;
		}
	}
}

DataLayer::~DataLayer()
{
}

bool DataLayer::Run(std::vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	if (chns_ == 1){
		for (int i = 0; i < images.size(); ++i){
			if (!images[i].image.empty()){
				if (images[i].image.channels() == 3){
					cvtColor(images[i].image, images[i].image, CV_BGR2GRAY);
				}
			}
		}
	} else{
		for (int i = 0; i < images.size(); ++i){
			if (!images[i].image.empty()){
				if (images[i].image.channels() == 1){
					cvtColor(images[i].image, images[i].image, CV_GRAY2RGB);
				}
			}
		}
	}

	if (__show || __save){
		for (int i = 0; i < images.size(); ++i){
			disp.push_back(Block(images[i].name, images[i].image, __show, __save));
		}
	}
	return true;
}

}

