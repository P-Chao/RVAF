#include "Layer.h"

namespace svaf{

size_t *Layer::id = NULL;
Figures<> *Layer::figures = NULL;

Layer::Layer()
{
}

Layer::Layer(LayerParameter& layer){
	__name = layer.name();
	__show = layer.show();
	__save = layer.save();
	__logi = layer.logi();
	__logt = layer.logt();
}

Layer::~Layer()
{
}

}
