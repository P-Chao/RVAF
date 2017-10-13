#include "Layer.h"

namespace svaf{

size_t *Layer::id = NULL;
Figures<> *Layer::figures = NULL;
SvafApp Layer::task_type = SvafApp::NONE;
bool Layer::gui_mode = false;
Circuit *pCir = NULL;

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

void Layer::RLOG(std::string& i){
	pCir->RLOG(i);
}

}
