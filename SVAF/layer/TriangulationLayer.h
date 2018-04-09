/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#pragma once
#include "StereoLayer.h"

#pragma comment(lib, "libeng.lib")
#pragma comment(lib, "libmx.lib")
#pragma comment(lib, "libmat.lib")
#pragma comment(lib, "libmex.lib")

namespace Matlab{
#include <engine.h>
}

namespace svaf{

class TriangulationLayer :
	public StereoLayer
{
public:
	explicit TriangulationLayer(LayerParameter& layer);
	~TriangulationLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
protected:
	void OpenMatlab();
	void CloseMatlab();
	void ComputeWorld();
	void ComputeWorld(double lx, double ly, double rx, double ry,
		double& xx, double& yy, double& zz);

private:
	World *pWorld_;
	Matlab::Engine* m_Ep;

	string toolbox_dir, calibmat_dir;
	bool isMatlabVisible;
	bool isSavePointCloud;
};

}

