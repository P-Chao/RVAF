#pragma once
#include "Layer.h"
#include "StereoLayer.h"

namespace svaf{

typedef struct _UnionFeature{
	int featid;
	int	imgIndex;
	int refIndex;
	Mat	RT;
	vector<float> despl;
	vector<float> despr;
	Point3f pointL;
	// used to display
	Point2f pointl;
	Point2f pointr;
	
} Feat;

class FeaturePoolLayer :
	public StereoLayer
{
public:
	explicit FeaturePoolLayer(LayerParameter& layer);
	~FeaturePoolLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
	
protected:
	bool RunForFeaturePool(vector<Block>&, vector<Block>&, LayerParameter&, void*);
	void GenerateFeaturePool(vector<Block>&);
	bool ExcuteEularMatch(vector<Block>&);
	void ClearFeaturePool();
	void DisplayResult(vector<Block>&, vector<Block>&);
	//Feat SearchFeaturePool(vector<float>&, vector<float>&);
	int SearchFeaturePool(vector<float>&, vector<float>&);
	void StoreToFeaturePool();
	Mat SolveRT(vector<Point3f>&, vector<Point3f>&);
	Mat SolveRT(vector<Point3f>&, vector<Point2f>&);

private:
	World *pWorld_;
	
	vector<Feat> featpool_;

	vector<Mat> left;
	vector<Mat> right;

	vector<int> l_l_idx;
	vector<int> l_r_idx;
	vector<int> r_l_idx;
	vector<int> r_r_idx;
};

}

