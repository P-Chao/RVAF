#pragma once
#include "Layer.h"

namespace svaf{

class EularMatchLayer :
	public Layer
{
public:
	explicit EularMatchLayer(LayerParameter& layer);
	~EularMatchLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
protected:
	int EularMatch(vector<Block>&);
	void EularEpipolarConstraint(vector<Block>&);
	bool EularMatchCVFormat(vector<Block>&);
	template<class T>
	bool EularMatchFunc(vector<Block>&){
		int p1_size = images[0].descriptors.rows;
		int p2_size = images[1].descriptors.rows;
		int length = images[0].descriptors.cols;
		float d0, d1, dist;

		DMatch match;
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<T>(i)[k]
						- images[1].descriptors.ptr<T>(j)[k];
					dist += diff * diff;
				}
				dist = sqrt(dist);

				if (dist < d0){
					d1 = d0;
					d0 = dist;
					match.trainIdx = j;
				}
				else if (dist < d1){
					d1 = dist;
				}
			}

			match.distance = d0;
			if (d0 / d1 < 0.65){
				images[0].matches.push_back(match);
			}
		}
	}

private:
	float thresh;

};

}


