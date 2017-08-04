#pragma once
#include "Layer.h"

namespace svaf{



class ECMatchLayer :
	public Layer
{
public:
	explicit ECMatchLayer(LayerParameter& layer);
	~ECMatchLayer();
	virtual bool Run(vector<Block>&, vector<Block>&, LayerParameter&, void*);
protected:
	bool EularMatch(vector<Block>&);
	bool ECEular(vector<Block>&);
	bool ECEularCrossCheck(vector<Block>&);
private:
	void debug_showmatch(vector<Block>& images){
		for (int i = 0; i < images[0].matches.size(); ++i){
			Mat mat0 = images[0].image.clone();
			Mat mat1 = images[0].pMatch->image.clone();
			Rect roi0(0, 0, mat0.cols, mat0.rows);
			Rect roi1(mat0.cols, 0, mat1.cols, mat1.rows);
			int width = mat0.cols + mat1.cols;
			int height = max(mat0.rows, mat1.rows);
			Mat image(height, width, mat0.type());
			mat0.copyTo(image(roi0));
			mat1.copyTo(image(roi1));
			Block image1 = *images[0].pMatch;
			int id0 = images[0].matches[i].queryIdx;
			int id1 = images[0].matches[i].trainIdx;
			Point2f pt0 = images[0].keypoint[id0].pt;
			Point2f pt1 = images[1].keypoint[id1].pt;
			pt1.x += mat0.cols;
			line(image, pt0, pt1, Scalar(255, 128, 0));
			imshow("temp", image);
			switch (waitKey(0)){
			case 'a':
				continue;
				break;
			case 'r':
				continue;
				break;
			default:
				return;
				break;
			}
		}
	}
};

}
