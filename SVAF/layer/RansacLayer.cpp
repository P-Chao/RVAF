#include "RansacLayer.h"

namespace svaf{

RansacLayer::RansacLayer(LayerParameter& layer) : Layer(layer)
{
}


RansacLayer::~RansacLayer()
{
}

bool RansacLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_GE(images.size(), 2) << "Need Binocular Image!";
	CHECK_NOTNULL(images[0].pMatch);
	Block& image0 = images[0];
	Block& image1 = *images[0].pMatch;
	
	float thresh = layer.ransac_param().thresh();

	vector<Point2f> pt0, pt1;
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}
		pt0.push_back(image0.points[i]);
		pt1.push_back(image1.points[image0.ptidx[i]]);
	}

	if (pt0.size() < 4){
		LOG(ERROR) << "Ransac Need More Than 4 Point.";
		LOG(ERROR) << "\nRansac No Work!\n";
		return true;
	}

	vector<uchar> mask(pt0.size());
	__t.StartWatchTimer();
	Mat homography = findHomography(pt1, pt0, mask, CV_RANSAC, thresh);
	__t.ReadWatchTimer("Ransac Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}

	int count = 0, j = 0;
	for (int i = 0; i < image0.ptidx.size(); ++i){
		if (image0.ptidx[i] < 0){
			continue;
		}
		if (!mask[j]){
			image0.ptidx[i] = -1;
		} else{
			count++;
		}
		j++;
	}
	(*figures)[__name][*id] = count;
	LOG(INFO) << "Before Ransac: " << mask.size();
	LOG(INFO) << "After Ransac: " << count;

	if (task_type == SvafApp::RANSAC_MATCH){
		__bout = true;
	} else {
		__bout = false;
	}

	if (__show || __save || __bout){
		Mat mat0 = images[0].image.clone();
		Mat mat1 = images[0].pMatch->image.clone();
		Rect roi0(0, 0, mat0.cols, mat0.rows);
		Rect roi1(mat0.cols, 0, mat1.cols, mat1.rows);
		int width = mat0.cols + mat1.cols;
		int height = max(mat0.rows, mat1.rows);
		Mat image(height, width, mat0.type());
		mat0.copyTo(image(roi0));
		mat1.copyTo(image(roi1));

		for (int i = 0; i < images[0].ptidx.size(); ++i){
			if (images[0].ptidx[i] < 0){
				continue;
			}
			Point2f pt0 = images[0].points[i];
			Point2f pt1 = images[1].points[images[0].ptidx[i]];
			pt1.x += mat0.cols;
			line(image, pt0, pt1, Scalar(255, 128, 0));
		}
		disp.push_back(Block("Ransac", image, __show, __save, __bout));
	}

	return true;
}

}
