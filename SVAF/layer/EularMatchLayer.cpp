/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/


#include "EularMatchLayer.h"

namespace svaf{

EularMatchLayer::EularMatchLayer(LayerParameter& layer) : Layer(layer)
{
}


EularMatchLayer::~EularMatchLayer()
{
}

bool EularMatchLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_GE(images.size(), 2) << "Need Image Pair(" << images.size() <<")";
	

	images[0].pMatch = &images[1];
	images[1].pMatch = &images[0];
	thresh = layer.eularmatch_param().thresh();
	
	

	if (!images[0].descriptors.empty()){
		if (images[0].keypoint.empty() || images[1].keypoint.empty()){
			LOG(ERROR) << "Match Not Run, No Point";
			return true;
		}
		if (images[0].descriptors.empty() || images[1].descriptors.empty()){
			LOG(ERROR) << "Match Not Run, No Descriptors";
			return true;
		}
		if (images[0].keypoint.size() != images[0].descriptors.rows ||
			images[1].keypoint.size() != images[1].descriptors.rows){
			LOG(ERROR) << "Match Not Run, Point and Discriptor not Match";
			return true;
		}
		CHECK_EQ(images[0].descriptors.cols, images[1].descriptors.cols);

		__t.StartWatchTimer();
		EularMatchCVFormat(images);
		__t.ReadWatchTimer("Eular Match Time");
		if (__logt){
			(*figures)[__name + "cv_t"][*id] = (float)__t;
		}
		LOG(INFO) << "Eular Matched <" << images[0].matches.size() << "> points.";

		if (task_type == SvafApp::POINT_MATCH){
			__bout = true;
		} else {
			__bout = false;
		}

		if (__show || __save || __bout){
			Mat img_match;
			drawMatches(images[0].image, images[0].keypoint, images[1].image, images[1].keypoint,
				images[0].matches, img_match);
			disp.push_back(Block("Eular Matched", img_match, __show, __save, __bout));
		}

		if (images[0].matches.size() == 0){
			LOG(ERROR) << "Eular Loss Match";
			return false;
		}
	}
	else{
		if (images[0].points.size() == 0 || images[1].points.size() == 0){
			LOG(ERROR) << "No Feature Stored!";
			LOG(ERROR) << "\nLoop Cut Short\n";
			return false;
		}
		
		CHECK_EQ(images[0].despciptors[0].size(), images[1].despciptors[0].size())
			<< "Descriptor Dimention Not Equal!";
		CHECK_EQ(images[0].despciptors.size(), images[0].points.size()) << "Desp Count Not Match!";
		CHECK_EQ(images[1].despciptors.size(), images[1].points.size()) << "Desp Count Not Match!";

		__t.StartWatchTimer();
		int matchcount = EularMatch(images);
		__t.ReadWatchTimer("Eular Match Time");
		if (__logt){
			(*figures)[__name + "_t"][*id] = (float)__t;
		}
		(*figures)[__name][*id] = matchcount;
		LOG(INFO) << "Eular Matched <" << matchcount << "> points.";

		if (__show || __save){
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
			disp.push_back(Block("Eular Match", image, __show, __save));
		}

		if (matchcount == 0){
			LOG(ERROR) << "Eular Loss Match";
			return false;
		}
	}

	return true;
}

int EularMatchLayer::EularMatch(vector<Block>& images){
	int count = 0;
	float d0, d1, dist;
	unsigned int match;
	const unsigned int desp0_size = images[0].despciptors.size();
	const unsigned int desp1_size = images[1].despciptors.size();
	const unsigned int desp_length = images[0].despciptors[0].size();

	images[0].ptidx.resize(desp0_size, -1);
	for (int i = 0; i < desp0_size; ++i){
		d0 = d1 = FLT_MAX;
		for (int j = 0; j < desp1_size; ++j){
			dist = 0.0f;
			for (int k = 0; k < desp_length; ++k){
				float diff = images[0].despciptors[i][k] - images[1].despciptors[j][k];
				dist += diff * diff;
			}
			dist = sqrt(dist);
	
			if (dist < d0){
				d1 = d0;
				d0 = dist;
				match = j;
			}
			else if (dist < d1){
				d1 = dist;
			}
		}

		if (d0 / d1 < thresh /*0.65*/){
			images[0].ptidx[i] = match;
			count++;
		}
	}
	return count;
}

void EularMatchLayer::EularEpipolarConstraint(vector<Block>& images){

}

bool EularMatchLayer::EularMatchCVFormat(vector<Block>& images){
	int p1_size = images[0].descriptors.rows;
	int p2_size = images[1].descriptors.rows;
	int length = images[0].descriptors.cols;
	float d0, d1, dist;

	DMatch match;
	switch (images[0].descriptors.depth()){
	case CV_8U:
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<uchar>(i)[k]
						- images[1].descriptors.ptr<uchar>(j)[k];
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
		break;
	case CV_8S:
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<char>(i)[k]
						- images[1].descriptors.ptr<char>(j)[k];
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
		break;
	case CV_16U:
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<unsigned short>(i)[k]
						- images[1].descriptors.ptr<unsigned short>(j)[k];
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
		break;
	case CV_16S:
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<short>(i)[k]
						- images[1].descriptors.ptr<short>(j)[k];
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
		break;
	case CV_32S:
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<int>(i)[k]
						- images[1].descriptors.ptr<int>(j)[k];
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
		break;
	case CV_32F:
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<float>(i)[k]
						- images[1].descriptors.ptr<float>(j)[k];
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
		break;
	case CV_64F:
		for (int i = 0; i < p1_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			for (int j = 0; j < p2_size; ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<double>(i)[k]
						- images[1].descriptors.ptr<double>(j)[k];
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
		break;
	default:
		break;

	}


	return true;
}

}
