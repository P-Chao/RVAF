#include "ECMatchLayer.h"

namespace svaf{

ECMatchLayer::ECMatchLayer(LayerParameter& layer) : Layer(layer)
{
}


ECMatchLayer::~ECMatchLayer()
{
}

bool ECMatchLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_GE(images.size(), 2) << "Need Image Pairs";
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

	images[0].pMatch = &images[1];
	images[1].pMatch = &images[0];
	
	ECEular(images);
	if (__show || __save){
		Mat img_match;
		drawMatches(images[0].image, images[0].keypoint, images[1].image, images[1].keypoint,
			images[0].matches, img_match);
		disp.push_back(Block("Eular Matched", img_match, __show, __save));
	}

	LOG(INFO) << "ECMatch Matched <" << images[0].matches.size() << "> Points.";

	debug_showmatch(images);

	return true;
}

bool ECMatchLayer::ECEular(vector<Block>& images){
	int p0_size = images[0].descriptors.rows;
	int p1_size = images[1].descriptors.rows;
	int length = images[0].descriptors.cols;

	vector<vector<int>> hash0(images[0].image.rows);
	vector<vector<int>>	hash1(images[1].image.rows);

	for (int i = 0; i < p0_size; ++i){
		hash0[images[0].keypoint[i].pt.y].push_back(i);
	}
	for (int i = 0; i < p1_size; ++i){
		hash1[images[1].keypoint[i].pt.y].push_back(i);
	}

	DMatch match;
	float d0, d1, dist;
	switch (images[0].descriptors.depth())
	{
	case CV_32F:
		for (int i = 0; i < p0_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			vector<int>& hashline = hash1[images[0].keypoint[i].pt.y];
			for (int j = 0; j < hashline.size(); ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<float>(i)[k]
						- images[1].descriptors.ptr<float>(hashline[j])[k];
					dist += diff * diff;
				}
				dist = sqrt(dist);

				if (dist < d0){
					d1 = d0;
					d0 = dist;
					match.trainIdx = hashline[j];
				} else if (dist < d1){
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
		LOG(FATAL) << "No Implyment!";
		break;
	}
	return true;
}

bool ECMatchLayer::ECEularCrossCheck(vector<Block>& images){
	int p0_size = images[0].descriptors.rows;
	int p1_size = images[1].descriptors.rows;
	int length = images[0].descriptors.cols;

	vector<vector<int>> hash0(images[0].image.rows);
	vector<vector<int>>	hash1(images[1].image.rows);

	for (int i = 0; i < p0_size; ++i){
		hash0[images[0].keypoint[i].pt.y].push_back(i);
	}
	for (int i = 0; i < p1_size; ++i){
		hash1[images[1].keypoint[i].pt.y].push_back(i);
	}

	DMatch match;
	float d0, d1, dist;
	switch (images[0].descriptors.depth())
	{
	case CV_32F:
		for (int i = 0; i < p0_size; ++i){
			match.queryIdx = i;
			d0 = d1 = FLT_MAX;
			vector<int>& hashline = hash1[images[0].keypoint[i].pt.y];
			for (int j = 0; j < hashline.size(); ++j){
				dist = 0.0f;
				for (int k = 0; k < length; ++k){
					float diff = images[0].descriptors.ptr<float>(i)[k]
						- images[1].descriptors.ptr<float>(hashline[j])[k];
					dist += diff * diff;
				}
				dist = sqrt(dist);

				if (dist < d0){
					d1 = d0;
					d0 = dist;
					match.trainIdx = hashline[j];
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
		LOG(FATAL) << "No Implyment!";
		break;
	}
	return true;
}

bool ECMatchLayer::EularMatch(vector<Block>& images){
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
