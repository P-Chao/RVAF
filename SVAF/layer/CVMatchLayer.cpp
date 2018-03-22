/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#include "CVMatchLayer.h"
#include <opencv2\legacy\legacy.hpp>

namespace svaf{

CVMatchLayer::CVMatchLayer(LayerParameter& layer) : Layer(layer)
{
	type = layer.cvmatch_param().type();
	crossCheck = layer.cvmatch_param().crosscheck();
	switch (type)
	{
	case svaf::CVMatchParameter_MatchType_BFL1:
		ptr = &CVMatchLayer::BFL1;
		matchname = "BFL1";
		break;
	case svaf::CVMatchParameter_MatchType_BFL2:
		ptr = &CVMatchLayer::BFL2;
		matchname = "BFL2";
		break;
	case svaf::CVMatchParameter_MatchType_BFH1:
		ptr = &CVMatchLayer::BFH1;
		matchname = "BFH1";
		break;
	case svaf::CVMatchParameter_MatchType_BFH2:
		ptr = &CVMatchLayer::BFH2;
		matchname = "BFH2";
		break;
	case svaf::CVMatchParameter_MatchType_FLANN:
		ptr = &CVMatchLayer::FLANN;
		matchname = "FLANN";
		break;
	default:
		break;
	}
}


CVMatchLayer::~CVMatchLayer()
{
}

bool CVMatchLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
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

	(this->*ptr)(images, disp);

	if (task_type == SvafApp::POINT_MATCH){
		__bout = true;
	} else {
		__bout = false;
	}

	if (__show || __save || __bout){
		Mat img_match;
		drawMatches(images[0].image, images[0].keypoint, images[1].image, images[1].keypoint,
			images[0].matches, img_match);
		disp.push_back(Block(layername + matchname, img_match, __show, __save, __bout));
	}

	return true;
}

bool CVMatchLayer::BFL1(vector<Block>& images, vector<Block>& disp){
	BFMatcher matcher(NORM_L1, crossCheck);
	vector<DMatch> matches;
	images[0].pMatch = &images[1];
	matcher.match(images[0].descriptors, images[1].descriptors, matches);
	LOG(INFO) << "Matched Count: " << matches.size();

	float max_dist = FLT_MIN, min_dist = FLT_MAX;
	for (int i = 0; i < images[0].descriptors.rows; ++i){
		float dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	LOG(INFO) << "min distance: " << min_dist;
	LOG(INFO) << "max distance: " << max_dist;

	for (int i = 0; i < images[0].descriptors.rows; ++i){
		if (matches[i].distance < 2 * min_dist){
			images[0].matches.push_back(matches[i]);
		}
	}

	LOG(INFO) << "Good Match Count: " << images[0].matches.size();
	return true;
}

bool CVMatchLayer::BFL2(vector<Block>& images, vector<Block>& disp){
	BFMatcher matcher(NORM_L2, crossCheck);
	vector<DMatch> matches;
	images[0].pMatch = &images[1];
	matcher.match(images[0].descriptors, images[1].descriptors, matches);
	LOG(INFO) << "Matched Count: " << matches.size();

	float max_dist = FLT_MIN, min_dist = FLT_MAX;
	for (int i = 0; i < images[0].descriptors.rows; ++i){
		float dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	LOG(INFO) << "min distance: " << min_dist;
	LOG(INFO) << "max distance: " << max_dist;

	for (int i = 0; i < images[0].descriptors.rows; ++i){
		if (matches[i].distance < 2 * min_dist){
			images[0].matches.push_back(matches[i]);
		}
	}

	LOG(INFO) << "Good Match Count: " << images[0].matches.size();
	return true;
}

bool CVMatchLayer::BFH1(vector<Block>& images, vector<Block>& disp){
	BFMatcher matcher(NORM_HAMMING, crossCheck);
	vector<DMatch> matches;
	images[0].pMatch = &images[1];
	matcher.match(images[0].descriptors, images[1].descriptors, matches);
	LOG(INFO) << "Matched Count: " << matches.size();

	float max_dist = FLT_MIN, min_dist = FLT_MAX;
	for (int i = 0; i < images[0].descriptors.rows; ++i){
		float dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	LOG(INFO) << "min distance: " << min_dist;
	LOG(INFO) << "max distance: " << max_dist;

	for (int i = 0; i < images[0].descriptors.rows; ++i){
		if (matches[i].distance < 2 * min_dist){
			images[0].matches.push_back(matches[i]);
		}
	}

	LOG(INFO) << "Good Match Count: " << images[0].matches.size();
	return true;
}

bool CVMatchLayer::BFH2(vector<Block>& images, vector<Block>& disp){
	BFMatcher matcher(NORM_HAMMING2, crossCheck);
	vector<DMatch> matches;
	images[0].pMatch = &images[1];
	matcher.match(images[0].descriptors, images[1].descriptors, matches);
	LOG(INFO) << "Matched Count: " << matches.size();

	float max_dist = FLT_MIN, min_dist = FLT_MAX;
	for (int i = 0; i < images[0].descriptors.rows; ++i){
		float dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	LOG(INFO) << "min distance: " << min_dist;
	LOG(INFO) << "max distance: " << max_dist;

	for (int i = 0; i < images[0].descriptors.rows; ++i){
		if (matches[i].distance < 2 * min_dist){
			images[0].matches.push_back(matches[i]);
		}
	}

	LOG(INFO) << "Good Match Count: " << images[0].matches.size();
	return true;
}

bool CVMatchLayer::FLANN(vector<Block>& images, vector<Block>& disp){
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	images[0].pMatch = &images[1];
	matcher.match(images[0].descriptors, images[1].descriptors, matches);
	LOG(INFO) << "Matched Count: " << matches.size();

	float max_dist = FLT_MIN, min_dist = FLT_MAX;
	for (int i = 0; i < images[0].descriptors.rows; ++i){
		float dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	LOG(INFO) << "min distance: " << min_dist;
	LOG(INFO) << "max distance: " << max_dist;

	for (int i = 0; i < images[0].descriptors.rows; ++i){
		if (matches[i].distance < 2 * min_dist){
			images[0].matches.push_back(matches[i]);
		}
	}

	LOG(INFO) << "Good Match Count: " << images[0].matches.size();

	return true;
}

}
