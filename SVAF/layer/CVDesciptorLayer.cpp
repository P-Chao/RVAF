/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
OpenCV实现特征描述
*/


#include "CVDesciptorLayer.h"
#include <opencv2\nonfree\nonfree.hpp>

using namespace pc;

namespace svaf{

// 构造函数
CVDesciptorLayer::CVDesciptorLayer(LayerParameter& layer) : Layer(layer)
{
	type = layer.cvdescriptor_param().type(); // 特征描述类型
	brieflength = layer.cvdescriptor_param().brief_param().length(); // 描述符长度
	// 根据特征描述类型创建描述符（OpenCV）
	switch (type)
	{
	case svaf::CVDescriptorParameter_DespType_SIFT:
		ptr = &CVDesciptorLayer::Sift;
		despname = "SIFT";
		break;
	case svaf::CVDescriptorParameter_DespType_SURF:
		ptr = &CVDesciptorLayer::Surf;
		despname = "SURF";
		break;
	case svaf::CVDescriptorParameter_DespType_BRIEF:
		ptr = &CVDesciptorLayer::Brief;
		despname = "Brief";
		break;
	case svaf::CVDescriptorParameter_DespType_BRISK:
		ptr = &CVDesciptorLayer::Brisk;
		despname = "BRISK";
		break;
	case svaf::CVDescriptorParameter_DespType_ORB:
		ptr = &CVDesciptorLayer::ORB;
		despname = "ORB";
		break;
	case svaf::CVDescriptorParameter_DespType_FREAK:
		ptr = &CVDesciptorLayer::Freak;
		despname = "FREAK";
		break;
	case svaf::CVDescriptorParameter_DespType_OPPONENT:
		ptr = &CVDesciptorLayer::OppenentColor;
		despname = "Oppenent";
	default:
		break;
	}

}

// 析构函数
CVDesciptorLayer::~CVDesciptorLayer()
{
}

// 运行特征描述算法
bool CVDesciptorLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	
	(this->*ptr)(images, disp);

	return true;
}

// OpenCV调用SIFT特征点检测
bool CVDesciptorLayer::Sift(vector<Block>& images, vector<Block>& disp){
	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(despname);
	for (int i = 0; i < images.size(); ++i){
		extractor->compute(images[i].image, images[i].keypoint, images[i].descriptors);
	}
	return true;
}

// OpenCV调用SURF特征点检测
bool CVDesciptorLayer::Surf(vector<Block>& images, vector<Block>& disp){
	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(despname);
	for (int i = 0; i < images.size(); ++i){
		extractor->compute(images[i].image, images[i].keypoint, images[i].descriptors);
	}
	return true;
}

// OpenCV调用Brief特征点检测
bool CVDesciptorLayer::Brief(vector<Block>& images, vector<Block>& disp){
	BriefDescriptorExtractor extractor(brieflength);
	for (int i = 0; i < images.size(); ++i){
		extractor.compute(images[i].image, images[i].keypoint, images[i].descriptors);
	}
	return true;
}

// OpenCV调用Brisk特征点检测
bool CVDesciptorLayer::Brisk(vector<Block>& images, vector<Block>& disp){
	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(despname);
	for (int i = 0; i < images.size(); ++i){
		extractor->compute(images[i].image, images[i].keypoint, images[i].descriptors);
	}
	return true;
}

// OpenCV调用ORB特征点检测
bool CVDesciptorLayer::ORB(vector<Block>& images, vector<Block>& disp){
	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(despname);
	for (int i = 0; i < images.size(); ++i){
		extractor->compute(images[i].image, images[i].keypoint, images[i].descriptors);
	}
	return true;
}

// OpenCV调用Freak特征点检测
bool CVDesciptorLayer::Freak(vector<Block>& images, vector<Block>& disp){
	Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create(despname);
	for (int i = 0; i < images.size(); ++i){
		extractor->compute(images[i].image, images[i].keypoint, images[i].descriptors);
	}
	return true;
}

bool CVDesciptorLayer::OppenentColor(vector<Block>& images, vector<Block>& disp){
	
	// Adapt to color
	return true;
}

}
