/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
OpenCV实现特征点检测
*/

#include "CVPointLayer.h"
#include <opencv2\nonfree\nonfree.hpp>

namespace svaf{

// 构造函数
CVPointLayer::CVPointLayer(LayerParameter& layer) : Layer(layer)
{
	type = layer.cvpoint_param().type(); // 特征点检测算法类型
	switch (type)
	{
	case svaf::CVPointParameter_PointType_FAST: // FAST特征点检测
		ptr = &CVPointLayer::Fast;
		featname = "Fast";
		break;
	case svaf::CVPointParameter_PointType_FASTX: // FASTX特征点检测
		ptr = &CVPointLayer::FastX;
		featname = "FastX";
		break;
	case svaf::CVPointParameter_PointType_MSER: // MSER特征点检测
		ptr = &CVPointLayer::MSER;
		featname = "MSER";
		break;
	case svaf::CVPointParameter_PointType_ORB: // ORB特征点检测
		ptr = &CVPointLayer::ORB;
		featname = "ORB";
		break;
	case svaf::CVPointParameter_PointType_BRISK: // BRISK特征点检测
		ptr = &CVPointLayer::Brisk;
		featname = "Brisk";
		break;
	case svaf::CVPointParameter_PointType_FREAK: // FREAK特征点检测
		ptr = &CVPointLayer::Freak;
		featname = "Freak";
		break;
	case svaf::CVPointParameter_PointType_STAR: // STAR特征点检测
		ptr = &CVPointLayer::Star;
		featname = "Star";
		break;
	case svaf::CVPointParameter_PointType_SIFT: // SIFT特征点检测
		ptr = &CVPointLayer::Sift;
		featname = "Sift";
		break;
	case svaf::CVPointParameter_PointType_SURF: // SURF特征点检测
		ptr = &CVPointLayer::Surf;
		featname = "Surf";
		break;
	case svaf::CVPointParameter_PointType_GFTT: // GFTT特征点检测
		ptr = &CVPointLayer::GFTT;
		featname = "GFTT";
		break;
	case svaf::CVPointParameter_PointType_HARRIS: // Harris特征点检测
		ptr = &CVPointLayer::Harris;
		featname = "Harris";
		break;
	case svaf::CVPointParameter_PointType_DENSE: // DENSE特征点检测
		ptr = &CVPointLayer::Dense;
		featname = "Dense";
		break;
	case svaf::CVPointParameter_PointType_SBLOB: // SBLOB特征点检测
		ptr = &CVPointLayer::SimpleBlob;
		featname = "SimpleBlob";
		break;
	default:
		break;
	}
	
}

// 析构函数
CVPointLayer::~CVPointLayer()
{
}

// 运行算法
bool CVPointLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	// 读取参数
	layerparam = layer.cvpoint_param();

	if (!layer.cvpoint_param().isadd()){
		for (int i = 0; i < images.size(); ++i){
			images[i].keypoint.clear();
			images[i].descriptors.release();
		}
	}

	// 调用算法
	(this->*ptr)(images, disp);

	if (task_type == SvafApp::S_POINT || task_type == SvafApp::B_POINT || task_type == SvafApp::S_POINTDESP || task_type == SvafApp::B_POINTDESP){
		__bout = true;
	} else {
		__bout = false;
	}

	if (__show || __save || __bout){
		for (int i = 0; i < images.size(); ++i){
			Mat img_point;
			drawKeypoints(images[i].image, images[i].keypoint, img_point); // 绘制特征点
			disp.push_back(Block(images[i].name + featname + __name, img_point, __show, __save, __bout));
		}
	}

	return true;
}

// OpenCV调用Fast特征点检测
bool CVPointLayer::Fast(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	FastFeatureDetector fast(layerparam.fast_param().thresh(), layerparam.fast_param().isnms());

	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		fast.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用FastX特征点检测
bool CVPointLayer::FastX(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	int xtype = 0;
	auto ftype = layerparam.fast_param().ftype();
	switch (ftype)
	{
	case svaf::FastParamCP_FastType_T_9_16:
		xtype = FastFeatureDetector::TYPE_9_16;
		break;
	case svaf::FastParamCP_FastType_T_7_12:
		xtype = FastFeatureDetector::TYPE_7_12;
		break;
	case svaf::FastParamCP_FastType_T_5_8:
		xtype = FastFeatureDetector::TYPE_5_8;
		break;
	default:
		break;
	}
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		cv::FASTX(images[i].image, pt, layerparam.fast_param().thresh(), 
			layerparam.fast_param().isnms(), xtype);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用MSER特征点检测
bool CVPointLayer::MSER(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.mser_param();
	cv::MSER mser(param.delta(), param.min_area(), param.max_area(), 
		param.max_varia(), param.min_divers(), param.max_evolution(), 
		param.area_thresh(), param.min_margin(), param.edgeblur_size());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		mser.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用ORB特征点检测
bool CVPointLayer::ORB(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.orb_param();
	auto obrtype = param.scoretype();
	int otype = 0;
	switch (obrtype)
	{
	case svaf::ORBParamCP_OBRScoreType_HARRIS_SCORE:
		otype = ORB::HARRIS_SCORE;
		break;
	case svaf::ORBParamCP_OBRScoreType_FAST_SCORE:
		otype = ORB::FAST_SCORE;
		break;
	default:
		break;
	}
	cv::ORB orb(param.nfeatures(), param.scalefactor(), param.nlevels(), param.edgethresh(), 
		param.firstlevel(), param.wta_k(), otype, param.patchsize());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		orb.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用Brisk特征点检测
bool CVPointLayer::Brisk(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.brisk_param();
	BRISK brisk(param.thresh(), param.octaves(), param.ptscale());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		brisk.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用Freak特征点检测
bool CVPointLayer::Freak(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	FREAK freak;
	LOG(FATAL) << "No Impliment!";
	return false;
}

// OpenCV调用STAR特征点检测
bool CVPointLayer::Star(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.star_param();
	cv::StarDetector star(param.maxsize(), param.response_thresh(), param.projected_thresh(), 
		param.binarized_thresh(), param.nms_size());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		star.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用SIFT特征点检测
bool CVPointLayer::Sift(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.sift_param();
	SIFT sift(param.nfeatures(), param.octaves(), param.contrast_thresh(), 
		param.edge_thresh(), param.sigma());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		sift.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用SURF特征点检测
bool CVPointLayer::Surf(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.surf_param();
	SURF surf(param.hassian_thresh(), param.octaves(), param.intevals(), 
		param.extended(), param.upright());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		surf.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用GFTT特征点检测
bool CVPointLayer::GFTT(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.gftt_param();
	GoodFeaturesToTrackDetector gftt(param.maxcornners(), param.quality(), param.mindist(),
		param.blocksize(), param.useharris(), param.k());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		gftt.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用Harris特征点检测
bool CVPointLayer::Harris(vector<Block>& images, vector<Block>& disp){
	/*vector<KeyPoint> pt;
	Ptr<FeatureDetector> detector = FeatureDetector::create("HARRIS");
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		detector->detect(images[i].image, pt);
		for (int j = 0; j < pt.size(); ++j){
			images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		}
	}
	return true;*/
	vector<KeyPoint> pt;
	auto param = layerparam.gftt_param();
	GoodFeaturesToTrackDetector gftt(param.maxcornners(), param.quality(), param.mindist(),
		param.blocksize(), true, param.k());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		gftt.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用Dense特征点检测
bool CVPointLayer::Dense(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	auto param = layerparam.dense_param();
	DenseFeatureDetector dense(param.initfeatscale(), param.featscalelevel(), param.featscalemul(),
		param.initxystep(), param.initbound(), param.varyxyscale(), param.varybdscale());
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		dense.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

// OpenCV调用Blob特征点检测
bool CVPointLayer::SimpleBlob(vector<Block>& images, vector<Block>& disp){
	vector<KeyPoint> pt;
	SimpleBlobDetector sb;
	for (int i = 0; i < images.size(); ++i){
		pt.clear();
		__t.StartWatchTimer();
		sb.detect(images[i].image, pt);
		__t.ReadWatchTimer(featname + " Time");
		images[i].keypoint.insert(images[i].keypoint.begin(), pt.begin(), pt.end());
		LOG(INFO) << featname << " Detected <" << pt.size() << "> points";
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia][*id] = pt.size();
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

bool CVPointLayer::Grid(vector<Block>& images, vector<Block>& disp){
	Ptr<FeatureDetector> detector = FeatureDetector::create("Grid");
	return false;
}

bool CVPointLayer::Pyramid(vector<Block>& images, vector<Block>& disp){
	Ptr<FeatureDetector> detector = FeatureDetector::create("Pyramid");
	return false;
}

}


