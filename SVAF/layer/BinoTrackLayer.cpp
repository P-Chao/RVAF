/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
双目目标跟踪
*/

#include "BinoTrackLayer.h"

namespace pc{
	extern Mat g_org, g_img;
	extern Binotrack_param binotrack_param;
}

namespace svaf{

bool BinoTrackLayer::reinit_;

// 构造函数
BinoTrackLayer::BinoTrackLayer(LayerParameter& layer) : Layer(layer), adaboost(NULL), scalefactor_(-1)
{
	// 参数读入
	init_type_ = layer.bittrack_param().init_type(); // 初始化目标位置方式
	track_type_ = layer.bittrack_param().track_type();
	trackcount_ = layer.bittrack_param().track_count(); // 连续跟踪帧数量
	
	// haar特征数量
	min_rectnum_ = layer.bittrack_param().haarmin_rectnum();
	max_rectnum_ = layer.bittrack_param().haarmax_rectnum();

	init_negnum_ = layer.bittrack_param().init_negnum();
	negnum_ = layer.bittrack_param().negnum();	// 负样本个数
	posmax_ = layer.bittrack_param().posmax();
	numsel_ = layer.bittrack_param().numsel();
	numfeat_ = layer.bittrack_param().numfeat();
	srchwinsz_ = layer.bittrack_param().srchwinsz();
	negsample_strat_ = layer.bittrack_param().negsample_strat();

	// 学习率
	lrate_ = layer.bittrack_param().lrate();
	posrad_ = layer.bittrack_param().posrad();
	init_posrad_ = layer.bittrack_param().init_posrad();

	// 跟踪控制参数
	pc::binotrack_param.tss = layer.bittrack_param().tss();
	pc::binotrack_param.sync = layer.bittrack_param().sync();
	pc::binotrack_param.pool = layer.bittrack_param().pool();
	pc::binotrack_param.mixfeat = layer.bittrack_param().mixfeat();
	pc::binotrack_param.showprob = layer.bittrack_param().showprob();

	// 跟踪框的尺度参数
	if (layer.bittrack_param().has_scalefactor()){
		scalefactor_ = layer.bittrack_param().scalefactor(); // 为加速跟踪算法，跟踪框处理图像常常经过缩放
	} else{
		trsize_.width = layer.bittrack_param().tr_width();
		trsize_.height = layer.bittrack_param().tr_height();
	}
	
	// 初始化目标位置的方式
	switch (init_type_)
	{
	case svaf::MilTrackParameter_InitType_MOUSE: // 使用鼠标勾选目标位置
		break;
	case svaf::MilTrackParameter_InitType_SELECT: // 在脚本中设定目标位置
		if (layer.bittrack_param().init_rect_size() == 0){
			LOG(FATAL) << "Init Rect Has Not Set!";
		}
		for (int i = 0; i < layer.bittrack_param().init_rect_size(); ++i){
			cv::Rect rect;
			rect.x = layer.bittrack_param().init_rect(i).x();
			rect.y = layer.bittrack_param().init_rect(i).y();
			rect.width = layer.bittrack_param().init_rect(i).width();
			rect.height = layer.bittrack_param().init_rect(i).height();
			init_rects_.push_back(rect);
		}
		break;
	case svaf::MilTrackParameter_InitType_AUTORECT: // 自动选择中心进行跟踪
		break;
	case svaf::MilTrackParameter_InitType_ADABOOST: // 根据目标检测算法初始化目标位置
		if (layer.has_adaboost_param()){
			adaboost = new AdaboostLayer(layer);
		} else{
			LOG(FATAL) << "Adaboost Create Failed!";
		}
		break;
	default:
		LOG(FATAL) << "Unrecognized Init Type！";
		break;
	}

	// 跟踪类型
	switch (track_type_)
	{
	default:
	case svaf::MilTrackParameter_TrackType_MIL: // MIL算法进行跟踪
	case svaf::MilTrackParameter_TrackType_ADA: // Adaboost检测
		LOG(ERROR) << "track type error, default use MIL_GRAY";
	case svaf::MilTrackParameter_TrackType_MIL_GRAY: // MIL灰度图像跟踪
		binotrack_firstframe = pc::miltrack_firstframe_sync;
		binotrack_frame = pc::miltrack_frame_sync;
		break;
	case svaf::MilTrackParameter_TrackType_MIL_RGB: // MIL使用RGB图像跟踪
	case svaf::MilTrackParameter_TrackType_MIL_LUV: // MIL使用LUV色彩空间进行跟踪
	case svaf::MilTrackParameter_TrackType_MIL_HSV: // MIL使用HSV空间进行跟踪
		binotrack_firstframe = pc::miltrack_firstframe_sync_mc;
		binotrack_frame = pc::miltrack_frame_sync_mc;
		break;
	}

}

// 析构函数，释放资源
BinoTrackLayer::~BinoTrackLayer(){
	if (adaboost){
		delete adaboost;
	}
}

// 运行跟踪算法
bool BinoTrackLayer::Run(std::vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	if ((*id) == 0 || reinit_ || (*id) % trackcount_ == 0){
		// 在特定帧初始化目标位置
		trackers_.resize(images.size());
		insize_.resize(images.size());
		winsize_.resize(images.size());
		x_factor_.resize(images.size());
		y_factor_.resize(images.size());
		for (int i = 0; i < images.size(); ++i){
			insize_[i] = images[i].image.size();
		}

		reinit_ = !InitFirstFrame(images, layer);
		LOG(INFO) << trackers_.size() <<" Trackers Has Been Created.";
	} else {
		// 非特定帧运行跟踪算法
		for (int i = 0; i < images.size(); ++i){
			CHECK_EQ(insize_[i], images[i].image.size()) << "Image Size Changed!";
		}
		TrackFrame(images);
	}

	if (reinit_){
		LOG(ERROR) << "Init Tracker Failed";
		LOG(ERROR) << "\nLoop cut short!\n";
		return false;
	}

	// 恢复尺度（为加速跟踪算法，跟踪框处理图像常常经过缩放）
	RecoverScale(images, disp);

	for (int i = 0; i < images.size(); ++i){
		char alicia[3];
		sprintf(alicia, "%d", i);
		(*figures)[__name + alicia + "_x"][*id] = images[i].roi.x;
		(*figures)[__name + alicia + "_y"][*id] = images[i].roi.y;
		(*figures)[__name + alicia + "_wd"][*id] = images[i].roi.width;
		(*figures)[__name + alicia + "_ht"][*id] = images[i].roi.height;
	}
	return true;
}

// 视频跟踪
bool BinoTrackLayer::TrackFrame(vector<Block>& images){
	for (int i = 0; i < images.size(); ++i){
		cv::resize(images[i].image, trackers_[i].img, trackers_[i].trsize);
		// 对于彩色图像选取处理的颜色空间
		if (trackers_[i].img.channels() == 3){
			switch (track_type_)
			{
			default:
			case svaf::MilTrackParameter_TrackType_MIL:
			case svaf::MilTrackParameter_TrackType_ADA:
			case svaf::MilTrackParameter_TrackType_MIL_GRAY:
				cvtColor(trackers_[i].img, trackers_[i].img, CV_BGR2GRAY);
				break;
			case svaf::MilTrackParameter_TrackType_MIL_RGB:
				break;
			case svaf::MilTrackParameter_TrackType_MIL_LUV:
				cvtColor(trackers_[i].img, trackers_[i].img, CV_BGR2Luv);
				break;
			case svaf::MilTrackParameter_TrackType_MIL_HSV:
				break;
			}
		}
		else{
			if (track_type_ != svaf::MilTrackParameter_TrackType_MIL_GRAY){
				LOG(FATAL) << "MIL Run On Multi Channels, But Image Only Have One Channel";
			}
		}
	}
	__t.StartWatchTimer();
	// 输入参数运行跟踪
	binotrack_frame(trackers_[0].img, trackers_[0].rect, trackers_[0].trparam, trackers_[0].ftrparam,
		trackers_[1].img, trackers_[1].rect, trackers_[1].trparam, trackers_[1].ftrparam);
	__t.ReadWatchTimer("Bino Track Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}
	return true;
}

// 目标位置的初始化
bool BinoTrackLayer::InitFirstFrame(vector<Block>& images, LayerParameter& layer){
	vector<Rect> ada_rect;
	// Adaboost目标检测方式初始化
	if (init_type_ == MilTrackParameter_InitType_ADABOOST){
		bool retval = adaboost->RunForOneRect(images, layer, ada_rect);
		if (!retval){ // 检测失败，需要重新运行初始化
			reinit_ = true;
			LOG(ERROR) << "Adaboost Init Rect Failed.";
			return false;
		}
	}

	for (int i = 0; i < images.size(); ++i){
		switch (init_type_){
		case svaf::MilTrackParameter_InitType_MOUSE:
			// 使用鼠标勾选目标位置
			pc::Rect r;
			pc::g_org = images[i].image;
			pc::g_org.copyTo(pc::g_img);
			namedWindow("Select Track Target");
			setMouseCallback("Select Track Target", pc::on_mouse, &r);
			imshow("Select Track Target", pc::g_img);
			waitKey(0);

			init_rect_.x = r.x;
			init_rect_.y = r.y;
			init_rect_.width = r.width;
			init_rect_.height = r.height;
			destroyWindow("Select Track Target");
			break;
		case svaf::MilTrackParameter_InitType_SELECT:
			// 使用脚本设置的像素坐标
			if (i >= init_rects_.size()){
				init_rect_ = init_rects_[0];
			} else{
				init_rect_ = init_rects_[i];
			}
			break;
		case svaf::MilTrackParameter_InitType_AUTORECT:
			// 图像中心坐标为目标位置
			init_rect_.width = images[i].image.cols / 12;
			init_rect_.height = images[i].image.rows / 8;
			init_rect_.x = images[i].roi.width / 2 - init_rect_.width / 2;
			init_rect_.y = images[i].roi.height / 2 - init_rect_.width / 2;
			break;
		case svaf::MilTrackParameter_InitType_ADABOOST:
			// 使用Adaboost检测得到的位置坐标
			init_rect_ = ada_rect[i];
			break;
		default:
			break;
		}

		winsize_[i].width = init_rect_.width;
		winsize_[i].height = init_rect_.height;

		InitSetParam(trackers_[i]);
		ComputeScale(i);
	}

	// 按照尺度或大小缩放图像并转换色彩空间
	for (int i = 0; i < images.size(); ++i){
		cv::resize(images[i].image, trackers_[i].img, trackers_[i].trsize);
		if (trackers_[i].img.channels() == 3){
			switch (track_type_)
			{
			default:
			case svaf::MilTrackParameter_TrackType_MIL:
			case svaf::MilTrackParameter_TrackType_ADA:
			case svaf::MilTrackParameter_TrackType_MIL_GRAY:
				cvtColor(trackers_[i].img, trackers_[i].img, CV_BGR2GRAY);
				break;
			case svaf::MilTrackParameter_TrackType_MIL_RGB:
				break;
			case svaf::MilTrackParameter_TrackType_MIL_LUV:
				break;
			case svaf::MilTrackParameter_TrackType_MIL_HSV:
				break;
			}
		} else{
			if (track_type_ != svaf::MilTrackParameter_TrackType_MIL_GRAY){
				LOG(FATAL) << "MIL Run On Multi Channels, But Image Only Have One Channel";
			}
		}
	}
	// 运行跟踪
	__t.StartWatchTimer();
	binotrack_firstframe(trackers_[0].img, trackers_[0].rect, trackers_[0].trparam, trackers_[0].ftrparam,
		trackers_[1].img, trackers_[1].rect, trackers_[1].trparam, trackers_[1].ftrparam);
	__t.ReadWatchTimer("Bino Track Init Time");
	if (__logt){
		(*figures)[__name + "_t"][*id] = (float)__t;
	}


	return true;
}

// 初始化跟踪参数
void BinoTrackLayer::InitSetParam(BoostTrack& tracker){
	
	tracker.ftrparam.minRectNum = min_rectnum_;
	tracker.ftrparam.maxRectNum = max_rectnum_;

	tracker.trparam.negnumtrain = negnum_;
	tracker.trparam.init_negnumtrain = init_negnum_;
	tracker.trparam.posradtrain = posrad_;
	tracker.trparam.init_postrainrad = init_posrad_;
	tracker.trparam.posmaxtrain = posmax_;
	tracker.trparam.uselogR = uselogR_;
	tracker.trparam.srchwinsz = srchwinsz_;
	tracker.trparam.negsamplestrat = negsample_strat_;

	tracker.trparam.numFeat = numfeat_;
	tracker.trparam.numSel = numsel_;
	tracker.trparam.lRate = lrate_;

	if (scalefactor_ > 0){
		tracker.rect.x = (int)(init_rect_.x / scalefactor_ + 0.5);
		tracker.rect.y = (int)(init_rect_.y / scalefactor_ + 0.5);
		tracker.rect.width = (int)(init_rect_.width / scalefactor_ + 0.5);
		tracker.rect.height = (int)(init_rect_.height / scalefactor_ + 0.5);
	} else{
		float scalefactor_x = 0;
	}

	tracker.trparam.curt_rect.x = tracker.rect.x;
	tracker.trparam.curt_rect.y = tracker.rect.y;
	tracker.trparam.curt_rect.width = tracker.rect.width;
	tracker.trparam.curt_rect.height = tracker.rect.height;

}

// 计算目标尺度和尺度因子
void BinoTrackLayer::ComputeScale(int i){
	if (scalefactor_ > 0){
		//for (int i = 0; i < trackers_.size(); ++i){
			trackers_[i].rect.x = (int)(init_rect_.x * scalefactor_ + 0.5);
			trackers_[i].rect.y = (int)(init_rect_.y * scalefactor_ + 0.5);
			trackers_[i].rect.width = (int)(init_rect_.width * scalefactor_ + 0.5);
			trackers_[i].rect.height = (int)(init_rect_.height * scalefactor_ + 0.5);

			trackers_[i].trsize = Size(insize_[i].width * scalefactor_ + 0.5, insize_[i].height * scalefactor_ + 0.5);
		//}
	}
	else{
		//for (int i = 0; i < trackers_.size(); ++i){
			
			x_factor_[i] = (float)trsize_.width / (float)insize_[i].width;
			y_factor_[i] = (float)trsize_.height / (float)insize_[i].height;

			trackers_[i].rect.x = (int)(init_rect_.x * x_factor_[i] + 0.5);
			trackers_[i].rect.y = (int)(init_rect_.y * y_factor_[i] + 0.5);
			trackers_[i].rect.width = (int)(init_rect_.width * x_factor_[i] + 0.5);
			trackers_[i].rect.height = (int)(init_rect_.height * y_factor_[i] + 0.5);

			trackers_[i].trsize = trsize_;
		//}
	}
}

// 恢复目标尺度
void BinoTrackLayer::RecoverScale(vector<Block>& images, vector<Block>& disp){

	if (scalefactor_ > 0){
		for (int i = 0; i < trackers_.size(); ++i){
			Mat disp_img = images[i].image.clone();
			if (images[i].image.channels() == 1){
				cvtColor(disp_img, disp_img, CV_GRAY2BGR);
			}
			int x = trackers_[i].rect.x / scalefactor_ + 0.5;
			int y = trackers_[i].rect.y / scalefactor_ + 0.5;
			int width = trackers_[i].rect.width / scalefactor_ + 0.5;
			int height = trackers_[i].rect.height / scalefactor_ + 0.5;

			images[i].roi.x += x;
			images[i].roi.y += y;
			images[i].roi.width = width;
			images[i].roi.height = height;

			if (task_type == SvafApp::S_DETECT || task_type == SvafApp::B_DETECT){
				__bout = true;
			} else {
				__bout = false;
			}

			if (__show || __save || __bout){
				rectangle(disp_img, Rect(x, y, width, height), Scalar(255, 255, 255), 2);
				disp.push_back(Block(images[i].name + " Track", disp_img, __show, __save));
			}

			images[i].image = images[i].image(Rect(x, y, width, height)).clone();
		}
	} else{
		for (int i = 0; i < trackers_.size(); ++i){
			Mat disp_img = images[i].image.clone();
			if (images[i].image.channels() == 1){
				cvtColor(disp_img, disp_img, CV_GRAY2BGR);
			}
			int x = trackers_[i].rect.x / x_factor_[i] + 0.5;
			int y = trackers_[i].rect.y / y_factor_[i] + 0.5;
			int width = trackers_[i].rect.width / x_factor_[i] + 0.5;
			int height = trackers_[i].rect.height / y_factor_[i] +0.5;

			images[i].roi.x += x;
			images[i].roi.y += y;
			images[i].roi.width = width;
			images[i].roi.height = height;

			if (task_type == SvafApp::S_DETECT || task_type == SvafApp::B_DETECT ||
				task_type == SvafApp::PC_TRIANGLE || task_type == SvafApp::PC_MULMATRIX ||
				task_type == SvafApp::PC_REGISTRATION || task_type == SvafApp::PR_CENTER){
				__bout = true;
			} else {
				__bout = false;
			}

			if (__show || __save || __bout){
				rectangle(disp_img, Rect(x, y, width, height), Scalar(255, 255, 255), 2);
				disp.push_back(Block(images[i].name + " Track", disp_img, __show, __save));
			}

			images[i].image = images[i].image(Rect(x, y, width, height)).clone();
		}
	}
	
}

}

