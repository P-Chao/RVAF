/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
*/

#include "MilTrackLayer.h"

namespace pc{
	extern Mat g_org, g_img;
}

namespace svaf{

bool MilTrackLayer::reinit_;

MilTrackLayer::MilTrackLayer(LayerParameter& layer) : Layer(layer), adaboost(NULL), scalefactor_(-1)
{
	init_type_ = layer.miltrack_param().init_type();
	track_type_ = layer.miltrack_param().track_type();
	trackcount_ = layer.miltrack_param().track_count();
	
	min_rectnum_ = layer.miltrack_param().haarmin_rectnum();
	max_rectnum_ = layer.miltrack_param().haarmax_rectnum();

	init_negnum_ = layer.miltrack_param().init_negnum();
	negnum_ = layer.miltrack_param().negnum();
	posmax_ = layer.miltrack_param().posmax();
	numsel_ = layer.miltrack_param().numsel();
	numfeat_ = layer.miltrack_param().numfeat();
	srchwinsz_ = layer.miltrack_param().srchwinsz();
	negsample_strat_ = layer.miltrack_param().negsample_strat();

	lrate_ = layer.miltrack_param().lrate();
	posrad_ = layer.miltrack_param().posrad();
	init_posrad_ = layer.miltrack_param().init_posrad();

	if (layer.miltrack_param().has_scalefactor()){
		scalefactor_ = layer.miltrack_param().scalefactor();
	} else{
		trsize_.width = layer.miltrack_param().tr_width();
		trsize_.height = layer.miltrack_param().tr_height();
	}

	switch (init_type_)
	{
	case svaf::MilTrackParameter_InitType_MOUSE:
		break;
	case svaf::MilTrackParameter_InitType_SELECT:
		if (layer.miltrack_param().init_rect_size() == 0){
			LOG(FATAL) << "Init Rect Has Not Set!";
		}
		for (int i = 0; i < layer.miltrack_param().init_rect_size(); ++i){
			cv::Rect rect;
			rect.x = layer.miltrack_param().init_rect(i).x();
			rect.y = layer.miltrack_param().init_rect(i).y();
			rect.width = layer.miltrack_param().init_rect(i).width();
			rect.height = layer.miltrack_param().init_rect(i).height();
			init_rects_.push_back(rect);
		}
		break;
	case svaf::MilTrackParameter_InitType_AUTORECT:
		break;
	case svaf::MilTrackParameter_InitType_ADABOOST:
		if (layer.has_adaboost_param()){
			adaboost = new AdaboostLayer(layer);
		} else{
			LOG(FATAL) << "Adaboost Create Failed!";
		}
		break;
	default:
		LOG(FATAL) << "Unrecognized Init Type£¡";
		break;
	}

	switch (track_type_)
	{
	default:
		LOG(ERROR) << "track type error, deault use MILTrack!";
	case svaf::MilTrackParameter_TrackType_MIL:
		track_frame = pc::miltrack_frame;
		track_firstframe = pc::miltrack_firstframe;
		break;
	case svaf::MilTrackParameter_TrackType_ADA:
		track_frame = pc::adatrack_frame;
		track_firstframe = pc::adatrack_firstframe;
		break;
	}

}

MilTrackLayer::~MilTrackLayer(){
	if (adaboost){
		delete adaboost;
	}
}

bool MilTrackLayer::Run(std::vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	if ((*id) == 0 || reinit_ || (*id) % trackcount_ == 0){
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

bool MilTrackLayer::TrackFrame(vector<Block>& images){
	for (int i = 0; i < images.size(); ++i){
		cv::resize(images[i].image, trackers_[i].img, trackers_[i].trsize);
		if (trackers_[i].img.channels() == 3){
			cvtColor(trackers_[i].img, trackers_[i].img, CV_BGR2GRAY);
		}
		__t.StartWatchTimer();
		track_frame(trackers_[i].img, trackers_[i].rect, trackers_[i].trparam, trackers_[i].ftrparam);
		__t.ReadWatchTimer("MIL Track Time");
		if (__logt){
			char alicia[3];
			sprintf(alicia, "%d", i);
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}
	return true;
}

bool MilTrackLayer::InitFirstFrame(vector<Block>& images, LayerParameter& layer){
	vector<Rect> ada_rect;
	if (init_type_ == MilTrackParameter_InitType_ADABOOST){
		bool retval = adaboost->RunForOneRect(images, layer, ada_rect);
		if (!retval){
			reinit_ = true;
			LOG(ERROR) << "Adaboost Init Rect Failed.";
			return false;
		}
	}

	for (int i = 0; i < images.size(); ++i){
		switch (init_type_)
		{
		case svaf::MilTrackParameter_InitType_MOUSE:
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
			if (i >= init_rects_.size()){
				init_rect_ = init_rects_[0];
			} else{
				init_rect_ = init_rects_[i];
			}
			break;
		case svaf::MilTrackParameter_InitType_AUTORECT:
			init_rect_.width = images[i].image.cols / 12;
			init_rect_.height = images[i].image.rows / 8;
			init_rect_.x = images[i].roi.width / 2 - init_rect_.width / 2;
			init_rect_.y = images[i].roi.height / 2 - init_rect_.width / 2;
			break;
		case svaf::MilTrackParameter_InitType_ADABOOST:
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

	for (int i = 0; i < images.size(); ++i){
		cv::resize(images[i].image, trackers_[i].img, trackers_[i].trsize);
		if (trackers_[i].img.channels() == 3){
			cvtColor(trackers_[i].img, trackers_[i].img, CV_BGR2GRAY);
		}
		__t.StartWatchTimer();
		track_firstframe(trackers_[i].img, trackers_[i].rect, trackers_[i].trparam, trackers_[i].ftrparam);
		__t.ReadWatchTimer("MIL InitTrack Time");
		if (__logt){
			char alicia[3];
			sprintf(alicia, "%d", i);
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
	}

	return true;
}

void MilTrackLayer::InitSetParam(BoostTrack& tracker){
	
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

void MilTrackLayer::ComputeScale(int i){
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

void MilTrackLayer::RecoverScale(vector<Block>& images, vector<Block>& disp){

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
				disp.push_back(Block(images[i].name + " Track", disp_img, __show, __save, __bout));
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
				disp.push_back(Block(images[i].name + " Track", disp_img, __show, __save, __bout));
			}

			images[i].image = images[i].image(Rect(x, y, width, height)).clone();
		}
	}
	
}

}

