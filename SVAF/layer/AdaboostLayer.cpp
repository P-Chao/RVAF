/*
Stereo Vision Algorithm Framework, Copyright(c) 2016-2018, Peng Chao
目标检测
*/

#include "AdaboostLayer.h"
#include "../../AcfDetect/acfDetect.h"
#include "../../AcfDetect/chnsCompute.h"
#include "../../AcfDetect/chnsPyramid.h"
#include "../../AcfDetect/disp.h"
#include <glog\logging.h>

using namespace pc;

namespace svaf{

// 初始化Adaboost目标检测
AdaboostLayer::AdaboostLayer(LayerParameter& layer) : scaleindex(-1), ksvideo(false), ksframe(false), kseline(false), epoLine(-1), Layer(layer)
{
	string	detectorfile = layer.adaboost_param().detector();
	detector.Open((char*)detectorfile.data()); // 打开分类器文件

	thresh = layer.adaboost_param().thresh(); // 检测阈值
	nms = layer.adaboost_param().nms();	// 非极大抑制因子

	if (layer.adaboost_param().sync_frame()){ // 对于每一帧保持尺度相同
		ksframe = true;	
	}
	if (layer.adaboost_param().sync_video()){ // 对于真个视频序列保持尺度相同
		ksframe = true;
		ksvideo = true;
	}
	if (layer.adaboost_param().sync_epipolar()){ // 对于左右两图应用极线约束
		ksframe = true;
		kseline = true;
	}
	video_firstframe = true;

	// Acf特征参数
	opt.minDs = Size(60, 60);
	opt.nPerOct = 8;
	opt.nApprox = opt.nPerOct - 1;
	opt.nOctUp = 0;
	opt.smooth = 1;
	opt.pad = Size(4, 4);
	opt.lambdas.push_back(0);
	opt.lambdas.push_back(0.2259f);
	opt.lambdas.push_back(0.2259f);
	opt.chnsOpt.shrink = 4;
	opt.chnsOpt.colorSmooth = 1;
	opt.chnsOpt.gradMagNormRad = 5;
	opt.chnsOpt.gradMagNormConst = 0.005f;
	opt.chnsOpt.gradMagFull = 0;
	opt.chnsOpt.gradHistBinSize = opt.chnsOpt.shrink;
	opt.chnsOpt.gradHistOrients = 6;
	opt.chnsOpt.gradHistSoftBin = 0;
	opt.chnsOpt.gradHistUseHog = 0;
	opt.chnsOpt.gradHistClipHog = 0.2f;

	// 对ROI结果进行修正
	roipad.x = -layer.adaboost_param().pad_rect().left();
	roipad.y = -layer.adaboost_param().pad_rect().top();
	roipad.width = layer.adaboost_param().pad_rect().right() + layer.adaboost_param().pad_rect().left();
	roipad.height = layer.adaboost_param().pad_rect().bottom() + layer.adaboost_param().pad_rect().top();
	xshift = layer.adaboost_param().pad_rect().xshift();

	// 初始化图像金字塔
	pc::ChnsComputeInit();
}

// 析构函数
AdaboostLayer::~AdaboostLayer()
{
}

// 运行算法
bool AdaboostLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	CHECK_GT(images.size(), 0) << "No Image In Block!";
	result_sc.clear();
	result_rect.clear();
	result_sc.resize(images.size());
	result_rect.resize(images.size());

	if (ksframe == false){
		ImageDetect(images, disp); // 进行检测
	} else{
		ScaleDetect(images, disp); // 目标同尺度假设下进行检测
	}
	bool ret = ResultROI(images, disp);

	if (ret){
		for (int i = 0; i < images.size(); ++i){
			images[i].xct = images[i].roi.x + images[i].roi.width / 2.0;
			images[i].yct = images[i].roi.y + images[i].roi.height / 2.0;
		}
		if (images.size() == 2){
			images[1].useroi = true;
			images[0].useroi = true;
		} else{
			images[1].useroi = false;
			images[0].useroi = false;
		}
	}
	

	return ret;
}

// 供MiltrackLayer和BinotrakLayer调用的一次性目标检测函数
bool AdaboostLayer::RunForOneRect(vector<Block>& images, LayerParameter& layer, vector<Rect>& rect){
	CHECK_GT(images.size(), 0) << "No Image In Block!";
	result_sc.clear();
	result_rect.clear();
	result_sc.resize(images.size());
	result_rect.resize(images.size());

	__show = false;
	__save = false;

	vector<Block> disp;

	if (ksframe == false){
		ImageDetect(images, disp);
	}
	else{
		ScaleDetect(images, disp);
	}
	
	// 保存结果矩形的位置
	for (int i = 0; i < images.size(); ++i){
		if (result_rect[i].size() == 0){
			return false;
		} else{
			rect.push_back(result_rect[i][0]);
		}
	}

	return true;
}

// 对于没一副图像单独运行没有相互约束的目标检测
bool AdaboostLayer::ImageDetect(vector<Block>& images, vector<Block>& disp){
	for (int i = 0; i < images.size(); ++i){
		vector<DetectResult> result;
		Mat img = images[i].image.clone();
		__t.StartWatchTimer();
		AcfDetectImg(img, opt, detector, result, nms);
		__t.ReadWatchTimer("Adaboost Image Time");
		if (__logt){
			char alicia[3];
			sprintf(alicia, "%d", i);
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
		SelectROI(images[i], i, result);
		if (result.size() > 0)
			LOG(INFO) << "Box: " << result[0].rs << ", " << result[0].modelHt;
	}
	return true;
}

// 假设目标处于同一尺度下进行检测
bool AdaboostLayer::ScaleDetect(vector<Block>& images, vector<Block>& disp){
	bool flag = true;
	bool ret = true;
	int i = 0;
	// 对于第一幅图像进行处理，得到尺度
	if (video_firstframe){
		video_firstframe = !ksvideo;

		vector<DetectResult> result;
		Mat img = images[0].image.clone();
		__t.StartWatchTimer();
		AcfDetectImg(img, opt, detector, result, nms);
		__t.ReadWatchTimer(__name + "Adaboost Image Init Sync Time");
		if (__logt){
			(*figures)[__name + "0_t"][*id] = (float)__t;
		}

		flag = SelectROI(images[0], i, result, true);
		

		i = 1;
		if (flag){
			epoLine = kseline ? result[0].rs : -1;
			scaleindex = result[0].scaleindex;
			LOG(INFO) << "Box: " << result[0].rs << ", " << result[0].modelHt;
		} else{
			video_firstframe = true;
			flag = false;
			scaleindex = -1;
			epoLine = -1;

			LOG(ERROR) << "\nLoop Cut Short\n";
			return false;
		}
	}

	// 在后续图像序列中按照之前得到的制度进行搜索
	for (; i < images.size(); ++i){
		vector<DetectResult> result;
		Mat img = images[i].image.clone();

		char alicia[3];
		sprintf(alicia, "%d", i);
		if (scaleindex < 0){
			__t.StartWatchTimer();
			AcfDetectImg(img, opt, detector, result, nms);
			__t.ReadWatchTimer("Adaboost Image Time");
		} else{
			__t.StartWatchTimer();
			AcfDetectImgScale(img, opt, detector, result, scaleindex, epoLine, nms);
			__t.ReadWatchTimer("Adaboost Image Time");
		}
		if (__logt){
			(*figures)[__name + alicia + "_t"][*id] = (float)__t;
		}
		if (result.size() > 0){
			LOG(INFO) << "Box: " << result[0].rs << ", " << result[0].modelHt;
			RLOG("adaboost detected object.");
		}
		
		bool isleft = true;
		if (i > 0){
			isleft = false;
		}
		if (!SelectROI(images[i], i, result, isleft)){
			LOG(ERROR) << "\nLoop Cut Short\n";
			return false;
		}
	}

	//if (kseline && flag){
	//	//DCHECK_EQ(disp[0].rects[0].y, disp[1].rects[0].y) << " Epipolar Constrain Failed!";
	//	//DCHECK_EQ(disp[0].rects[0].height, disp[1].rects[0].height) << " Epipolar Constrain Failed!";
	//	if (images[0].roi.y != images[1].roi.y){
	//		LOG(ERROR) << " Epipolar Constrain Failed!";
	//		LOG(ERROR) << "\nLoop Cut Short\n";
	//		return false;
	//	}
	//	CHECK_EQ(images[0].roi.height, images[1].roi.height) << " Epipolar Constrain Failed!";
	//}
	return true;
}

// 从检测结果中选择正常的进行保存，并修正坐标
bool AdaboostLayer::SelectROI(Block& image, int num, vector<DetectResult>& result, bool isleft){
	if (result.size() == 0){
		return false;
	}
	for (int i = 0; i < result.size(); ++i){
		if (result[i].hs < thresh){
			continue;
		}
		
		Rect rect(result[i].cs, result[i].rs, result[i].modelWd, result[i].modelHt);
		rect.x += roipad.x;
		rect.y += roipad.y;
		rect.width += roipad.width;
		rect.height += roipad.height;
		if (isleft){
			rect.x += xshift;
		}else{
			rect.x -= xshift;
		}
		if ((rect.x + rect.width) >= image.image.cols || ((rect.y + rect.height) >= image.image.rows)
			|| rect.x <= 0 || rect.y <= 0){
			continue;
		}
		result_rect[num].push_back(rect);
		result_sc[num].push_back(result[i].hs);
	}
	if (result_rect[num].size() == 0){
		return false;
	}
	return true;
}

// 产生结果ROI区域
bool AdaboostLayer::ResultROI(vector<Block>& images, vector<Block>& disp){
	bool return_flag = true;
	for (int i = 0; i < images.size(); ++i){
		Mat disp_img = images[i].image.clone();
		if (images[i].image.channels() == 1){
			cvtColor(disp_img, disp_img, CV_GRAY2BGR);
		}

		for (int j = 0; j < result_rect[i].size(); ++j){
			char str[20];
			Point focus;
			focus.x = result_rect[i][j].x + result_rect[i][j].width / 2;
			focus.y = result_rect[i][j].y + result_rect[i][j].height / 2;
			rectangle(disp_img, result_rect[i][j], Scalar(255, 255, 255), 2);
			sprintf(str, "%5.2f\n", result_sc[i][j]);
			putText(disp_img, str, focus, FONT_HERSHEY_COMPLEX, 0.7,
				Scalar(255, 255, 255), 2);
		}

		if (task_type == SvafApp::S_DETECT || task_type == SvafApp::B_DETECT || 
			task_type == SvafApp::PC_TRIANGLE || task_type == SvafApp::PC_MULMATRIX || 
			task_type == SvafApp::PC_REGISTRATION || task_type == SvafApp::PR_CENTER){
			__bout = true;
		} else {
			__bout = false;
		}

		if (__show || __save || __bout){
			disp.push_back(Block(images[i].name + " Adaboost", disp_img, __show, __save, __bout));
		}

		if (result_rect[i].size() == 0){
			return_flag = false;
			LOG(ERROR) << "Adaboost Detected No Object.";
			LOG(ERROR) << "\nLoop Cut Short\n";
			continue;
		}
		LOG(INFO) << "Adaboost Detected <" << result_rect[i].size() << "> Objects.";

		images[i].roi.x += result_rect[i][0].x;
		images[i].roi.y += result_rect[i][0].y;
		images[i].roi.width = result_rect[i][0].width;
		images[i].roi.height = result_rect[i][0].height;

		images[i].image = images[i].image(result_rect[i][0]).clone();
	}

	if (kseline && return_flag){
		if (images[0].roi.y != images[1].roi.y){
			DLOG(ERROR) << "Yl: " << images[0].roi.y << " Yr: " << images[1].roi.y;
			LOG(ERROR) << " Epipolar Constrain Failed!";
			LOG(ERROR) << "\nLoop Cut Short\n";
			
			return false;
		}
		CHECK_EQ(images[0].roi.height, images[1].roi.height) << " Epipolar Constrain Failed!";
	}

	return return_flag;
}

}
