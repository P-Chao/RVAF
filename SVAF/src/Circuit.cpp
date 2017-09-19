#include "Circuit.h"
#include "..\layer\Layer.h"
#include "..\layer\DataLayer.h"
#include "..\layer\RansacLayer.h"
#include "..\layer\CVMatchLayer.h"
#include "..\layer\CVPointLayer.h"
#include "..\layer\ECMatchLayer.h"
#include "..\layer\AdaboostLayer.h"
#include "..\layer\MilTrackLayer.h"
#include "..\layer\SgmMatchLayer.h"
#include "..\layer\SupixSegLayer.h"
#include "..\layer\BinoTrackLayer.h"
#include "..\layer\EadpMatchLayer.h"
#include "..\layer\MatrixMulLayer.h"
#include "..\layer\SurfPointLayer.h"
#include "..\layer\EularMatchLayer.h"
#include "..\layer\IAEstimateLayer.h"
#include "..\layer\ICPEstimateLayer.h"
#include "..\layer\NDTEstimateLayer.h"
#include "..\layer\CenterPointLayer.h"
#include "..\layer\CVDesciptorLayer.h"
#include "..\layer\StereoRectifyLayer.h"
#include "..\layer\TriangulationLayer.h"
#include "..\layer\SurfDescriptorLayer.h"

#include <WinBase.h>

namespace svaf{

string Circuit::time_id_ = "";
Figures<float> Circuit::sout_;

string GetTimeString();

Circuit::Circuit(SvafTask& svafTask, bool use_mapping) : 
	layers_(svafTask), 
	useMapping_(use_mapping),
	linklist_(NULL), 
	svaf_(svafTask),
	id_(0){
	Layer::figures = &sout_;
	Layer::id = &id_;
	pause_ms_ = svafTask.pause();
	world_.rectified = false;
	if (useMapping_){
		mutex_ = OpenEvent(MUTEX_ALL_ACCESS, false, "SVAF_GUI2ALG_CMD_MUTEX");
		fileMapping_ = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, "SVAF_GUI2ALG_CMD");
		pMsg_ = (LPTSTR)MapViewOfFile(fileMapping_, FILE_MAP_ALL_ACCESS, 0, 0, 0);
	}
	Build();
	Run();
}

Circuit::~Circuit(){
	Node *q, *p = linklist_;
	while (p){
		LOG(INFO) << "Destroy Layer [" << p->name << "].";
		q = p->next;
		delete p->layer;
		delete p;
		p = q;
	}
	StereoRectifyLayer::ReleaseTable();
	if (useMapping_){
		if (!UnmapViewOfFile(fileMapping_)){}
		CloseHandle(fileMapping_);
		CloseHandle(mutex_);
	}
}

void Circuit::Build(){
	
	CHECK_GT(layers_.Size(), 0) << "Layer Count Error!";
	Node *p = linklist_, *q;
	while (p){
		LOG(INFO) << "Destroy Layer [" << p->name << "].";
		q = p->next;
		delete p->layer;
		delete p;
		p = q;
	}
	linklist_ = NULL;
	for (int i = 0; i < 1 + layers_.Size(); ++i){
		if (layers_[i].name() == layers_[i].bottom()){
			linklist_ = new Node(layers_[i].name());
			LOG(INFO) << linklist_->name << " -> ";
			break;
		}
		CHECK_EQ(i, layers_.Size() - 1) << "Not Found Start Layer!";
	}
	p = linklist_;
	while (true){
		if (layers_[p->name].name() == layers_[p->name].top()){
			LOG(INFO) << "Build All Layers.";
			break;
		}
		CHECK(layers_.IsLayerExit(layers_[p->name].top())) 
			<< layers_[p->name].top() << " is not exit!";
		p->next = new Node(layers_[p->name].top());
		LOG(INFO) << " -> " << p->next->name;
		p = p->next;
	}

	p = linklist_;
	Layer *layerinstance = NULL;
	void * param = NULL;
	while (p){
		auto type = layers_[p->name].type();
		auto layer = layers_[p->name];
		switch (type)
		{
		case svaf::LayerParameter_LayerType_NONE:
		case svaf::LayerParameter_LayerType_IMAGE:
		case svaf::LayerParameter_LayerType_IMAGE_PAIR:
		case svaf::LayerParameter_LayerType_VIDEO:
		case svaf::LayerParameter_LayerType_VIDEO_PAIR:
		case svaf::LayerParameter_LayerType_CAMERA:
		case svaf::LayerParameter_LayerType_CAMERA_PAIR:
		case svaf::LayerParameter_LayerType_DSP:
		case svaf::LayerParameter_LayerType_DSP_PAIR:
		case svaf::LayerParameter_LayerType_KINECT:
		case svaf::LayerParameter_LayerType_IMAGE_FOLDER:
		case svaf::LayerParameter_LayerType_IMAGE_PAIR_FOLDER:
			layerinstance = new DataLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_ADABOOST:
			layerinstance = new AdaboostLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_MILTRACK:
			layerinstance = new MilTrackLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_BITTRACK:
			layerinstance = new BinoTrackLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_SIFT_POINT:
			break;
		case svaf::LayerParameter_LayerType_SURF_POINT:
			layerinstance = new SurfPointLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_STAR_POINT:
			break;
		case svaf::LayerParameter_LayerType_BRISK_POINT:
			break;
		case svaf::LayerParameter_LayerType_FAST_POINT:
			break;
		case svaf::LayerParameter_LayerType_ORB_POINT:
			break;
		case svaf::LayerParameter_LayerType_KAZE_POINT:
			break;
		case svaf::LayerParameter_LayerType_HARRIS_POINT:
			break;
		case svaf::LayerParameter_LayerType_CV_POINT:
			layerinstance = new CVPointLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_SIFT_DESP:
			break;
		case svaf::LayerParameter_LayerType_SURF_DESP:
			layerinstance = new SurfDescriptorLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_STAR_DESP:
			break;
		case svaf::LayerParameter_LayerType_BRIEF_DESP:
			break;
		case svaf::LayerParameter_LayerType_CV_DESP:
			layerinstance = new CVDesciptorLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_BRISK_DESP:
			break;
		case svaf::LayerParameter_LayerType_FAST_DESP:
			break;
		case svaf::LayerParameter_LayerType_ORB_DESP:
			break;
		case svaf::LayerParameter_LayerType_KAZE_DESP:
			break;
		case svaf::LayerParameter_LayerType_KDTREE_MATCH:
			break;
		case svaf::LayerParameter_LayerType_EULAR_MATCH:
			layerinstance = new EularMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_RANSAC:
			layerinstance = new RansacLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_BF_MATCH:
			break;
		case svaf::LayerParameter_LayerType_FLANN_MATCH:
			break;
		case svaf::LayerParameter_LayerType_EC_MATCH:
			layerinstance = new ECMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_CV_MATCH:
			layerinstance = new CVMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_SGM_MATCH:
			layerinstance = new SgmMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_EADP_MATCH:
			layerinstance = new EadpMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_TRIANG:
			layerinstance = new TriangulationLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_MXMUL:
			layerinstance = new MatrixMulLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_CENTER_POS:
			layerinstance = new CenterPointLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_IA_EST:
			layerinstance = new IAEstimateLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_IAICP_EST:
			layerinstance = new ICPEstimateLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_IANDT_EST:
			layerinstance = new NDTEstimateLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_RECTIFY:
			layerinstance = new StereoRectifyLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_SUPIX_SEG:
			layerinstance = new SupixSegLayer(layer);
			break;
		default:
			layerinstance = NULL;
			break;
		}

		CHECK_NOTNULL(layerinstance);
		p->layer = layerinstance;
		p->param = param;
		param = NULL;
		LOG(INFO) << "Layer [" << p->name << "] Builded.";
		
		p = p->next;
	}
}

void Circuit::Run(){

	while (layers_.IsBinocular()){
		LOG(INFO) << "#Frame " << id_ << " Begin: ";

		Mat left, right;
		pair<Mat, Mat> matpair = make_pair(left, right);
		layers_ >> matpair;
		if (matpair.first.empty()){
			LOG(WARNING) << "Mat empty, Finish All Process.";
			break;
		}
		InitStep();
		images_.push_back(Block("left", matpair.first.clone()));
		images_.push_back(Block("right", matpair.second.clone()));
		RunStep();
		if (!Disp()){
			break;
		}
		EndStep();
		if (!ReciveCmd()){
			break;
		}
	}

	while (!layers_.IsBinocular()){
		LOG(INFO) << "#Frame " << id_ << " Begin: ";

		Mat mat;
		layers_ >> mat;
		if (mat.empty()){
			LOG(WARNING) << "Mat empty, Finish All Process.";
			break;
		}
		InitStep();
		images_.push_back(Block("left", mat.clone()));
		RunStep();
		if (!Disp()){
			break;
		}
		EndStep();
		if (!ReciveCmd()){
			break;
		}
	}

	Analysis();
}

void Circuit::RunStep(){
	
	void *param = NULL;
	Node *p = linklist_;
	while (p){
		LayerParameter layer = layers_[p->name];
		param = p->param;
		if (!p->layer->Run(images_, disp_, layer, param)){
			break;
		}
		p = p->next;
	}
	
}

bool Circuit::Disp(){
	
	for (int i = 0; i < disp_.size(); ++i){
		if (disp_[i].isShow){
			imshow(disp_[i].name, disp_[i].image);
		}
		if (disp_[i].isSave){
			imwrite(string("tmp/") + disp_[i].name + " " + Circuit::time_id_ 
				+ ".png", disp_[i].image);
		}
	}
	if (pause_ms_ < 0)
		return true;
	char key = waitKey(pause_ms_);
	switch (key)
	{
	case 'q':
		return false;
		break;
	case 'e':
		exit(-1);
		break;
	case 'p':
		waitKey();
		break;
	case 'r':
		MilTrackLayer::reinit_ = true;
		break;
	default:
		break;
	}
	return true;
}

void Circuit::InitStep(){
	disp_.clear();
	images_.clear();

	char idch[5];
	sprintf(idch, "%04d", id_);
	string timestr = GetTimeString();
	string idstr(idch);
	Circuit::time_id_ = timestr + "_" + idstr;
	sout_.setRow(id_+1);
}

void Circuit::EndStep(){
	id_++;
}

void Circuit::Analysis(){
	string timestr = GetTimeString();
	if (!layers_.images_.empty() && !layers_.IsBinocular()){
		sout_.print2txt_im(string("tmp/A_") + timestr + ".txt", layers_.images_);
	} else if (!layers_.imagepairs_.empty() && layers_.IsBinocular()){
		sout_.print2txt_impair(string("tmp/A_") + timestr + ".txt", layers_.imagepairs_);
	} else{
		sout_.print2txt(string("tmp/A_") + timestr + ".txt");
	}
	LOG(INFO) << "Analysis Data Has Been Saved: \n" << string("tmp/A_") + timestr + ".txt";
	if (id_ < 5){
		sout_.print2scr();
	}
	
}

bool Circuit::ReciveCmd(){
	if (!useMapping_){
		return true;
	}
	LPTSTR p = pMsg_;
	int cmd = ((int*)p)[0];
	if (cmd == 1){ // exit();
		LOG(INFO) << "Recived exit command.";
		return false;
	} else if (cmd == 2){
		while (cmd != 3) {
			WaitForSingleObject(mutex_, INFINITE);
			cmd = ((int*)p)[0];
		}
	}
	((int*)p)[0] = 0;
	return true;
}

string GetTimeString(){
	char filename[16];
	time_t rawtime;
	struct tm * t_tm;
	time(&rawtime);
	t_tm = localtime(&rawtime);
	int it = clock() % 1000;

	sprintf(filename, "%02d%02d%02d%02d%02d%02d%03d",
		t_tm->tm_year - 100, t_tm->tm_mon + 1, t_tm->tm_mday,
		t_tm->tm_hour, t_tm->tm_min, t_tm->tm_sec, it);
	string strFileName(filename);
	return strFileName;
}

}
