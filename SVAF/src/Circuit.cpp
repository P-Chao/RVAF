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

Circuit::Circuit(SvafTask& svafTask, bool gui_mode) : 
	layers_(svafTask), 
	guiMode_(gui_mode),
	useMapping_(gui_mode),
	linklist_(NULL), 
	svaf_(svafTask),
	id_(0){
	Layer::figures = &sout_;
	Layer::id = &id_;
	Layer::task_type = SvafApp::NONE;
	Layer::gui_mode = guiMode_;
	Layer::pCir = this;
	pause_ms_ = svafTask.pause();
	world_.rectified = false;
	if (useMapping_){
		c_mutex_ = OpenEvent(MUTEX_ALL_ACCESS, false, "SVAF_GUI2ALG_CMD_MUTEX");
		c_fileMapping_ = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, "SVAF_GUI2ALG_CMD");
		c_pMsg_ = (LPTSTR)MapViewOfFile(c_fileMapping_, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		d_mutex_ = CreateEvent(nullptr, false, false, "SVAF_ALG2GUI_DATA_MUTEX");
		d_fileMapping_ = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, "SVAF_ALG2GUI_DATA");
		d_pMsg_ = (LPTSTR)MapViewOfFile(d_fileMapping_, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		i_mutex_ = CreateEvent(nullptr, false, false, "SVAF_ALG2GUI_INFO_MUTEX");
		i_fileMapping_ = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, "SVAF_ALG2GUI_INFO");
		i_pMsg_ = (LPTSTR)MapViewOfFile(i_fileMapping_, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		if (!c_mutex_ || !c_fileMapping_ || !c_pMsg_ || !d_mutex_ || !d_fileMapping_ || !d_pMsg_ || 
			!i_fileMapping_ || !i_pMsg_){
			useMapping_ = false;
		}
	}
	RLOG("SVAF opened.");
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
		if (!UnmapViewOfFile(c_fileMapping_)){}
		CloseHandle(c_fileMapping_);
		CloseHandle(c_mutex_);
		if (!UnmapViewOfFile(d_fileMapping_)){}
		CloseHandle(d_fileMapping_);
		CloseHandle(d_mutex_);
		if (!UnmapViewOfFile(i_fileMapping_)){}
		CloseHandle(i_fileMapping_);
		CloseHandle(i_mutex_);
	}
	RLOG("SVAF closed.");
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
			Layer::task_type = SvafApp::S_SHOW;
			layerinstance = new DataLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_ADABOOST:
			Layer::task_type = SvafApp::S_DETECT;
			layerinstance = new AdaboostLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_MILTRACK:
			Layer::task_type = SvafApp::S_DETECT;
			layerinstance = new MilTrackLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_BITTRACK:
			Layer::task_type = SvafApp::S_DETECT;
			layerinstance = new BinoTrackLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_SIFT_POINT:
			Layer::task_type = SvafApp::S_POINT;
			break;
		case svaf::LayerParameter_LayerType_SURF_POINT:
			Layer::task_type = SvafApp::S_POINT;
			layerinstance = new SurfPointLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_STAR_POINT:
			Layer::task_type = SvafApp::S_POINT;
			break;
		case svaf::LayerParameter_LayerType_BRISK_POINT:
			Layer::task_type = SvafApp::S_POINT;
			break;
		case svaf::LayerParameter_LayerType_FAST_POINT:
			Layer::task_type = SvafApp::S_POINT;
			break;
		case svaf::LayerParameter_LayerType_ORB_POINT:
			Layer::task_type = SvafApp::S_POINT;
			break;
		case svaf::LayerParameter_LayerType_KAZE_POINT:
			Layer::task_type = SvafApp::S_POINT;
			break;
		case svaf::LayerParameter_LayerType_HARRIS_POINT:
			Layer::task_type = SvafApp::S_POINT;
			break;
		case svaf::LayerParameter_LayerType_CV_POINT:
			Layer::task_type = SvafApp::S_POINT;
			layerinstance = new CVPointLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_SIFT_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			break;
		case svaf::LayerParameter_LayerType_SURF_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			layerinstance = new SurfDescriptorLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_STAR_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			break;
		case svaf::LayerParameter_LayerType_BRIEF_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			break;
		case svaf::LayerParameter_LayerType_CV_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			layerinstance = new CVDesciptorLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_BRISK_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			break;
		case svaf::LayerParameter_LayerType_FAST_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			break;
		case svaf::LayerParameter_LayerType_ORB_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			break;
		case svaf::LayerParameter_LayerType_KAZE_DESP:
			Layer::task_type = SvafApp::S_POINTDESP;
			break;
		case svaf::LayerParameter_LayerType_KDTREE_MATCH:
			Layer::task_type = SvafApp::POINT_MATCH;
			break;
		case svaf::LayerParameter_LayerType_EULAR_MATCH:
			Layer::task_type = SvafApp::POINT_MATCH;
			layerinstance = new EularMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_RANSAC:
			Layer::task_type = SvafApp::RANSAC_MATCH;
			layerinstance = new RansacLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_BF_MATCH:
			Layer::task_type = SvafApp::RANSAC_MATCH;
			break;
		case svaf::LayerParameter_LayerType_FLANN_MATCH:
			Layer::task_type = SvafApp::RANSAC_MATCH;
			break;
		case svaf::LayerParameter_LayerType_EC_MATCH:
			Layer::task_type = SvafApp::RANSAC_MATCH;
			layerinstance = new ECMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_CV_MATCH:
			Layer::task_type = SvafApp::RANSAC_MATCH;
			layerinstance = new CVMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_SGM_MATCH:
			Layer::task_type = SvafApp::STEREO_MATCH;
			layerinstance = new SgmMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_EADP_MATCH:
			Layer::task_type = SvafApp::STEREO_MATCH;
			layerinstance = new EadpMatchLayer(layer);
			break;
		case svaf::LayerParameter_LayerType_TRIANG:
			Layer::task_type = SvafApp::PC_TRIANGLE;
			layerinstance = new TriangulationLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_MXMUL:
			Layer::task_type = SvafApp::PC_MULMATRIX;
			layerinstance = new MatrixMulLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_CENTER_POS:
			Layer::task_type = SvafApp::PR_CENTER;
			layerinstance = new CenterPointLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_IA_EST:
			Layer::task_type = SvafApp::PC_REGISTRATION;
			layerinstance = new IAEstimateLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_IAICP_EST:
			Layer::task_type = SvafApp::PC_REGISTRATION;
			layerinstance = new ICPEstimateLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_IANDT_EST:
			Layer::task_type = SvafApp::PC_REGISTRATION;
			layerinstance = new NDTEstimateLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_RECTIFY:
			Layer::task_type = SvafApp::S_RECTIFY;
			layerinstance = new StereoRectifyLayer(layer);
			param = (void*)&world_;
			break;
		case svaf::LayerParameter_LayerType_SUPIX_SEG:
			Layer::task_type = SvafApp::S_SUPIX;
			layerinstance = new SupixSegLayer(layer);
			break;
		default:
			Layer::task_type = SvafApp::NONE;
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
	RLOG("All Layer Builded.");
}

void Circuit::Run(){
	char buf[256] = {0};
	while (layers_.IsBinocular()){
		LOG(INFO) << "#Frame " << id_ << " Begin: ";

		Mat left, right;
		pair<Mat, Mat> matpair = make_pair(left, right);
		layers_ >> matpair;
		if (matpair.first.empty()){
			LOG(WARNING) << "Mat empty, Finish All Process.";
			sprintf(buf, "Frame %d Begin.", id_);
			RLOG(buf);
			break;
		}
		sprintf(buf, "Frame %d Begin.", id_);
		RLOG(buf);
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
			sprintf(buf, "Frame %d Begin.", id_);
			RLOG(buf);
			break;
		}
		sprintf(buf, "Frame %d Begin.", id_);
		RLOG(buf);
		InitStep();
		images_.push_back(Block("left", mat.clone()));
		RunStep();
		if (!Disp()){
			break;
		}
		EndStep();
		
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
	SendData();
	ReciveCmd();
	RLOG("Process Finished.");
}

void Circuit::Analysis(){
	RLOG("Analysis Data.");
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
	LPTSTR p = c_pMsg_;
	int cmd = ((int*)p)[0];
	if (cmd == 1){ // exit();
		LOG(INFO) << "Recived exit command.";
		return false;
	} else if (cmd == 2){
		while (cmd != 3) {
			WaitForSingleObject(c_mutex_, INFINITE);
			cmd = ((int*)p)[0];
		}
	}
	((int*)p)[0] = 0;
	return true;
}

using Bucket = struct{
	char	head[4];
	char	message[10][128];
	int		msgCount;
	int		imgCount;
	int		cols[8];
	int		rows[8];
	int		chns[8];
	int		offs[8];
	int		PointSize[4];
	int		PointChns[4];// xyz(3) or xyzrgb(6)
	int		PointOffs[4];
	int		pclCount;
};

void Circuit::SendData(){

	if (!useMapping_ || !d_pMsg_){
		return;
	}

	LPTSTR p = d_pMsg_;
	char *pBuf = p;
	Bucket *pBucket = (Bucket*)pBuf;
	sprintf(pBucket->head, "pch");
	sprintf(pBucket->message[0], "123");
	pBucket->msgCount = 1;

	int frameCount = 0;
	int pointCount = 0;
	int offset = sizeof(Bucket);
	for (int i = 0; i < disp_.size(); ++i){
		if (disp_[i].isOutput && (!disp_[i].isOutput3DPoint) && (!disp_[i].image.empty()) && frameCount < 8){
			int cols = disp_[i].image.cols;
			int rows = disp_[i].image.rows;
			int chns = disp_[i].image.channels();
			int length = cols * rows * chns;
			memcpy(pBuf + offset, disp_[i].image.data, length);
			pBucket->cols[frameCount] = cols;
			pBucket->rows[frameCount] = rows;
			pBucket->chns[frameCount] = chns;
			pBucket->offs[frameCount] = offset;
			offset += (cols * rows * chns);
			frameCount++;
		}
		if (disp_[i].isOutput && disp_[i].isOutput3DPoint && pointCount < 4){
			int count = disp_[i].point3d.size();
			if (count <= 0){
				continue;
			}
			int chns = (disp_[i].color3d.size() == disp_[i].point3d.size()) ? 6 : 3;
			float *points = (float *)(pBuf + offset);
			if (chns == 3){
				for (int j = 0; j < count; ++j){
					points[j * 3 + 0] = disp_[i].point3d[j].x;
					points[j * 3 + 1] = disp_[i].point3d[j].y;
					points[j * 3 + 2] = disp_[i].point3d[j].z;
				}
			} else{
				for (int j = 0; j < count; ++j){
					points[j * 6 + 0] = disp_[i].point3d[j].x;
					points[j * 6 + 1] = disp_[i].point3d[j].y;
					points[j * 6 + 2] = disp_[i].point3d[j].z;
					points[j * 6 + 3] = disp_[i].color3d[j].r;
					points[j * 6 + 4] = disp_[i].color3d[j].g;
					points[j * 6 + 5] = disp_[i].color3d[j].b;
				}
			}
			memcpy(pBuf + offset, points, count * chns * sizeof(float));
			pBucket->PointSize[pointCount] = count;
			pBucket->PointChns[pointCount] = chns;
			pBucket->PointOffs[pointCount] = offset;
			offset += (count * chns * sizeof(float));
			pointCount++;
		}
	}
	pBucket->imgCount = frameCount;
	pBucket->pclCount = pointCount;
	SetEvent(d_mutex_);
}

void Circuit::RLOG(std::string infoStr){

	if (!useMapping_ || !i_pMsg_){
		return;
	}

	LPTSTR p = i_pMsg_;
	char *pBuf = p;
	memcpy(pBuf, infoStr.data(), infoStr.length());
	pBuf[infoStr.length() + 1] = '\0';
	SetEvent(i_mutex_);
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
