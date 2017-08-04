#include "Param.h"
#include <io.h>

using namespace std;

namespace svaf{

void getFiles(string path, vector<string>& files)
{
	//文件句柄  
	intptr_t   hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if (path[path.length() - 1] != '\\' || path[path.length() - 1] != '/'){
		path = path + '/';
	}
	if ((hFile = _findfirst(p.assign(path).append("*").c_str(), &fileinfo)) != -1){
		do{ //如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib &  _A_SUBDIR)){
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append(fileinfo.name), files);
			} else
			{
				//files.push_back(p.assign(path).append(fileinfo.name));
				files.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

Param::Param(SvafTask& svafTask) : index_(-1), frame_(0){
	int size = svafTask.layer_size();
	for (int i = 0; i < size; ++i){
		LayerParameter layer = svafTask.layer(i);
		layers_[i] = layer;
		namelayers_[layer.name()] = layer;
		LOG(INFO) << "Layer " << i << ": " << "name: " << layer.name();
	}
	InitDataSource();
	InitVideoState();
}

Param::~Param()
{
}

void Param::InitDataSource(){
	
	for (int i = 0; i < Size(); ++i){
		auto &layer = layers_[i];

		LayerParameter_LayerType layertype = layer.type();
		switch (layertype)
		{
		case svaf::LayerParameter_LayerType_IMAGE:
			runtype_ = layertype;
			isbinocular_ = false;
			for (int j = 0; j < layer.imagedata_param().name_size(); ++j){
				images_.push_back(layer.imagedata_param().name(j));
				LOG(INFO) << "data_source:\n" << layer.imagedata_param().name(j);
			}
			break;
		case svaf::LayerParameter_LayerType_IMAGE_PAIR:
			runtype_ = layertype;
			isbinocular_ = true;
			for (int j = 0; j < layer.imagepair_param().pair_size(); ++j){
				auto binocular = make_pair(layer.imagepair_param().pair(j).left(),
					layer.imagepair_param().pair(j).right());
				imagepairs_.push_back(binocular);
				LOG(INFO) << "data source:\n" << "left:" << layer.imagepair_param().pair(j).left()
					<< "\nright:" << layer.imagepair_param().pair(j).right();
			}
			break;
		case svaf::LayerParameter_LayerType_VIDEO:
			runtype_ = layertype;
			isbinocular_ = false;
			for (int j = 0; j < layer.videodata_param().name_size(); ++j){
				videos_.push_back(layer.videodata_param().name(j));
			}
			break;
		case svaf::LayerParameter_LayerType_VIDEO_PAIR:
			runtype_ = layertype;
			isbinocular_ = true;
			for (int j = 0; j < layer.videopair_param().pair_size(); ++j){
				auto binocular = make_pair(layer.videopair_param().pair(j).left(),
					layer.videopair_param().pair(j).right());
				videopairs_.push_back(binocular);
			}
			break;
		case svaf::LayerParameter_LayerType_CAMERA:
			runtype_ = layertype;
			isbinocular_ = false;
			camera_[0] = layer.cameradata_param().camere();
			break;
		case svaf::LayerParameter_LayerType_CAMERA_PAIR:
			runtype_ = layertype;
			isbinocular_ = true;
			camera_[0] = layer.camerapair_param().leftcamera();
			camera_[1] = layer.camerapair_param().rightcamera();
			break;
		case svaf::LayerParameter_LayerType_DSP:
			runtype_ = layertype;
			isbinocular_ = false;
			break;
		case svaf::LayerParameter_LayerType_DSP_PAIR:
			runtype_ = layertype;
			isbinocular_ = true;
			break;
		case svaf::LayerParameter_LayerType_KINECT:
			runtype_ = layertype;
			isbinocular_ = false;
			break;
		case svaf::LayerParameter_LayerType_IMAGE_FOLDER:
			runtype_ = layertype;
			isbinocular_ = false;
			for (int j = 0; j < layer.folder_param().name_size(); ++j){
				vector<string> files; files.clear();
				getFiles(layer.folder_param().name(j), files);
				for (int k = 0; k < files.size(); ++k){
					images_.push_back(layer.folder_param().name(j) + files[k]);
				}
			}
			break;
		case svaf::LayerParameter_LayerType_IMAGE_PAIR_FOLDER:
			runtype_ = layertype;
			isbinocular_ = true;
			for (int j = 0; j < layer.pairfolder_param().pair_size(); ++j){
				vector<std::string> lfiles; lfiles.clear();
				vector<std::string> rfiles; rfiles.clear();
				string left_path = layer.pairfolder_param().pair(j).left();
				string right_path = layer.pairfolder_param().pair(j).right();
				cout << left_path;
				getFiles(left_path, lfiles);
				getFiles(right_path, rfiles);
				CHECK_EQ(lfiles.size(), rfiles.size()) << "Two folder don't have same count of image file!";
				sort(lfiles.begin(), lfiles.end());
				sort(rfiles.begin(), rfiles.end());
				for (int k = 0; k < lfiles.size(); ++k){
					CHECK_EQ(lfiles[k], rfiles[k]) << lfiles[k] << " and " << rfiles[k] << " doesn't match!";
					auto pairstr = make_pair(left_path + lfiles[k], right_path + rfiles[k]);
					imagepairs_.push_back(pairstr);
				}
			}
			break;
		default:
			break;
		}
	}
}

void Param::InitVideoState(){
	int videoframecount2;
	switch (runtype_)
	{
	case svaf::LayerParameter_LayerType_VIDEO:
		index_++;
		cap_[0].open(getVideo());
		videoframecount_ = cap_[0].get(CV_CAP_PROP_FRAME_COUNT);
		CHECK(cap_[0].isOpened()) << "video 1 open failed.\t" << getVideo();
		LOG(INFO) << "Video Opened: \n" << getVideo()
				<< "\tFrameCount:" << videoframecount_;
		frame_ = -1;
		break;
	case svaf::LayerParameter_LayerType_VIDEO_PAIR:
		index_++;
		cap_[0].open(getVideoPair().first);
		cap_[1].open(getVideoPair().second);
		videoframecount_ = cap_[0].get(CV_CAP_PROP_FRAME_COUNT);
		videoframecount2 = cap_[1].get(CV_CAP_PROP_FRAME_COUNT);
		CHECK(cap_[0].isOpened()) << "video 1 open failed.\t" << getVideoPair().first;
		CHECK(cap_[1].isOpened()) << "video 2 open failed.\t" << getVideoPair().second;
		LOG(INFO) << "Video Opened: \n" << getVideoPair().first
				<< "\tFrameCount: " << videoframecount_
				<< "\t\tAnd " << getVideoPair().second
				<< "\tFrameCount: " << videoframecount2;
		if (videoframecount_ != videoframecount2){
			videoframecount_ = min(videoframecount_, videoframecount2);
			LOG(ERROR) << "framecount is not equal, take: " << videoframecount_;
		}
		frame_ = -1;
		break;
	case svaf::LayerParameter_LayerType_CAMERA:
		cap_[0].open(camera_[0]);
		CHECK(cap_[0].isOpened()) << "Camera Open Filed: \n" << camera_[0];
		LOG(INFO) << "Camera Opened: \n" << camera_[0];
		break;
	case svaf::LayerParameter_LayerType_CAMERA_PAIR:
		cap_[0].open(camera_[0]);
		cap_[1].open(camera_[1]);
		CHECK(cap_[0].isOpened()) << "camera 1 open failed";
		CHECK(cap_[1].isOpened()) << "camera 2 open failed";
		break;
	case svaf::LayerParameter_LayerType_DSP:
		CHECK(dspcamera.open()) << "Base is not run, please open Base program";
		break;
	case svaf::LayerParameter_LayerType_DSP_PAIR:
		CHECK(dspcamera.open()) << "Base is not run, please open Base program";
		break;
	case svaf::LayerParameter_LayerType_KINECT:
		break;
	default:
		break;
	}
}

int Param::Size(){
	return layers_.size();
}

bool Param::IsBinocular() const{
	return isbinocular_;
}

bool Param::IsLayerExit(string str){
	auto iter = namelayers_.find(str);
	if (iter != namelayers_.end()){
		return true;
	}
	return false;
}

LayerParameter_LayerType Param::getDataSource() const{
	return runtype_;
}

Param& Param::operator>>(Mat& mat){
	Mat tempframe;
	switch (runtype_)
	{
	case svaf::LayerParameter_LayerType_IMAGE_PAIR_FOLDER:
	case svaf::LayerParameter_LayerType_IMAGE:
		index_++;
		if (index_ >= images_.size()){
			mat.release();
			LOG(INFO) << "No Fetch Data.";
		} else{
			mat = imread(getImage());
			LOG(INFO) << "Fetched Image:\n" << getImage();
		}
		break;
	case svaf::LayerParameter_LayerType_VIDEO:
		frame_++;
		if (frame_ < videoframecount_){
			cap_[0] >> mat;
		} else{
			index_++;
			cap_[0].release();
			if (index_ >= videos_.size()){
				mat.release();
			} else{
				cap_[0].open(getVideo());
				videoframecount_ = cap_[0].get(CV_CAP_PROP_FRAME_COUNT);
				CHECK(cap_[0].isOpened()) << "video 1 open failed.\t" << getVideo();
				LOG(INFO) << "Video Opened: \n" << getVideo() 
						<< "\tFrameCount:" << videoframecount_;
				frame_ = 0;
				cap_[0] >> mat;
			}
		}
		if (mat.empty()){
			mat.release();
			LOG(ERROR) << "Exception Frame: " << frame_;
		}
		break;
	case svaf::LayerParameter_LayerType_CAMERA:
		cap_[0] >> mat;
		if (mat.empty()){
			cap_[0] >> mat;
		}
		if (mat.empty()){
			mat.release();
			LOG(ERROR) << "Camera Grab Image Filed: " << camera_[0];
		}
		break;
	case svaf::LayerParameter_LayerType_DSP:
		if (dspcamera.getImage()){
			CHECK_GE(dspcamera.images_.size(), 1) << "dsp no data";
			mat = dspcamera.images_[0].clone();
		} else{
			mat.release();
		}
		break;
	case svaf::LayerParameter_LayerType_KINECT:
		break;
	default:
		mat.release();
		LOG(ERROR) << "Not Run On Binocular!";
		break;
	}
	
	return *this;
}

Param& Param::operator>>(pair<Mat, Mat>& matpair){
	int videoframecount2 = 0;
	switch (runtype_)
	{
	case svaf::LayerParameter_LayerType_IMAGE_PAIR_FOLDER:
	case svaf::LayerParameter_LayerType_IMAGE_PAIR:
		index_++;
		if (index_ >= imagepairs_.size()){
			matpair.first.release();
			matpair.second.release();
			LOG(INFO) << "No Fetch Data.";
		} else{
			matpair.first = imread(getImagePair().first);
			matpair.second = imread(getImagePair().second);
			LOG(INFO) << "Fetched Image Pair:\n"
				<< "left: " << getImagePair().first
				<< "\nright: " << getImagePair().second;
		}
		break;
	case svaf::LayerParameter_LayerType_VIDEO_PAIR:
		frame_++;
		if (frame_ < videoframecount_){
			cap_[0] >> matpair.first;
			cap_[1] >> matpair.second;
		}
		else{
			index_++;
			cap_[0].release();
			cap_[1].release();
			if (index_ >= videos_.size()){
				matpair.first.release();
				matpair.second.release();
			} else{
				cap_[0].open(getVideoPair().first);
				cap_[1].open(getVideoPair().second);
				videoframecount_ = cap_[0].get(CV_CAP_PROP_FRAME_COUNT);
				videoframecount2 = cap_[1].get(CV_CAP_PROP_FRAME_COUNT);
				CHECK(cap_[0].isOpened()) << "video 1 open failed.\t" << getVideoPair().first;
				CHECK(cap_[1].isOpened()) << "video 2 open failed.\t" << getVideoPair().second;
				LOG(INFO) << "Video Opened: \n" << getVideoPair().first
						<< "\tFrameCount: " << videoframecount_
						<< "\t\tAnd " << getVideoPair().second
						<< "\tFrameCount: " << videoframecount2;
				if (videoframecount_ != videoframecount2){
					videoframecount_ = min(videoframecount_, videoframecount2);
					LOG(ERROR) << "framecount is not equal, take: " << videoframecount_;
				}
				frame_ = 0;
				cap_[0] >> matpair.first;
				cap_[1] >> matpair.second;
			}
		}
		if (matpair.first.empty()){
			matpair.first.release();
			matpair.second.release();
			LOG(ERROR) << getVideoPair().first << "Exception Frame: " << frame_;
		}
		if (matpair.second.empty()){
			matpair.first.release();
			matpair.second.release();
			LOG(ERROR) << getVideoPair().second << "Exception Frame: " << frame_;
		}
		break;
	case svaf::LayerParameter_LayerType_CAMERA_PAIR:
		cap_[0] >> matpair.first;
		cap_[1] >> matpair.second;
		if (matpair.first.empty() || matpair.second.empty()){
			cap_[0] >> matpair.first;
			cap_[1] >> matpair.second;
		}
		if (matpair.first.empty() || matpair.second.empty()){
			LOG(ERROR) << "Camera Grab Image Failed: " << camera_[0];
		}
		break;
	case svaf::LayerParameter_LayerType_DSP_PAIR:
		if (dspcamera.getImage()){
			CHECK_GE(dspcamera.images_.size(), 2) << "dsp no data";
			matpair.first = dspcamera.images_[0].clone();
			matpair.second = dspcamera.images_[1].clone();
		} else{
			matpair.first.release();
			matpair.second.release();
		}
		break;
	default:
		matpair.first.release();
		matpair.second.release();
		LOG(ERROR) << "Now Run On Binocular!";
		break;
	}
	return *this;
}

LayerParameter& Param::operator[](int& index){
	return layers_[index];
}

LayerParameter& Param::operator[](string& str){
	return namelayers_[str];
}

string Param::getImage(){
	if (index_ >= images_.size()){
		LOG(ERROR) << "Image total count: " << images_.size() << ". No Enough Image!";
	}
	return images_[index_];
}

pair<string, string> Param::getImagePair(){
	if (index_ >= imagepairs_.size()){
		LOG(ERROR) << "Image total count: " << imagepairs_.size() << ". No Enough Image!";
	}
	return imagepairs_[index_];
}

string Param::getVideo(){
	if (index_ >= videos_.size()){
		LOG(ERROR) << "Video total count: " << videos_.size() << ". No Enough Video!";
	}
	return videos_[index_];
}

pair<string, string> Param::getVideoPair(){
	if (index_ >= videopairs_.size()){
		LOG(ERROR) << "Video total count: " << videopairs_.size() << ". No Enough Video!";
	}
	return videopairs_[index_];
}

int Param::getImageCount(){
	if (imagepairs_.size() == 0){
		return images_.size();
	}
	return imagepairs_.size();
}

int Param::getVideoCount(){
	if (videopairs_.size() == 0){
		return videos_.size();
	}
	return videopairs_.size();
}

unsigned int Param::getCurrentFrameCount(){
	return index_;
}

}

