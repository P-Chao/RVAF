#include "StereoRectifyLayer.h"
#ifdef _OPENMP
#include <omp.h>
#endif

namespace svaf{

void *StereoRectifyLayer::pTable[14][2] = { NULL };
uint StereoRectifyLayer::l_length, StereoRectifyLayer::r_length;
uint StereoRectifyLayer::cols, StereoRectifyLayer::rows;



void StereoRectifyLayer::ReleaseTable(){
	for (int i = 0; i < 14; ++i){
		if (pTable[i][0]){
			delete pTable[i][0];
			pTable[i][0] = NULL;
		}
		if (pTable[i][1]){
			delete pTable[i][1];
			pTable[i][1] = NULL;
		}
	}
}

static void index3channal(uint* output, uint* input, uint length){
	for (int i = 0; i < length; ++i){
		output[3 * i] = 3 * input[i];
		output[3 * i + 1] = 3 * input[i] + 1;
		output[3 * i + 2] = 3 * input[i] + 2;
	}
}

void StereoRectifyLayer::ReadTable(const string& filename){
	ReleaseTable();
	FILE *fp = fopen(filename.c_str(), "rb+");
	if (fp == NULL){
		LOG(FATAL) << filename << " Open Failed!";
	}
	LOG(INFO) << "Rectify File: " << filename << " Opened.";

	uint head[4];
	fread(&head, 4, 4, fp);
	cols = head[0];
	rows = head[1];
	l_length = head[2];
	r_length = head[3];
	for (int i = 0; i < 4; ++i){
		pTable[i][0] = new float[l_length];
		pTable[i][1] = new float[r_length];
	}
	for (int i = 4; i < 9; ++i){
		pTable[i][0] = new uint[l_length];
		pTable[i][1] = new uint[r_length];
	}
	for (int i = 9; i < 14; ++i){
		pTable[i][0] = new uint[3*l_length];
		pTable[i][1] = new uint[3*r_length];
	}

	for (int i = 0; i < 9; ++i){
		fread(pTable[i][0], 4, l_length, fp);
	}
	for (int i = 0; i < 9; ++i){
		fread(pTable[i][1], 4, r_length, fp);
	}
	uint check;
	fread(&check, 4, 1, fp);
	if (check != 5 + 9 * (l_length + r_length)){
		LOG(FATAL) << check << " Check End Failed!";
	}
	fclose(fp);
	for (int i = 9; i < 14; ++i){
		index3channal((uint*)pTable[i][0], (uint*)pTable[i-5][0], l_length);
		index3channal((uint*)pTable[i][1], (uint*)pTable[i-5][1], r_length);
	}
	LOG(INFO) << "Rectify Tabel Has Been Created.";
}

StereoRectifyLayer::StereoRectifyLayer(LayerParameter& layer) : Layer(layer)
{
	ReadTable(layer.rectify_param().filename());
}


StereoRectifyLayer::~StereoRectifyLayer()
{
}

bool StereoRectifyLayer::Run(vector<Block>& images, vector<Block>& disp, LayerParameter& layer, void* param){
	if (pTable[0][0] == NULL){
		ReadTable(layer.rectify_param().filename());
	}

	CHECK_GE(images.size(), 2) << "Need Image Pair(" << images.size() << ")";
	CHECK_EQ(images[0].image.cols, cols) << "Dimention Not Match!";
	CHECK_EQ(images[0].image.rows, rows) << "Dimention Not Match!";
	CHECK_EQ(images[1].image.cols, cols) << "Dimention Not Match!";
	CHECK_EQ(images[1].image.rows, rows) << "Dimention Not Match!";
	Mat left_image_rectify(rows, cols, images[0].image.type());
	Mat right_image_rectify(rows, cols, images[1].image.type());

	const float * const a1_left = (float *)pTable[0][0];
	const float * const a2_left = (float *)pTable[1][0];
	const float * const a3_left = (float *)pTable[2][0];
	const float * const a4_left = (float *)pTable[3][0];
	const float * const a1_right = (float *)pTable[0][1];
	const float * const a2_right = (float *)pTable[1][1];
	const float * const a3_right = (float *)pTable[2][1];
	const float * const a4_right = (float *)pTable[3][1];

	const uint * const ind_1_left = (uint *)pTable[4][0];
	const uint * const ind_2_left = (uint *)pTable[5][0];
	const uint * const ind_3_left = (uint *)pTable[6][0];
	const uint * const ind_4_left = (uint *)pTable[7][0];
	const uint * const ind_new_left = (uint *)pTable[8][0];
	const uint * const ind_1_left_3c = (uint *)pTable[9][0];
	const uint * const ind_2_left_3c = (uint *)pTable[10][0];
	const uint * const ind_3_left_3c = (uint *)pTable[11][0];
	const uint * const ind_4_left_3c = (uint *)pTable[12][0];
	const uint * const ind_new_left_3c = (uint *)pTable[13][0];

	const uint * const ind_1_right = (uint *)pTable[4][1];
	const uint * const ind_2_right = (uint *)pTable[5][1];
	const uint * const ind_3_right = (uint *)pTable[6][1];
	const uint * const ind_4_right = (uint *)pTable[7][1];
	const uint * const ind_new_right = (uint *)pTable[8][1];
	const uint * const ind_1_right_3c = (uint *)pTable[9][1];
	const uint * const ind_2_right_3c = (uint *)pTable[10][1];
	const uint * const ind_3_right_3c = (uint *)pTable[11][1];
	const uint * const ind_4_right_3c = (uint *)pTable[12][1];
	const uint * const ind_new_right_3c = (uint *)pTable[13][1];

	Mat& left_image = images[0].image;
	Mat& right_image = images[1].image;
	const uint left = l_length;
	const uint right = r_length;
	const char lchannels = images[0].image.channels();
	const char rchannels = images[1].image.channels();

	uchar *const l_rect = left_image_rectify.data;
	uchar *const r_rect = right_image_rectify.data;
	const uchar *const l_data = left_image.data;
	const uchar *const r_data = right_image.data;
	
	// left
	__t.StartWatchTimer();
	if (lchannels == 3){
		const unsigned int left_3c = left * 3;
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < left_3c; ++i){
			const unsigned int j = i / 3;
			*(l_rect + ind_new_left_3c[i]) = 0.5 +
				a1_left[j] * *(l_data + ind_1_left_3c[i]) + a2_left[j] * *(l_data + ind_2_left_3c[i])
				+ a3_left[j] * *(l_data + ind_3_left_3c[i]) + a4_left[j] * *(l_data + ind_4_left_3c[i]);
		}
	}
	else if (lchannels == 1){
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < left; ++i){
			*(l_rect + ind_new_left[i]) = 0.5 +
				a1_left[i] * *(l_data + ind_1_left[i]) + a2_left[i] * *(l_data + ind_2_left[i])
				+ a3_left[i] * *(l_data + ind_3_left[i]) + a4_left[i] * *(l_data + ind_4_left[i]);
		}
	}
	else if (lchannels > 1){
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < left; ++i){
			for (int c = 0; c < lchannels; ++c){
				unsigned int index = i*lchannels + c;
				*(l_rect + ind_new_left[index]) = 0.5 +
					a1_left[i] * *(l_data + ind_1_left[index]) + a2_left[i] * *(l_data + ind_2_left[index])
					+ a3_left[i] * *(l_data + ind_3_left[index]) + a4_left[i] * *(l_data + ind_4_left[index]);
			}
		}
	}
	__t.ReadWatchTimer("Rectify Binocular Time");
	if (__logt){
		(*figures)[__name + "l_t"][*id] = (float)__t;
	}
	

	// right
	__t.StartWatchTimer();
	if (rchannels == 3){
		const unsigned int right_3c = right * 3;
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < right * 3; ++i){
			const unsigned int j = i / 3;
			*(r_rect + ind_new_right_3c[i]) = 0.5 +
				a1_right[j] * *(r_data + ind_1_right_3c[i]) + a2_right[j] * *(r_data + ind_2_right_3c[i])
				+ a3_right[j] * *(r_data + ind_3_right_3c[i]) + a4_right[j] * *(r_data + ind_4_right_3c[i]);
		}
	}
	else if (rchannels == 1){
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < right; ++i){
			*(r_rect + ind_new_left[i]) = 0.5 +
				a1_right[i] * *(r_data + ind_1_left[i]) + a2_right[i] * *(r_data + ind_2_right[i])
				+ a3_right[i] * *(r_data + ind_3_left[i]) + a4_right[i] * *(r_data + ind_4_right[i]);
		}
	}
	else if (rchannels > 1){
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < right; ++i){
			for (int c = 0; c < rchannels; ++c){
				const unsigned int index = i*rchannels + c;
				*(r_rect + ind_new_right[index]) = 0.5 +
					a1_right[i] * *(r_data + ind_1_right[index]) + a2_right[i] * *(r_data + ind_2_right[index])
					+ a3_right[i] * *(r_data + ind_3_right[index]) + a4_right[i] * *(r_data + ind_4_right[index]);
			}
		}
	}
	__t.ReadWatchTimer("Rectify Binocular Time");
	if (__logt){
		(*figures)[__name + "r_t"][*id] = (float)__t;
	}


	cv::swap(left_image, left_image_rectify);
	cv::swap(right_image, right_image_rectify);

	auto pWorld_ = (World *)param;
	pWorld_->rectified = true;
	LOG(INFO) << "Rectified two image.";

	if (task_type == SvafApp::S_RECTIFY || task_type == SvafApp::B_RECTIFY){
		__bout = true;
	} else {
		__bout = false;
	}

	if (__show || __save){
		disp.push_back(Block(images[0].name + "rectified", images[0].image, __show, __save, __bout));
		disp.push_back(Block(images[1].name + "rectified", images[1].image, __show, __save, __bout));
	}

	return true;
}

}
