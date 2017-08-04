/* SurfDetect
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1. The program use surf to detect feature points
// and generate feature descriptor. You can also directly sample image from video
// or camera, a webcamera is necessary if you run the program on camera mode.
// The program can mosaic two image and draw the match point, you can see usage
// and input the parameters by command line. The surf detect part referenced on
// OpenSurt(https://github.com/amarburg/opencv-ffi-ext/tree/master/ext/opensurf)
// project, and the core of code write by C. The test shows the program is more
// efficiency than OpenSurf. Enjoy it!
// Created at: 26 Aug. 2016, all rights reserved.*/


#include "common.h"

namespace pc{


void computeintegral_channel(const fimg& img, fimg& ii_img){
	ii_img.chns = img.chns;
	ii_img.cols = img.cols;
	ii_img.rows = img.rows;
	ii_img.data[0] = img.data[0];

	float rs = 0.0f;
	for (int j = 0; j < img.cols; j++){
		rs += img.data[j];
		ii_img.data[j] = rs;
	}

	for (int i = 1; i < img.rows; i++){
		rs = 0.0f;
		for (int j = 0; j < img.cols; j++){
			rs += img.data[i * img.cols + j];
			ii_img.data[i * ii_img.cols + j] = rs + ii_img.data[(i - 1) * img.cols + j];
		}
	}
}

void compute_integral(const uimg& img, fimg& ii_img){
	if (img.chns == 1){
		fimg imgf;
		imgf.chns = img.chns;
		imgf.cols = img.cols;
		imgf.rows = img.rows;
		const int length = img.chns * img.cols * img.rows;
		imgf.data = (float*)malloc(img.chns * img.cols * img.rows * sizeof(float));
		for (int i = 0; i < length; ++i){
			imgf.data[i] = img.data[i] / 255.0f;
		}
		computeintegral_channel(imgf, ii_img);
		free(imgf.data);
	}
	else{
		assert(false);
	}
}

float sumRect(const fimg& ii_img, const int row, const int col, 
	const int rows, const int cols){
	const int r1 = std::min(row, ii_img.rows) - 1;
	const int c1 = std::min(col, ii_img.cols) - 1;
	const int r2 = std::min(row + rows, ii_img.rows) - 1;
	const int c2 = std::min(col + cols, ii_img.cols) - 1;

	float value[4] = { 0 };
	if (r1 >= 0 && c1 >= 0) value[0] = imgElem(ii_img, r1, c1);
	if (r1 >= 0 && c2 >= 0) value[1] = imgElem(ii_img, r1, c2);
	if (r2 >= 0 && c1 >= 0) value[2] = imgElem(ii_img, r2, c1);
	if (r2 >= 0 && c2 >= 0) value[3] = imgElem(ii_img, r2, c2);

	return std::max(0.0f, value[0] - value[1] - value[2] + value[3]);
}

}
