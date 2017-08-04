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
#include "integral.h"
using namespace std;

namespace pc{


extern SurfParam suparam;

static void buildResponseLayer(const fimg& ii_img, ResponseLayer& layer){
	const int step = layer.step;
	const int b = (layer.filter - 1) / 2;
	const int l = layer.filter / 3;
	const int w = layer.filter;
	const double inv_area = 1.f / (w * w);

	double Dxx, Dyy, Dxy;
	for (int ar = 0, index = 0; ar < layer.height; ++ar){
		for (int ac = 0; ac < layer.width; ++ac, index++){
			const int r = ar * step;
			const int c = ac * step;
			Dxx = sumRect(ii_img, r - l + 1, c - b, 2 * l - 1, w)
				- sumRect(ii_img, r - l + 1, c - l / 2, 2 * l - 1, l) * 3;
			Dyy = sumRect(ii_img, r - b, c - l + 1, w, 2 * l - 1)
				- sumRect(ii_img, r - l / 2, c - l + 1, l, 2 * l - 1) * 3;
			Dxy = +sumRect(ii_img, r - l, c + 1, l, l)
				+ sumRect(ii_img, r + 1, c - l, l, l)
				- sumRect(ii_img, r - l, c - l, l, l)
				- sumRect(ii_img, r + 1, c + 1, l, l);

			Dxx *= inv_area;
			Dyy *= inv_area;
			Dxy *= inv_area;

			layer.responses[index] = Dxx * Dyy - 0.81f * Dxy * Dxy;
			layer.laplacian[index] = (Dxx + Dyy >= 0 ? 1 : 0);
		}
	}
}

vector<ResponseLayer> buildResponseMap(const fimg& ii_img){
	vector<ResponseLayer> responseMap;
	const int w = ii_img.cols / suparam.stride;
	const int h = ii_img.rows / suparam.stride;
	const int s = suparam.stride;

	int filtersize = 3;
	int scalecoeff = 6;
	int scalefactor = 1;
	
	ResponseLayer layer;
	layer.step = s;
	layer.width = w;
	layer.height = h;

	for (int i = 1; i <= 4; ++i){
		filtersize += scalecoeff;
		layer.filter = filtersize;
		responseMap.push_back(layer);
	}

	for (int i = 1; i < suparam.octaves; ++i){
		scalecoeff *= 2;
		filtersize += scalecoeff;

		scalefactor *= 2;
		layer.step = s * scalefactor;
		layer.width = w / scalefactor;
		layer.height = h / scalefactor;
		layer.filter = filtersize;
		responseMap.push_back(layer);

		filtersize += scalecoeff;
		layer.filter = filtersize;
		responseMap.push_back(layer);
	}

	for (int i = 0; i < responseMap.size(); ++i){
		responseMap[i].laplacian = 
			(uchar*)calloc(responseMap[i].width * responseMap[i].height, sizeof(uchar));
		responseMap[i].responses = 
			(float*)calloc(responseMap[i].width * responseMap[i].height, sizeof(float));
		buildResponseLayer(ii_img, responseMap[i]);
	}

	return responseMap;
}

void releaseResponseMap(vector<ResponseLayer>& responseMap){
	for (int i = 0; i < responseMap.size(); ++i){
		free(responseMap[i].laplacian);
		free(responseMap[i].responses);
	}
}

static inline float getResponse(const ResponseLayer& thislayer, 
	const int top_row, const int top_col, const ResponseLayer& toplayer){
	const int scale = thislayer.width / toplayer.width;
	return layerResponse(thislayer, scale * top_row, scale * top_col);
}

static inline uchar getLaplacian(const ResponseLayer& thislayer, 
	const int top_row, const int top_col, const ResponseLayer& toplayer){
	const int scale = thislayer.width / toplayer.width;
	return layerLaplacian(thislayer, scale * top_row, scale * top_col);
}

static void interpolatateStep(const int r, const int c, 
	const ResponseLayer& t, const ResponseLayer& m, const ResponseLayer& b, 
	double& xi, double& xr, double& xc){
	double dD[3], H[3][3], Hi[3][3];
	dD[0] = (getResponse(m, r, c + 1, t) - getResponse(m, r, c - 1, t)) / 2.0;		// dx
	dD[1] = (getResponse(m, r + 1, c, t) - getResponse(m, r - 1, c, t)) / 2.0;		// dy
	dD[2] = (layerResponse(t, r, c) - getResponse(b, r, c, t)) / 2.0;				// ds

	const double v = getResponse(m, r, c, t);
	H[0][0] = getResponse(m, r, c + 1, t) + getResponse(m, r, c - 1, t) - 2 * v;
	H[1][1] = getResponse(m, r + 1, c, t) + getResponse(m, r - 1, c, t) - 2 * v;
	H[2][2] = layerResponse(t, r, c) + getResponse(b, r, c, t) - 2 * v;
	H[0][1] = (getResponse(m, r + 1, c + 1, t) - getResponse(m, r + 1, c - 1, t)
		- getResponse(m, r - 1, c + 1, t) + getResponse(m, r - 1, c - 1, t)) / 4.0;
	H[0][2] = (layerResponse(t, r, c + 1) - layerResponse(t, r, c - 1)
		- getResponse(b, r, c + 1, t) + getResponse(b, r, c - 1, t)) / 4.0;
	H[1][2] = (layerResponse(t, r + 1, c) - layerResponse(t, r - 1, c)
		- getResponse(b, r + 1, c, t) + getResponse(b, r - 1, c, t)) / 4.0;
	H[1][0] = H[0][1];
	H[2][0] = H[0][2];
	H[2][1] = H[1][2];

	const double factor = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1])
		- H[1][0] * (H[0][1] * H[2][2] - H[0][2] * H[2][1])
		+ H[2][0] * (H[0][1] * H[1][2] - H[0][2] * H[1][1]);
	Hi[0][0] = (H[1][1] * H[2][2] - H[1][2] * H[2][1]) / factor;
	Hi[0][1] = (H[0][2] * H[2][1] - H[0][1] * H[2][2]) / factor;
	Hi[0][2] = (H[0][1] * H[1][2] - H[0][2] * H[1][1]) / factor;
	Hi[1][0] = Hi[0][1]; //Hi[1][0] = (H[1][2] * H[2][0] - H[1][0] * H[2][2]) / factor;
	Hi[1][1] = (H[0][0] * H[2][2] - H[0][2] * H[2][0]) / factor;
	Hi[1][2] = (H[1][0] * H[0][2] - H[1][2] * H[0][0]) / factor;
	Hi[2][0] = Hi[0][2]; //Hi[2][0] = (H[1][0] * H[2][1] - H[1][1] * H[2][0]) / factor;
	Hi[2][1] = Hi[1][2]; //Hi[2][1] = (H[0][1] * H[2][0] - H[0][0] * H[2][1]) / factor;
	Hi[2][2] = (H[0][0] * H[1][1] - H[0][1] * H[1][0]) / factor;

	xc = -Hi[0][0] * dD[0] - Hi[0][1] * dD[1] - Hi[0][2] * dD[2];
	xr = -Hi[1][0] * dD[0] - Hi[1][1] * dD[1] - Hi[1][2] * dD[2];
	xi = -Hi[2][0] * dD[0] - Hi[2][1] * dD[1] - Hi[2][2] * dD[2];
}

void interpolateExtremun(const int r, const int c, const ResponseLayer& t, 
	const ResponseLayer& m, const ResponseLayer& b, vector<Ipoint>& ipts){
	const int filterStep = m.filter - b.filter;
	assert(filterStep > 0 && t.filter - m.filter == m.filter -b.filter);
	Ipoint ipt;
	double xi = 0, xr = 0, xc = 0;
	interpolatateStep(r, c, t, m, b, xi, xr, xc);
	if (fabs(xi) < 0.5f && fabs(xr) < 0.5f && fabs(xc) < 0.5f){
		ipt.x = (c + xc) * t.step;
		ipt.y = (r + xr) * t.step;
		ipt.s = 0.1333f * (m.filter + xi * filterStep);
		ipt.laplacian = getLaplacian(m, r, c, t);
		ipts.push_back(ipt);
	}
}

const bool isExtremum(const int r, const int c, 
	const ResponseLayer& t, const ResponseLayer& m, const ResponseLayer& b){
	const int layerBorder = (t.filter + 1) / (2 * t.step);		// 检查边界和阈值
	if (r <= layerBorder || r >= t.height - layerBorder || 
		c <= layerBorder || c >= t.width - layerBorder){
		return false;
	}
	const float candidate = getResponse(m, r, c, t);				// 计算m在相当于t中(r,c)位置上的响应
	if (candidate < suparam.thresh){
		return false;
	}

	for (int rr = -1; rr <= 1; ++rr){
		for (int cc = -1; cc <= 1; ++cc){
			if (layerResponse(t, r+rr, c+cc) >= candidate || (rr != 0 || cc != 0) && 
				getResponse(m, r+rr, c+cc, t) >= candidate || 
				getResponse(b, r+rr, c+cc, t) >= candidate){
				return false;
			}
		}
	}
	return true;
}

}
