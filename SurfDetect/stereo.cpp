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

#pragma comment(lib, "libeng.lib")
#pragma comment(lib, "libmx.lib")
#pragma comment(lib, "libmat.lib")
#pragma comment(lib, "libmex.lib")

#include <iostream>
#include <vector>
#include "common.h"

using namespace std;

namespace Matlab{
#include <engine.h>
}

namespace pc{


static Matlab::Engine* g_Ep;
extern AlgParam algparam;

void OpenMatlab(){
	if (!(g_Ep = Matlab::engOpen(nullptr))){
		std::cout << "engOpen failed" << std::endl;
		return;
	}
	Matlab::engSetVisible(g_Ep, algparam.showprocess);
	Matlab::engEvalString(g_Ep, "cd F:\\MATLAB\\calibration");
	Matlab::engEvalString(g_Ep, "addpath(\'D:\\Program Files\\MATLAB\\R2016a\\toolbox\\calib')");
	Matlab::engEvalString(g_Ep, "clear");
	Matlab::engEvalString(g_Ep, "load('Calib_Results_stereo.mat')");
}

void CloseMatlab(){
	Matlab::engClose(g_Ep);
}

vector<cv::Point3d> stereoTriangulation(vector<pair<Ipoint, Ipoint>>& matches){
	OpenMatlab();
	Matlab::mxArray* leftPoint = Matlab::mxCreateDoubleMatrix(2, matches.size(), Matlab::mxREAL);
	Matlab::mxArray* rightPoint = Matlab::mxCreateDoubleMatrix(2, matches.size(), Matlab::mxREAL);
	for (int i = 0; i < matches.size(); ++i){
		Matlab::mxGetPr(leftPoint)[2 * i] = matches[i].first.x;
		Matlab::mxGetPr(leftPoint)[2 * i + 1] = matches[i].first.y;
		Matlab::mxGetPr(rightPoint)[2 * i] = matches[i].second.x;
		Matlab::mxGetPr(rightPoint)[2 * i + 1] = matches[i].second.y;
	}
	Matlab::engPutVariable(g_Ep, "xL", leftPoint);
	Matlab::engPutVariable(g_Ep, "xR", rightPoint);
	Matlab::engEvalString(g_Ep, "[XL, XR] = stereo_triangulation(xL, xR, om, T, fc_left, cc_left, kc_left, alpha_c_left, fc_right, cc_right, kc_right, alpha_c_right); ");
	Matlab::mxArray* xL = Matlab::engGetVariable(g_Ep, "XL");
	Matlab::mxArray* xR = Matlab::engGetVariable(g_Ep, "XR");

	vector<cv::Point3d> pointsL(matches.size()), pointsR(matches.size());
	for (int i = 0; i < matches.size(); ++i){
		pointsL[i].x = Matlab::mxGetPr(xL)[3 * i + 0];
		pointsL[i].y = Matlab::mxGetPr(xL)[3 * i + 1];
		pointsL[i].z = Matlab::mxGetPr(xL)[3 * i + 2];
		pointsR[i].x = Matlab::mxGetPr(xR)[3 * i + 0];
		pointsR[i].y = Matlab::mxGetPr(xR)[3 * i + 1];
		pointsR[i].z = Matlab::mxGetPr(xR)[3 * i + 2];
	}

	Matlab::mxDestroyArray(leftPoint);
	Matlab::mxDestroyArray(rightPoint);
	CloseMatlab();
	return pointsL;
}

}
