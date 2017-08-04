/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1, and see useage before run the program to track
// the target by MIL or OBA method. The code referenced Boris's paper and code
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#pragma once
#include "common.h"

namespace pc{

void compute_integral(uimg&, fimg&);
//vector<Haar*> generatefeature(int);
vector<Haar*> generatefeature(int, FeatureParam&);
vector<Haar*> generatefeature_mc(int, FeatureParam&, int);
float computefeature(uimg, fimg, Haar*, pc::Rect);
float computefeature_mc(vector<uimg>&, vector<fimg>&, Haar*, pc::Rect);
vector<vector<float>> computefeature_set_mc(vector<uimg>&, vector<fimg>&, vector<pc::Rect>&, vector<Haar*>&);
vector<vector<float>> computefeature_set(uimg, fimg, vector<pc::Rect>&, vector<Haar*>&);

}
