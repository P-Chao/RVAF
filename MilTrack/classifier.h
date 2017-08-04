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

vector<float> online_classifysetf(Weak&, vector<pc::Rect>, vector<vector<float>>, fimg, bool);
vector<float> online_classifysetf_mc(Weak&, vector<pc::Rect>, vector<vector<float>>, vector<fimg>&, bool);
vector<bool> online_classifyset(Weak&, vector<pc::Rect>, vector<vector<float>>, fimg, bool);
vector<bool> online_classifyset_mc(Weak&, vector<pc::Rect>, vector<vector<float>>, vector<fimg>&, bool);
void online_trainupdate(Weak&, int, int, vector<vector<float>>&, vector<vector<float>>&);

}
