/* BoostTrack
// Copyright 2016 Peng Chao, (mail to:me@p-chao.com | http://p-chao.com). Build
// this work with opencv > 2.3.1, and see useage before run the program to track
// the target by MIL or OBA method. The code referenced Boris's paper and code
// (http://vision.ucsd.edu/~bbabenko), without IPP library and mainly use littal
// C++ properity, core of code write with C. Enjoy it!
// Created at: 15 Jul. 2016, all rights reserved.*/


#pragma once
#include <opencv2\opencv.hpp>
using namespace std;
//using namespace cv;

#define  sign(s)	((s > 0 ) ? 1 : ((s<0) ? -1 : 0))
//#define  round(v)   ((int) (v+0.5))
#define	 esp		1e-45f
#define  espf		1e-9

namespace pc{

typedef unsigned int uint;

typedef struct _Image_uchar{
	uchar*	data;
	int		rows;
	int		cols;
	int		chns;
} uimg;

typedef struct _Image_float{
	float*	data;
	int		rows;
	int		cols;
	int		chns;
} fimg;

typedef struct _Rect{
	int		x;
	int		y;
	int		width;
	int		height;
} Rect;

typedef struct _Haar{
	uint			height;
	uint			width;
	uint			channel;
	vector<float>	weights;
	vector<pc::Rect>	rects;
	vector<float>	rsums;
	double			maxSum;
} Haar;

typedef struct _Weak{
	bool	trained;
	int		index;
	float	lRate;
	Haar*	ftrs;
	
	float	mu0, mu1, sig0, sig1;
	float	q;
	int		s;
	float	n1, n0;
	float	e1, e0;
} Weak;

struct FeatureParam{
	int		width = 40;
	int		height = 40;
	int		minRectNum = 2;
	int		maxRectNum = 6;
	int		numCh = -1;
	int		useChannels[1024];
};

struct TrackParam{
	int		linewidth				= 2;
	uint	negnumtrain				= 65;
	uint	init_negnumtrain		= 65;
	float	posradtrain				= 1.0f;  // MIL 4.0  ADA 1.0
	float	init_postrainrad		= 3.0f;
	uint	posmaxtrain				= 100000;
	bool	uselogR					= true;
	uint	srchwinsz				= 25;
	uint	negsamplestrat			= 1;
	uint	count					= 0;
	pc::Rect	curt_rect;

	int				numFeat = 250;
	int				numSel = 50;
	float			lRate = 0.85;
	vector<int>		selectors;
	vector<Haar*>	haars;
	vector<Weak>	weakclf;

	int						numsamples;
	vector<float>			alphas;
	vector<vector<float>>	countFPv, countFNv, countTPv, countTNv;

};

struct Binotrack_param{
	bool tss = false;
	bool sync = false;
	bool pool = false;
	bool mixfeat = false;
	bool showprob = false;
};

int		randint(const int, const int);
float	randfloat();
void	_StartWatchTimer();
void	_ReadWatchTimer();
vector<float> vecsqr(vector<float>);
vector<float> vecsub(vector<float>, const float);
double	vecvar(vector<float>);
double	vecmean(vector<float>&);
double	sigmoid(double);
int		vecmaxindex(vector<float>&);
void	vecminmax(vector<float>&, float&, float&);
cv::Mat convert2img(vector<float>&, vector<pc::Rect>&, int, int, float, float);
void	vecmaxindex_sync(pc::Rect&, pc::Rect&,
			vector<float>&, vector<float>&, vector<pc::Rect>&, vector<pc::Rect>&);

}
