/*
 * acfDetect.h
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#ifndef ACFDETECT_H_
#define ACFDETECT_H_

#include <stdint.h>
#include <vector>

#include "chnsPyramid.h"

using namespace std;

namespace pc{

typedef struct _DetectResult
{
	float cs;
	float modelWd;
	float rs;
	float modelHt;
	float hs;
	bool operator > (const _DetectResult& rhs) const
	{
		return hs > rhs.hs;
	}
	int	 scaleindex;
} DetectResult;

class AcfDetector
{
public:
	uint32_t* fids;
	float*    thrs;
	uint32_t* child;
	float*    hs;
	float*    weights;
	uint32_t* depth;
	uint32_t  treeDepth;
	uint32_t  nTrees, nTreeNodes;
	uint32_t  stride;
	int32_t   cascThr;
	int32_t   modelHt, modelWd;
	int32_t   modelPadHt, modelPadWd;

	AcfDetector(char* path);
	AcfDetector(){};
	void Open(char* path);
	~AcfDetector();

	void Detect(Channels& data, int32_t shrink, vector<DetectResult>& result, float epipolarLine = -10);
};

void AcfDetectImg(Mat& img, PyramidOpt& opt, AcfDetector& detector, vector<DetectResult>& result, float nms = 0.65);
void AcfDetectImgScale(Mat& img, PyramidOpt& opt, AcfDetector& detector, vector<DetectResult>& result, int scaleindex, float epipolarLine = -10, float nums = 0.65);

}
#endif /* ACFDETECT_H_ */
