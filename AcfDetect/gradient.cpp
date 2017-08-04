/*
 * gradient.h
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#include "wrappers.hpp"

#include <math.h>
#include <vector>

#include <opencv2/opencv.hpp>

#include "convTri.h"
#include "gradient.h"
#include "disp.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace cv;

namespace pc{

// build lookup table a[] s.t. a[x*n]~=acos(x) for x in [-1,1]
float* acosTable()
{
  const int n=10000, b=100; int i;
  static float a[n*2+b*2];
  static bool init = false;
  float *a1=a+n+b;
  if(init) return a1;
  for(i=-n-b; i<-n; i++)   a1[i] = M_PI;
  for(i=-n; i<n; i++)      a1[i] = float(acos(i/float(n)));
  for(i=n; i<n+b; i++)     a1[i] = 0;
  for(i=-n-b; i<n/10; i++) if( a1[i] > M_PI - 1e-6f ) a1[i] = M_PI - 1e-6f;
  init = true;
  return a1;
}

// Compute x and y gradients for just one row
void Grad1(float* row, float* gradX, float* gradY, int32_t height, int32_t width, int32_t y)
{
	float* pixPre, *pixNext;
	float r;
	// Compute row for gradient y
	pixPre = row - width;
	pixNext = row + width;
	r = 0.5f;
	if(y == 0)
	{
		r = 1.0f;
		pixPre += width;
	}
	else if(y == height - 1)
	{
		r = 1.0f;
		pixNext -= width;
	}

#if(USE_SIMD == 1)
	int32_t width1_4 = width / 4;
	__m128 _r1 = _mm_set1_ps(r);

	for(int32_t x = 0; x < width1_4; x++)
	{

		__asm__(
				"MOVAPS %1, %%xmm0;"				//xmm0 = pixNext
				"MOVAPS %2, %%xmm1;"				//xmm1 = pixPre
				"SUBPS %%xmm1, %%xmm0;" 			//xmm0 = pixNext - pixPre
				"MULPS %3, %%xmm0;"                 //xmm0 = (pixNext - pixPre)*r
				"MOVAPS %%xmm0, %0"
				: "=m"(gradY[x*4])
				: "m"(pixNext[x*4]), "m"(pixPre[x*4]), "m"(_r1)
				: "xmm0", "xmm1");

	}
#else
	for(int32_t x = 0; x < width; x++)
	{
		*gradY++ = (*pixNext++ - *pixPre++) * r;
	}

#endif

	// Compute row of gradient x
	pixPre = row;
	pixNext = row + 1;
	*gradX++ = (*pixNext++ - *pixPre++);
	pixPre--;

#if(USE_SIMD == 1)
	int32_t width2_4 = width / 4;
	r = 0.5f;
	__m128 _r2 =  _mm_set1_ps(r);

	for(int32_t x = 0; x < (width2_4-1 ); x ++, gradX += 4, pixNext += 4, pixPre += 4)
	{
		__asm__(
				"MOVUPS %1, %%xmm0;"				//xmm0 = pixNext
				"MOVUPS %2, %%xmm1;"				//xmm1 = pixPre
				"SUBPS %%xmm1, %%xmm0;"				//xmm0 = pixNext - pixPre
				"MULPS %3, %%xmm0;"					//xmm0 = (pixNext - pixPre)*r
				"MOVUPS %%xmm0, %0"
				: "=m"(gradX[0])
				: "m"(pixNext[0]), "m"(pixPre[0]), "m"(_r2)
				: "xmm0", "xmm1");
	}

	*gradX++ = (*pixNext++ - *pixPre++) * 0.5f;
	*gradX++ = (*pixNext++ - *pixPre++)*0.5f;
	pixNext--;
	*gradX = (*pixNext - *pixPre);
#else
	for(int32_t x = 1; x < width-1; x++)
	{
		*gradX++ = (*pixNext++ - *pixPre++) * 0.5f;
	}
	pixNext--;
	*gradX = (*pixNext - *pixPre);
#endif
}

void GradientMag(Mat& src, Mat& mag, Mat& ori, int32_t channel, int32_t normRad, float normConst, int32_t full)
{
	Mat smoothMag;
	int32_t width = src.size().width;
	int32_t height = src.size().height;
	int32_t pixNum = width * height;

	float* gradX = (float*)alMalloc(width * sizeof(float), 16);
	float* gradY = (float*)alMalloc(width * sizeof(float), 16);
	float *acost = acosTable(), acMult = 10000.0f;

	smoothMag.create(src.size(), CV_32FC1);
	mag.create(src.size(), CV_32FC1);
	ori.create(src.size(), CV_32FC1);

	assert(width % 4 == 0);

	// Compute gradient magnitude and orientation for each row
	for(int32_t y = 0; y < height; y++)
	{
		for(int32_t c = 0; c < 1; c++)
		{
			Grad1((float*)&src.data[y * width * 4], gradX, gradY, height, width, y);
#if(USE_SIMD == 1)
			int32_t width_4 = width / 4;
			float minConst[4]    __attribute__((aligned(16))) = {1e10f, 1e10f, 1e10f, 1e10f};
			float mzeroConst[4]  __attribute__((aligned(16))) = {-0.f, -0.f, -0.f, -0.f};
			float acMultConst[4] __attribute__((aligned(16))) = {acMult, acMult, acMult, acMult};
			for(int32_t x = 0; x < width_4; x++)
			{
				__asm__(
					"MOVAPS %2, %%xmm0;"				//xmm0 = gradX
					"MOVAPS %3, %%xmm1;"				//xmm1 = gradY
					"MOVAPS %%xmm0, %%xmm3;"
					"MOVAPS %%xmm1, %%xmm5;"
					"MULPS %%xmm3, %%xmm3;"	//xmm3 = gradX * gradX
					"MULPS %%xmm5, %%xmm5;"	//xmm5 = gradY * gradY
					"ADDPS %%xmm3, %%xmm5;"				//xmm5 = gradX *gradX + gradY * gradY
					"RSQRTPS %%xmm5, %%xmm5;"			//xmm5 = 1 / sqrt(gradX *gradX + gradY * gradY)
					"MOVAPS %4, %%xmm4;"				//TODO: Optimized it
					"MINPS %%xmm4, %%xmm5;"				//xmm5 = min(xmm5, 1e10f)
					"RCPPS %%xmm5, %%xmm2;"				//xmm2 = sqrt(gradX *gradX + gradY * gradY)
					"MOVAPS %%xmm2, %0;"				//mag.data = xmm2
					"MULPS %%xmm0, %%xmm5;"				//xmm5 = gradX / sqrt(gradX *gradX + gradY * gradY)
					"MOVAPS %5, %%xmm0;"				//xmm0 = acMult
					"MULPS %%xmm0, %%xmm5;"				//xmm5 = xmm5 * acMult
					"MOVAPS %6, %%xmm2;"				//xmm2 = -0.f
					"ANDPS %%xmm1, %%xmm2;"				//xmm2 = xmm2 & gradY
					"XORPS %%xmm2, %%xmm5;"				//xmm5 = xmm2 ^ xmm5
					"MOVAPS %%xmm5, %1"					//gradX = xmm5;
					: "=m"(mag.data[(y*width + x*4) * 4]), "=m"(gradX[x*4])
					: "m"(gradX[x*4]), "m"(gradY[x*4]), "m"(minConst[0]), "m"(acMultConst[0]), "m"(mzeroConst[0])
					: "xmm0", "xmm1", "xmm2", "xmm3", "xmm4", "xmm5");
			}
			for(int32_t x = 0; x < width; x++)
			{
				// compute and store gradient orientation
				*(float*)&ori.data[(y * width + x) * 4] = acost[(int32_t)gradX[x]];

			}
#else
			for(int32_t x = 0; x < width; x++)
			{
				// compute gradient mangitude and normalize gradient x
				float rcpSqrt;
				*(float*)&mag.data[(y * width + x) * 4] = gradX[x] * gradX[x] + gradY[x] * gradY[x];
				rcpSqrt = min(1.0f / sqrt(*(float*)&mag.data[(y * width + x) * 4]), 1e10f);
				*(float*)&mag.data[(y * width + x) * 4] = 1.0f / rcpSqrt;
				gradX[x] *= rcpSqrt;
				if(gradY[x] < 0.f)
				{
					gradX[x] = -gradX[x];
				}

				// compute and store gradient orientation
				*(float*)&ori.data[(y * width + x) * 4] = acost[(int32_t)(gradX[x] * acMult)];
			}
#endif
			if(full)
			{
				for(int32_t x = 0; x < width; x++)
				{
					*(float*)&ori.data[(y * width + x) * 4] += (gradY[x] < 0) * M_PI;
				}
			}
		}
	}

	if(normRad != 0)
	{
#if(USE_SIMD == 1)
		ConvTri((float*)mag.data, (float*)smoothMag.data, mag.rows, mag.cols, 1, normRad, 1);
//		ConvTrix(mag, smoothMag, normRad);
//		ConvConst(mag, smoothMag, normRad);
		for(int32_t i = 0; i < pixNum / 4; i++)
		{
			__asm__(
				"MOVAPS %1, %%xmm0;"
				"MOVAPS %2, %%xmm1;"
				"MOVSS %3, %%xmm2;"
				"SHUFPS $0x0, %%xmm2, %%xmm2;"
				"ADDPS %%xmm2, %%xmm1;"
				"DIVPS %%xmm1, %%xmm0;"
				"MOVAPS %%xmm0, %0"
				: "=m"(mag.data[i * 16])
				: "m"(mag.data[i * 16]), "m"(smoothMag.data[i * 16]), "m"(normConst)
				: "xmm0", "xmm1", "xmm2");

		}
#else
		ConvConst(mag, smoothMag, normRad);
		for(int32_t i = 0; i < pixNum; i++)
		{
			*(float*)&mag.data[i * 4] /= (*(float*)&smoothMag.data[i * 4] + normConst);
		}
#endif
	}



	alFree(gradX);
	alFree(gradY);
}

void GradQuantize(float* ori, float* mag, int32_t *O0, int32_t *O1, float *M0, float* M1,
		          int32_t nb, int32_t n, float norm, int32_t nOrients, int32_t full, int32_t interpolate)
{
	const float oMult = (float)nOrients / (full?2*M_PI:M_PI);
	const int32_t oMax = nOrients * nb;
	float o, od, m;
	int32_t i, o0, o1;


#if(USE_SIMD == 1)
	const __m128 _norm=_mm_set1_ps(norm), _oMult=_mm_set1_ps(oMult), _nbf=_mm_set1_ps((float)nb);
	const __m128i _oMax=_mm_set1_epi32(oMax), _nb=_mm_set1_epi32(nb);
	__m128i _o0, _o1, *_O0, *_O1; __m128 _o, _od, _m, *_M0, *_M1;
	_O0=(__m128i*) O0; _O1=(__m128i*) O1; _M0=(__m128*) M0; _M1=(__m128*) M1;

	if( interpolate ) for( i=0; i<=n-4; i+=4) {
//		__asm__(
//				"MOVUPS %4, %%xmm0;"        	//xmm0 = ori[i]
//				"MOVAPS %9, %%xmm1;"			//xmm1 = _oMult
//				"MULPS %%xmm0, %%xmm1;"     	//_o = xmm1 = ori[i] * _oMult
//				"CVTTPS2DQ %%xmm1, %%xmm2;" 	//_o0= xmm2 = (int) xmm1
//				"CVTDQ2PS %%xmm2, %%xmm2;"  	//_o0= xmm2 = (float)xmm2
//				"SUBPS %%xmm2, %%xmm1;" 		//_od = xmm1 = xmm1 - xmm2 = _o - _o0
//				"MULPS %10, %%xmm2;"         	//_o0 = xmm2 = xmm2 * _nbf= _o0 *_nbf
//				"CVTTPS2DQ %%xmm2, %%xmm2;"  	//_o0 = xmm2 = (int) xmm2 = (int) _o0
//				"MOVAPS %%xmm2, %%xmm3;"        //xmm3 = xmm2 =  _o0
//				"PCMPGTD %7, %%xmm2;"           //max(xmm2, oMax )->xmm2
//				"PAND %%xmm3, %%xmm2;"          //AND(xmm2,xmm3) = AND(xmm2,_o0) ->xmm2
//				"MOVAPS %%xmm2, %0;"            //*_O0++ = xmm3
//
//				"ADDPS %8, %%xmm2;"				//_o1 = xmm4 = xmm4 * _nb
//				"MOVAPS %%xmm2, %%xmm4;"        //
//				"PCMPGTD %7, %%xmm2;"           //xmm4 oMax
//				"PAND %%xmm2, %%xmm4;"
//				"MOVAPS %%xmm4, %1;"			//*_O1++ = xmm5
//
//				"MOVUPS %5, %%xmm5;"
//				"MOVAPS %6, %%xmm6;"
//				"MULPS %%xmm5, %%xmm6;"			//_m = xmm7 = mag[i]*_norm
//				"MULPS %%xmm6, %%xmm1;"         //xmm1 = xmm1 * xmm7 = _od * _m
//				"MOVAPS %%xmm1, %3;"			//*_M1++ = xmm1
//				"SUBPS %%xmm1, %%xmm6;"         //xmm7 = xmm7 -xmm1 = _m - *_M1
//				"MOVAPS %%xmm6, %2"				//*_M0++ = xmm7
//
//				:"=m"(*_O0++),"=m"(*_O1++),"=m"(*_M0++),"=m"(*_M1)
//				:"m"(ori[i]),"m"(mag[i]),"m"(_norm),"m"(_oMax),"m"(_nb),"m"(_oMult),"m"(_nbf)
//				:"xmm0","xmm1","xmm2","xmm3","xmm4","xmm5","xmm6");
//	    _M1++;


	    _o=_mm_mul_ps(_mm_load_ps(ori+i),_oMult);
	    _o0=_mm_cvttps_epi32(_o);
	    _od=_mm_sub_ps(_o,_mm_cvtepi32_ps(_o0));
	    _o0=_mm_cvttps_epi32(_mm_mul_ps(_mm_cvtepi32_ps(_o0),_nbf));
	    _o0=_mm_and_si128(_mm_cmpgt_epi32(_oMax,_o0),_o0);
	     *_O0++=_o0;

	    _o1=_mm_add_epi32(_o0,_nb);
	    _o1=_mm_and_si128(_mm_cmpgt_epi32(_oMax,_o1),_o1);
	    *_O1++=_o1;

	    _m=_mm_mul_ps(_mm_load_ps(mag+i),_norm);
	    *_M1=_mm_mul_ps(_od,_m);
	    *_M0++=_mm_sub_ps(_m,*_M1);
	    _M1++;
	  } else for( i=0; i<=n-4; i+=4 ) {

	    _o=_mm_mul_ps(_mm_load_ps(ori+i),_oMult);
	    _o0=_mm_cvttps_epi32(_mm_add_ps(_o,_mm_set1_ps(.5f)));
	    _o0=_mm_cvttps_epi32(_mm_mul_ps(_mm_cvtepi32_ps(_o0),_nbf));
	    _o0=_mm_and_si128(_mm_cmpgt_epi32(_oMax,_o0),_o0);
	    *_O0++=_o0;
	    *_M0++=_mm_mul_ps(_mm_load_ps(mag+i),_norm);
	    *_M1++=_mm_set1_ps(0.f);
	    *_O1++=_mm_set1_epi32(0);
	  }

	if( interpolate )
	{
		for(; i<n; i++ )
		{
			o=ori[i]*oMult; o0=(int) o; od=o-o0;
			o0*=nb; if(o0>=oMax) o0=0; O0[i]=o0;
			o1=o0+nb; if(o1==oMax) o1=0; O1[i]=o1;
			m=mag[i]*norm; M1[i]=od*m; M0[i]=m-M1[i];
		}
	}
	else
	{
		for(; i<n; i++ )
		{
			o=ori[i]*oMult; o0=(int) (o+.5f);
			o0*=nb; if(o0>=oMax) o0=0; O0[i]=o0;
			M0[i]=mag[i]*norm; M1[i]=0; O1[i]=0;
		}
	}

#else
	if(interpolate)
	{
		for(i = 0; i < n; i++)
		{
			o = ori[i] * oMult; // o值为归一化到0-orients的方向
			o0 = (int)o; // 整数部分决定了0-orients中哪一个
			od = o - o0; // 方向小数部分
			o0 *= nb; // o0作为索引，可以将0-90度，指向第一个hist，90-180指向第二个hist，o0是大块的基地址
			if(o0 >= oMax)
			{
				o0 = 0;
			}
			O0[i] = o0;
			o1 = o0 + nb;// 索引之后后跳一个nb
			if(o1 == oMax)
			{
				o1 = 0;
			}
			O1[i] = o1;
			m = mag[i] * norm;  // norm是1/bin/bin，就是会有这么多个bin加起来
			M1[i] = od * m;  // od 0-1，方向越正，幅度的响应越高
			M0[i] = m - M1[i];  // bool部分
		}// 实现了将一个梯度分布分到N个角度，每个角度都对应一个直方图
	}
	else
	{
		for(i = 0; i < n; i++)
		{
			o = ori[i] * oMult;
			o0 = (int)(o + 0.5f);
			o0 *= nb;
			if(o0 >= oMax)
			{
				o0 = 0;
			}
			O0[i] = o0;
			M0[i] = mag[i] * norm;
			M1[i] = 0;
			O1[i] = 0;
		}
	}
#endif
}

void GradHist(Mat& mag, Mat& ori, Mat& hist, int32_t height, int32_t width, int32_t binSize, int32_t nOrients, int32_t softBin, int32_t full)
{
	const int32_t hb = height / binSize, wb = width / binSize, nb = wb * hb;
	const float s = (float)binSize, sInv = 1/s, sInv2 = 1/s/s;
	float *H0, *H1, *M0, *M1;
	int32_t x, y;
	int32_t *O0, *O1;

	O0 = (int32_t*)alMalloc(width * sizeof(int32_t), 16);
	O1 = (int32_t*)alMalloc(width * sizeof(int32_t), 16);
	M0 = (float*)alMalloc(width * sizeof(float), 16);
	M1 = (float*)alMalloc(width * sizeof(float), 16);

	for(y = 0; y < height; y++)
	{
		// Compute target orientation bins for entire row
		GradQuantize((float*)(ori.data+y*width*4), (float*)(mag.data+y*width*4), O0, O1, M0, M1,
			nb, width, sInv2, nOrients, full, softBin >= 0);
		if(softBin % 2 == 0 || binSize == 1)
		{
			// interpolate w.r.t. orientation only, not spatial bin
			H1 = (float*)(hist.data+(y/binSize)*wb*4); // H1 为指向y坐标直方图数据的指针，通过H1来修改hist数据，这里是H1本身的偏移，后面的索引是在H1的基础上的偏移
			#define GH H1[O0[x]] += M0[x]; H1[O1[x]] += M1[x]; x++;
			if(binSize == 1)		for(x = 0; x < width;){ GH; H1++;}
			else if(binSize == 2)	for(x = 0; x < width;){ GH; GH; H1++;}
			else if(binSize == 3)	for(x = 0; x < width;){ GH; GH; GH; H1++;}
			else if(binSize == 4)	for(x = 0; x < width;){ GH; GH; GH; GH; H1++;}
			else
			{
				for(x = 0; x < width;)
				{
					for(int32_t x1 = 0; x1 < binSize; x1++)
					{
						GH;
					}
					H1++;
				}
			}

		}
	}

	alFree(O0);
	alFree(O1);
	alFree(M0);
	alFree(M1);
}

void GradientHist(Mat& mag, Mat& ori, vector<Mat>& hist, int32_t& nChns, int32_t binSize, int32_t nOrients, int32_t softBin, int32_t useHog, float clipHog, int32_t full)
{
	int32_t height = mag.size().height;
	int32_t width = mag.size().width;
	int32_t hb = height / binSize;
	int32_t wb = width / binSize;

	nChns = useHog == 0 ? nOrients : (useHog == 1 ? nOrients*4 : nOrients*3+5);

	int32_t histSize[3] = {nChns, hb, wb};
	Mat computeHist = Mat(3, histSize, CV_32F, Scalar::all(0));

	if(useHog == 0)
	{
		GradHist(mag, ori, computeHist, height, width, binSize, nOrients, softBin, full);
	}

	//Separate computeHist into hist
	int32_t hs = computeHist.size[1] * computeHist.size[2] * 4;
	hist.clear();
	for(int32_t i = 0; i < nChns; i++)
	{
		Mat h(computeHist.size[1], computeHist.size[2], CV_32F);
		memcpy(h.data, computeHist.data + i * hs, hs);
		hist.push_back(h);
	}
}

}

