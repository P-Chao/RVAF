/*
 * convTri.cpp
 *  for VisualStudio & opencv3.0.0
 *
 *  Created on: Aug 2, 2015
 *		Author: Peng Chao
 *	
 */

#include "wrappers.hpp"

#include <math.h>
#include <stdint.h>
#include <assert.h>

#include <opencv2\opencv.hpp>

#include "convTri.h"

namespace pc{



inline void PrefixSumSSE(__m128 x, float *s)
{
	x = _mm_add_ps(x, _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(x), 4)));
	__m128 out = _mm_add_ps(x, _mm_castsi128_ps(_mm_slli_si128(_mm_castps_si128(x), 8)));
	out = _mm_add_ps(out, _mm_setzero_ps());
	_mm_store_ps(s, out);
}

void ConvTriY(float* src, float* dst, int32_t w, int32_t r, int32_t s)
{
	r++;
	float t, u;
	int32_t j, r0 = r-1, r1 = r+1, r2 = 2*w-r, w0 = r+1, w1 = w-r+1, w2 = w;
	u = t = src[0];
	for(j = 1; j < r; j++)
	{
		u += t += src[j];
	}
	u = 2 * u - t;
	t = 0;

	if(s == 1)
	{
		dst[0] = u; j = 1;
		for(; j < w0; j++) dst[j] = u += t += (src[r-j]  + src[r0+j] - 2*src[j-1]);
#if(USE_SIMD == -1)
		int32_t w1_4 = w1 >> 2;
		for(; j < ((w0 + 4) & ~3); j++) dst[j] = u += t += (src[j-r1] + src[r0+j] - 2*src[j-1]);
		for(; j < w1_4;  j++)
		{
			__m128 s;
			__asm__(
				"MOVUPS %1, %%xmm0;"	//xmm0 = src[j-r1]
				"MOVUPS %2, %%xmm1;"	//xmm1 = src[r0+j]
				"MOVUPS %3, %%xmm2;"	//xmm2 = src[j-1]
				"ADDPS %%xmm0, %%xmm1;"	//xmm1 = src[j-r1] + src[r0+j]
				"VSUBPS %%xmm1, %%xmm2, %%xmm1;"	//xmm1 = src[j-r1] + src[r0+j] - src[j-1]
				"SUBPS %%xmm1, %%xmm2;"	//xmm2 = src[j-r1] + src[r0+j] - src[j-1] * 2
				"MOVAPS %%xmm2, %0"
				: "=m"(s)
				: "m"(src[j-r1]), "m"(src[r0+j]), "m"(src[j-1])
				: "xmm0", "xmm1", "xmm2");
			PrefixSumSSE(s, );
		}
		for(j = w1_4 << 2; j < w; j++) dst[j] = u += t += (src[j-r1] + src[r0+j] - 2*src[j-1]);
#else
		for(; j < w1; j++) dst[j] = u += t += (src[j-r1] + src[r0+j] - 2*src[j-1]);
#endif
		for(; j < w2; j++) dst[j] = u += t += (src[j-r1] + src[r2-j] - 2*src[j-1]);
	}
	else
	{
	    int32_t k = (s - 1) / 2;
	    w2 = (w / s) * s;
	    w0 = min(w0, w2);
	    w1 = min(w1, w2);

	    if(++k == s)
	    {
	    	k = 0;
	    	*dst++ = u;
	    }
	    j = 1;
	    for(; j < w0; j++)
	    {
	    	u += t += src[r-j] + src[r0+j] - 2*src[j-1];
	    	if(++k==s)
	    	{
	    		k=0;
	    		*dst++ = u;
	    	}
	    }
	    for(; j < w1; j++)
	    {
	    	u += t += src[j-r1] + src[r0+j] - 2*src[j-1];
	    	if(++k == s)
	    	{
	    		k=0;
	    		*dst++ = u;
	    	}
	    }
	    for(; j < w2; j++)
	    {
	    	u += t += src[j-r1] + src[r2-j] - 2*src[j-1];
	    	if(++k == s)
	    	{
	    		k = 0;
	    		*dst++ = u;
	    	}
	    }
	}
}

// Convolve src by a 2rx1 triangle
void ConvTri(float* src, float* dst, int32_t h, int32_t w, int32_t d, int32_t r, int32_t s)
{
	r++;
	float nrm = 1.0f / (r * r * r * r);
	int32_t i, j, k = (s - 1) / 2;
	int32_t w0, w1, h0;
	if(w % 4 == 0)
	{
		w0 = w1 = w;
	}
	else
	{
		w0 = w - (w % 4);
		w1 = w0 + 4;
	}
	h0 = (h / s) * s;
	float* T = (float*)alMalloc(2 * w * sizeof(float), 16);
	float* U = T + w1;
	while(d-- > 0)
	{
		// Initialize T and U
		for(j = 0; j < w; j++)
		{
			U[j] = T[j] = src[j];
		}

		for(i = 1; i < r; i++)
		{
			for(j = 0; j < w; j++)
			{
				U[j] += T[j] += src[j+i*w];
			}
		}

		for(j = 0; j < w; j++)
		{
			U[j] = nrm * (U[j]*2 - T[j]);
			T[j] = 0;
		}

		// Prepare and convolve each row in turn
		k++;
		if(k == s)
		{
			k = 0;
			ConvTriY(U, dst, w, r-1, s);
			dst += w / s;
		}
		for(i = 1; i < h; i++)
		{
			float *Il = src + (i - 1 - r) * w;
			if(i <= r)
			{
				Il = src + (r - i) * w;
			}
			float* Im = src + (i - 1) * w;
			float* Ir = src + (i - 1 + r) * w;
			if(i > h - r)
			{
				Ir = src + (2 * h - r - i) * w;
			}
#if(USE_SIMD == 1)
			float coefConst[4] __attribute__((aligned(16))) = {-2.0f, -2.0f, -2.0f, -2.0f};
			float nrmConst[4]  __attribute__((aligned(16))) = {nrm, nrm, nrm, nrm};
			for(j = 0; j < w0; j+=4)
			{
				__asm__(
					"MOVAPS %2, %%xmm0;"		//xmm0 = -2.0f
					"MOVAPS %3, %%xmm1;"		//xmm1 = Im
					"MULPS %%xmm0, %%xmm1;"		//xmm1 = -2 * Im
					"MOVAPS %4, %%xmm2;"		//xmm2 = Ir
					"ADDPS %%xmm2, %%xmm1;"		//xmm1 = Ir - 2 * Im
					"MOVAPS %5, %%xmm2;"		//xmm3 = Il
					"ADDPS %%xmm2, %%xmm1;"		//xmm1 = Il + Ir - 2 * Im
					"MOVAPS %7, %%xmm2;"		//xmm2 = T
					"ADDPS %%xmm2, %%xmm1;"		//xmm1 = T + (Il + Ir - 2 * Im)
					"MOVAPS %%xmm1, %1;"		//T = xmm1;
					"MOVAPS %6, %%xmm2;"		//xmm2 = nrm;
					"MULPS %%xmm2, %%xmm1;"		//xmm1 = nrm * T
					"MOVAPS %8, %%xmm2;"		//xmm2 = U
					"ADDPS %%xmm2, %%xmm1;"		//xmm1 = U + nrm * T
					"MOVAPS %%xmm1, %0;"		//U = xmm1;
					: "=m"(U[j]), "=m"(T[j])
					: "m"(coefConst[0]), "m"(Im[j]), "m"(Ir[j]), "m"(Il[j]), "m"(nrmConst[0]), "m"(T[j]), "m"(U[j])
					: "xmm0", "xmm1", "xmm2");
			}
			for(j = w0; j < w; j++)
			{
				T[j] += (Il[j] + Ir[j] - 2 * Im[j]);
				U[j] += nrm * T[j];
			}
#else
			for(j = 0; j < w; j++)
			{
				T[j] += (Il[j] + Ir[j] - 2 * Im[j]);
				U[j] += nrm * T[j];
			}
#endif
			k++;
			if(k == s)
			{
				k = 0;
				ConvTriY(U, dst, w, r-1, s);
				dst += w / s;
			}
		}
		src += w * h;
	}
	alFree(T);

}

void ConvTrix(Mat& src, Mat& dst, int32_t r)
{
	Mat tmp = Mat(src.rows, src.cols, CV_32F, Scalar::all(0));;
	ConvTri((float*)src.data, (float*)tmp.data, src.rows, src.cols, 1, r, 1);
	dst = tmp;
}

void ConvConst(Mat& src, Mat& dst, int32_t r, int32_t s)
{
	Mat f1, f2;
	Mat tmp;

	if(r <= 1)
	{
		//¹¹Ôì¾ØÕóºË
		float k[3];
		float p = 12 / r / (r+2) - 2;
		k[2] = k[0] = 1 / (2 + p);
		k[1] = p / (2 + p);
		f1 = Mat(1, 3, CV_32FC1, k);
		f2 = Mat(3, 1, CV_32FC1, k);
//		ConvTri((float*)src.data, (float*)tmp.data, src.rows, src.cols, 1, r, s);
		filter2D(src, tmp, -1, f1, Point(-1,-1), 0, BORDER_REFLECT);
		filter2D(tmp, dst, -1, f2, Point(-1,-1), 0, BORDER_REFLECT);
	}
	else
	{
		float* k = new float[2*r+1];
		float c = (r+1) * (r+1);
		for(int32_t i = 0; i < r; i++)
		{
			k[i] = (i+1) / c;
			k[r+1+i] = (r-i) / c;
		}
		k[r] = (r+1) / c;
		f1 = Mat(1, 2*r+1, CV_32FC1, k);
		f2 = Mat(2*r+1, 1, CV_32FC1, k);
		filter2D(src, tmp, -1, f1, Point(-1,-1), 0, BORDER_REFLECT);
		filter2D(tmp, dst, -1, f2, Point(-1,-1), 0, BORDER_REFLECT);
		delete [] k;
	}
	if(s > 1)
	{
		//Down sampling
		Size newSize;
		tmp = dst.clone();
		newSize.width = tmp.size().width / s;
		newSize.height = tmp.size().height / s;
		resize(tmp, dst, newSize, 0, 0, INTER_NEAREST);
	}
}

}
