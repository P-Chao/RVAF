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

static const double gauss25[7][7] = {
	0.02546481, 0.02350698, 0.01849125, 0.01239505, 0.00708017, 0.00344629, 0.00142946,
	0.02350698, 0.02169968, 0.01706957, 0.01144208, 0.00653582, 0.00318132, 0.00131956,
	0.01849125, 0.01706957, 0.01342740, 0.00900066, 0.00514126, 0.00250252, 0.00103800,
	0.01239505, 0.01144208, 0.00900066, 0.00603332, 0.00344629, 0.00167749, 0.00069579,
	0.00708017, 0.00653582, 0.00514126, 0.00344629, 0.00196855, 0.00095820, 0.00039744,
	0.00344629, 0.00318132, 0.00250252, 0.00167749, 0.00095820, 0.00046640, 0.00019346,
	0.00142946, 0.00131956, 0.00103800, 0.00069579, 0.00039744, 0.00019346, 0.00008024
};

static inline const float haarX(const fimg& ii_img, 
	const int row, const int column, const int s){
	return sumRect(ii_img, row - s / 2, column, s, s / 2)
		- sumRect(ii_img, row - s / 2, column - s / 2, s, s / 2);
}

static inline const float haarY(const fimg& ii_img, 
	const int row, const int column, const int s){
	return sumRect(ii_img, row, column - s / 2, s / 2, s)
		- sumRect(ii_img, row - s / 2, column - s / 2, s / 2, s);
}

static inline const float gaussian(const float x, const float y, const float sig){
	return 1.0f / (2.0f*PI*sig*sig) * exp(-(x*x + y*y) / (2.0f*sig*sig));
}

inline static const float getAngle(const float X, const float Y){
	if (X > 0 && Y >= 0)
		return atan(Y / X);
	if (X < 0 && Y >= 0)
		return PI - atan(-Y / X);
	if (X < 0 && Y < 0)
		return PI + atan(Y / X);
	if (X > 0 && Y < 0)
		return 2 * PI - atan(-Y / X);
	return 0;
}

static void computeDiscriptors(const fimg& ii_img, vector<Ipoint>& ipts, 
	const bool upright){
	for (int idx = 0; idx < ipts.size(); ++idx){
	  float co, si;
	  float cx = -0.5f;
	  float cy = 0.0f;
	  float len = 0.0f;
	  int count = 0;
	  float scale = ipts[idx].s;
	  int x = int(ipts[idx].x + 0.5);
	  int y = int(ipts[idx].y + 0.5);
	  float *desc = ipts[idx].descriptor;

	  if (upright){
		co = 1;
		si = 0;
	  } else{
		co = cos(ipts[idx].orientation);
		si = sin(ipts[idx].orientation);
	  }
	  int i = -8;
	  while (i < 12){
		int j = -8;
		i = i - 4;
		cx += 1.0f;
		cy = -0.5f;
		while (j < 12){
		  float dx, dy, mdx, mdy;
		  dx = dy = mdx = mdy = 0.0f;
		  cy += 1.0f;
		  j = j - 4;

		  const int ix = i + 5;
		  const int jx = j + 5;
		  const int xs = round(x + (-jx*scale*si + ix*scale*co));
		  const int ys = round(y + (jx*scale*co + ix*scale*si));
			for (int k = i; k < i + 9; ++k){
			  for (int l = j; l < j + 9; ++l){
				const int sample_x = round(x + (-l*scale*si + k*scale*co));
				const int sample_y = round(y + (l*scale*co + k*scale*si));

				const float gauss_s1 = gaussian(xs - sample_x, ys - sample_y, 2.5f*scale);
				const float rx = haarX(ii_img, sample_y, sample_x, 2 * round(scale));
				const float ry = haarY(ii_img, sample_y, sample_x, 2 * round(scale));

				const float rrx = gauss_s1 * (-rx * si + ry * co);
				const float rry = gauss_s1 * (rx * co + ry * si);

				dx += rrx;
				dy += rry;
				mdx += fabs(rrx);
				mdy += fabs(rry);
			  }
			}

			const float gauss_s2 = gaussian(cx - 2.0f, cy - 2.0f, 1.5f);
			desc[count++] = dx * gauss_s2;
			desc[count++] = dy * gauss_s2;
			desc[count++] = mdx * gauss_s2;
			desc[count++] = mdy * gauss_s2;

			len += (dx*dx + dy*dy + mdx*mdx + mdy*mdy) * gauss_s2 * gauss_s2;
			j += 9;
		  }
		  i += 9;
		}
		len = sqrt(len);
		for (int i = 0; i < 64; ++i){
			desc[i] /= len;
		}
	}

}

static void computeOrientation(const fimg& ii_img, vector<Ipoint>& ipts){
	const int id[] = { 6, 5, 4, 3, 2, 1, 0, 1, 2, 3, 4, 5, 6 };
	float resX[109], resY[109], Ang[109];

	for (int idx = 0; idx < ipts.size(); idx++){
		const int s = round(ipts[idx].s);
		const int r = round(ipts[idx].y);
		const int c = round(ipts[idx].x);

		int index = 0;
		for (int i = -6; i <= 6; ++i){
			for (int j = -6; j <= 6; ++j){
				if (i*i + j*j < 36){
					const float gauss = gauss25[id[i + 6]][id[j + 6]];
					resX[index] = gauss * haarX(ii_img, r + j*s, c + i*s, 4*s);
					resY[index] = gauss * haarY(ii_img, r + j*s, c + i*s, 4*s);
					Ang[index] = getAngle(resX[index], resY[index]);
					index++;
				}
			}
		}

		float sumX = 0.0f, sumY = 0.0f;
		float max = 0.0f, orientation = 0.0f;
		float ang1 = 0.0f, ang2 = 0.0f;

		for (ang1 = 0; ang1 < 2 * PI; ang1 += 0.15f){
			ang2 = (ang1 + PI / 3.0f > 2 * PI ? 
				ang1 - 5.0f * PI / 3.0f : ang1 + PI / 3.0f);
			sumX = sumY = 0.0f;
			for (unsigned int k = 0; k < 109; ++k){
				if (ang1 < ang2 && ang1 < Ang[k] && Ang[k] < ang2){
					sumX += resX[k];
					sumY += resY[k];
				} else if (ang2 < ang1 && ( (Ang[k] > 0 && Ang[k] < ang2) 
					|| (Ang[k] > ang1 && Ang[k] < 2 * PI) )){
					sumX += resX[k];
					sumY += resY[k];
				}
			}

			if (sumX * sumX + sumY * sumY > max){
				max = sumX * sumX + sumY * sumY;
				orientation = getAngle(sumX, sumY);
			}
		}

		ipts[idx].orientation = orientation;
	}
}

void surfDescriptors(const fimg& ii_img, vector<Ipoint>& ipts){
	if (!ipts.size()){
		return;
	}
	if (suparam.upright){
		computeDiscriptors(ii_img, ipts, true);
	} else{
		computeOrientation(ii_img, ipts);
		computeDiscriptors(ii_img, ipts, false);
	}

}


}