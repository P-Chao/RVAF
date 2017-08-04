/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchPostprocess.h 			                             */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: 立体匹配后处理                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _IMAGE_MATCH_POSTPROCESS_H_
#define _IMAGE_MATCH_POSTPROCESS_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#include "Stat.h"
#include "VectorBasic.h"

#define IMPP_CHECK_ERR_INVALID      -1
#define IMPP_CHECK_ERR_BORDER       -2
#define IMPP_CHECK_ERR_LR           -3
#define IMPP_CHECK_ERR_SPECKLE      -4

typedef struct tagImppInvalidInfo
{
    int *row;
    int *start;
    int *end;
    int linenum;

}ImppInvalidInfo;

typedef struct tagImppEafPara
{
    int xr;
    int yr;
    int dg;     // 1: MANHATTAN, 其他: EUCLIDEAN
    int dc;     // 1: MANHATTAN, 其他: INF
    int trunc;
    float sg;   // spatial(geo) sigma
    float sc;   // range(color) sigma
    float *tables;
    float *tablec;

}ImppEafPara;

extern void ImppOptimizeWtaPixel(IN int *dsi, 
                                 IN int dlength, 
                                 IN int num, 
                                 OUT PIXEL *label);
extern void ImppOptimizeWta(IN int *dsi, 
                            IN int dlength, 
                            IN int num, 
                            OUT int *label);
extern void ImppOptimizeWta16Pixel(IN ushort *dsi, 
                                   IN int dlength, 
                                   IN int num, 
                                   OUT PIXEL *label);
extern void ImppOptimizeWta16(IN ushort *dsi, 
                              IN int dlength, 
                              IN int num, 
                              OUT int *label);
extern void ImppDisp2ImagePixel(OUT int *dispimg, 
                                IN PIXEL *label, 
                                IN int area,
                                IN float factor);
extern void ImppDisp2Image(OUT int *dispimg, 
                           IN int *label, 
                           IN int area,
                           IN float factor);
extern void ImppDisp2Image8(OUT PIXEL *dispimg, 
                            IN PIXEL *label, 
                            IN int area,
                            IN int factor);
extern void ImppMedianFilterPixel(IN PIXEL *label, 
                                  OUT PIXEL *outlabel, 
                                  IN int height, 
                                  IN int width, 
                                  IN int radius);
extern void ImppMedianFilter(INOUT int *label, 
                             IN int height, 
                             IN int width, 
                             IN int radius);
extern void ImppLrCheckPixel(IN PIXEL *labelL, 
                             IN PIXEL *labelR, 
                             IN int height, 
                             IN int width, 
                             IN int T,
                             OUT PIXEL *label);
extern void ImppLrCheckInt(IN int *labelL, 
                           IN int *labelR, 
                           IN int height, 
                           IN int width, 
                           IN int T,
                           OUT int *label);
extern void ImppLrCheckFloat(IN float *labelL, 
                             IN float *labelR, 
                             IN int height, 
                             IN int width, 
                             IN float T,
                             OUT float *label);
extern void ImppRangeCheckFloat(INOUT float *label, IN int size, IN float maxd);
extern void ImppInvalidInit(OUT ImppInvalidInfo *invalid, IN int maxnum);
extern void ImppInvalidDestroy(OUT ImppInvalidInfo *invalid, IN int maxnum);
extern void ImppBackgroundFillPixel(IN PIXEL *check, 
                                    IN int height,
                                    IN int width,
                                    OUT PIXEL *label,
                                    OUT ImppInvalidInfo *invalid);
extern void ImppBackgroundFillInt(IN int *check, 
                                  IN int height,
                                  IN int width,
                                  OUT int *label,
                                  OUT ImppInvalidInfo *invalid);
extern void ImppBackgroundFillCoeff(IN float *coeff, 
                                    IN int *map, 
                                    IN float *check, 
                                    IN int height,
                                    IN int width,
                                    IN float maxdisp,
                                    OUT float *label,
                                    OUT ImppInvalidInfo *invalid);
extern void ImppEafTablePara0(INOUT ImppEafPara *eaf);
extern void ImppEafTablePara1(INOUT ImppEafPara *eaf);
extern void ImppEafTableInit(INOUT ImppEafPara *eaf);
extern void ImppEafTableDestroy(INOUT ImppEafPara *eaf);
extern void ImppWinWmfInt(IN int *label,
                          IN IMAGE_S *guild,
                          IN ImppInvalidInfo *invalid,
                          IN ImppEafPara *eaf,
                          IN int dlength,
                          OUT int *filt);
extern int ImppWinWmfOneInt(IN int *label,
                            IN IMAGE_S *guild,
                            IN ImppEafPara *eaf,
                            IN int cx,
                            IN int cy,
                            IN float *hist,
                            IN float *acchist,
                            IN int dlength);
extern void ImppWinWmfFloat(IN float *label,
                            IN IMAGE_S *guild,
                            IN ImppInvalidInfo *invalid,
                            IN ImppEafPara *eaf,
                            IN int dlength,
                            OUT float *filt);
extern float ImppWinWmfOneFloat(IN float *label,
                                IN IMAGE_S *guild,
                                IN ImppEafPara *eaf,
                                IN int cx,
                                IN int cy,
                                IN float *hist,
                                IN float *acchist,
                                IN int dlength,
                                IN int subpixel);
extern void ImageGrayMedian3x3(IN IMAGE_S *src, OUT IMAGE_S *filter);
extern void ImageColorMedian3x3(IN IMAGE_S *src, OUT IMAGE_S *filter);
#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
