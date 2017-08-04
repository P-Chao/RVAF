/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchSgm.h 			                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2015/05/05                                                          */
/*                                                                           */
/* Description: Sgm算法                                                      */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _IMAGE_MATCH_SGM_H_
#define _IMAGE_MATCH_SGM_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#include "ctmf.h"
#include "ImageGradient.h"
#include "ImageMatchCost.h"
#include "ImageMatchAggregation.h"
#include "ImageMatchPostprocess.h"

typedef struct tagImSgmInfo
{
    // 外部信息
    int imagenum;
    int height;
    int width;
    int channel;
    int dlength;
    IMAGE_S *src[2];

    // 参数
    int mode;
    int dispmr;         // 后处理label的中值滤波半径
    float r1;
    float r2;

    // 内部信息
    ushort *imagecost;
    ImaScanTreeUshortInfo aggr;
    ImppInvalidInfo invalid;

    // 输出视差
    PIXEL *left;
    PIXEL *right;
    PIXEL *sparse;
    PIXEL *fill;
    PIXEL *dense;

}ImSgmInfo;

extern void ImageMatchSgmPara(INOUT ImSgmInfo *Sgm);
extern void ImageMatchSgmInit(INOUT ImSgmInfo *Sgm);
extern void ImageMatchSgmDestroy(INOUT ImSgmInfo *Sgm);
extern void ImageMatchSgmProc(INOUT ImSgmInfo *Sgm);
extern void SgmGradient(INOUT ImSgmInfo *Sgm);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
