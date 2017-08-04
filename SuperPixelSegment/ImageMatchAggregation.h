/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchAggregation.h 	                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/06/16                                                          */
/*                                                                           */
/* Description: 图像匹配值累积                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _IMAGE_MATCH_AGGREGATION_H_
#define _IMAGE_MATCH_AGGREGATION_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#define IMA_STI_Q_WEIGHT        8
#define IMA_STI_S_WEIGHT        256

#ifdef PLATFORM_CCS
    #define IMA_STI_BUFEX           2
    #define IMA_STUS_BUFEX          4
#elif defined PLATFORM_SSE
    #define IMA_STI_BUFEX           8
    #define IMA_STUS_BUFEX          16
#else
    #define IMA_STI_BUFEX           2
    #define IMA_STUS_BUFEX          4
#endif
#define IMA_STI_BUFOFF              (IMA_STI_BUFEX >> 1)
#define IMA_STUS_BUFOFF             (IMA_STUS_BUFEX >> 1)


typedef struct tagImaScanTreeIntInfo
{
    // 外部信息
    int height;
    int width;
    int dlength;
    int *imagecost;
    IMAGE_S *src;
    IMAGE32_S *Gx;
    IMAGE32_S *Gy;

    // 参数
    int mode;
    floatcost sg;   // spatial(geo) sigma
    floatcost sc;   // range(color) sigma
    int P1;
    int P2;

    // 内部信息
    int *wgt[4];
    int *smoothcost;

    // 临时内存
    int wtablelen;
    int *wtable;
    int *LpBuf;
    int *SoBuf;

}ImaScanTreeIntInfo;

extern void ImaScanTreeIntAggrPara(ImaScanTreeIntInfo *aggr);
extern void ImaScanTreeIntAggrInit(ImaScanTreeIntInfo *aggr);
extern void ImaScanTreeIntAggrDestroy(ImaScanTreeIntInfo *aggr);
extern void ImaScanTreeIntAggrProc(INOUT ImaScanTreeIntInfo *aggr);
extern void ImaScanTreeIntAggrHor(IN ImaScanTreeIntInfo *aggr, 
                                  IN int *imagecost,
                                  OUT int *smoothcost);
extern void ImaScanTreeIntAggrVer(IN ImaScanTreeIntInfo *aggr, 
                                  IN int *imagecost,
                                  OUT int *smoothcost);
extern void StaLineDpaggrInt(IN int *imagecost, 
                             IN int *weight, 
                             IN int height, 
                             IN int width, 
                             IN int dlength, 
                             IN int begin, 
                             IN int P1,
                             IN int P2,
                             OUT int *aggrcost, 
                             OUT int *LpBuf);
extern void StaLineDpaggrInt1(IN int *imagecost, 
                              IN int *weight, 
                              IN int height, 
                              IN int width, 
                              IN int dlength, 
                              IN int begin, 
                              IN int P1,
                              IN int P2,
                              OUT int *aggrcost, 
                              OUT int *LpBuf);
extern void StaLineDpaggrInt2(IN int *imagecost, 
                              IN int *weight, 
                              IN int height, 
                              IN int width, 
                              IN int dlength, 
                              IN int begin, 
                              IN int P1,
                              IN int P2,
                              OUT int *aggrcost, 
                              OUT int *LpBuf);
extern void StaMoveCenterInt(IN int *imagecost, 
                             IN int height, 
                             IN int width, 
                             IN int dlength, 
                             IN int direct, 
                             OUT int *aggrcost);
extern void StaWeightTabelInt(IN floatcost fg,
                              IN floatcost sc,
                              OUT int *table,
                              IN int length);
extern void StaWeightLookupInt(IN IMAGE32_S *G, 
                               OUT int *weight1, 
                               OUT int *weight2, 
                               IN int *table,
                               IN int direct);


typedef struct tagImaScanTreeUshortInfo
{
    // 外部信息
    int height;
    int width;
    int dlength;
    ushort *imagecost;

    // 参数
    int mode;
    ushort P1;
    ushort P2;

    // 内部信息
    ushort *smoothcost;
    ushort *LpBuf;
    ushort *minLp;

}ImaScanTreeUshortInfo;

extern void ImaScanTreeUshortAggrPara(ImaScanTreeUshortInfo *aggr);
extern void ImaScanTreeUshortAggrInit(ImaScanTreeUshortInfo *aggr);
extern void ImaScanTreeUshortAggrDestroy(ImaScanTreeUshortInfo *aggr);
extern void ImaScanTreeUshortAggrProc(INOUT ImaScanTreeUshortInfo *aggr);
extern void StaLineSgmaggrUshort(IN ushort *imagecost, 
                                 IN int height, 
                                 IN int width, 
                                 IN int dlength, 
                                 IN int begin, 
                                 IN ushort P1,
                                 IN ushort P2,
                                 OUT ushort *aggrcost, 
                                 OUT ushort *LpBuf);
extern void StaLineSgmaggrUshort1(IN ushort *imagecost, 
                                  IN int height, 
                                  IN int width, 
                                  IN int dlength, 
                                  IN int begin, 
                                  IN ushort P1,
                                  IN ushort P2,
                                  OUT ushort *aggrcost, 
                                  OUT ushort *LpBuf);

extern void ImaScanTreeUshortAggrProcXdy(INOUT ImaScanTreeUshortInfo *aggr);
extern void StaLineSgmaggrUshortXdy(IN ushort *imagecost, 
                                    IN int height, 
                                    IN int width, 
                                    IN int dlength, 
                                    IN int begin, 
                                    IN ushort P1,
                                    IN ushort P2,
                                    OUT ushort *aggrcost, 
                                    INOUT ushort *minLp,
                                    INOUT ushort *LpBuf);


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
