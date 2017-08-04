/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchCost.h 	                                             */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/06/16                                                          */
/*                                                                           */
/* Description: 图像匹配值计算                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _IMAGE_MATCH_COST_H_
#define _IMAGE_MATCH_COST_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

#include "ImageColor.h"
#include "ImageCensus.h"

extern void ImcPaperCensusInt(IN IMAGE_S *imageL,
                              IN IMAGE_S *imageR,
                              IN int dlength,
                              IN int cw,
                              IN int ch,
                              OUT int *mcost,
                              OUT int *pixeldsi);

extern void ImcImageCensusInt(IN IMAGE_S *imageL, 
                              IN IMAGE_S *imageR, 
                              IN int cwx,
                              IN int cwy,
                              IN int dlength,
                              OUT int *imagedsi);

extern void ImcDsiReverseInt(IN int *dsi, 
                             OUT int *invdsi, 
                             IN int height, 
                             IN int width, 
                             IN int dlength,
                             IN int bordercost,
                             IN int intp);

extern void ImcPaperCensusUshort(IN IMAGE_S *imageL,
                                 IN IMAGE_S *imageR,
                                 IN int dlength,
                                 IN int cw,
                                 IN int ch,
                                 OUT ushort *mcost,
                                 OUT ushort *pixeldsi);

extern void ImcImageCensusUshort(IN IMAGE_S *imageL, 
                                 IN IMAGE_S *imageR, 
                                 IN int cwx,
                                 IN int cwy,
                                 IN int dlength,
                                 OUT ushort *imagedsi);

extern void ImcDsiReverseUshort(IN ushort *dsi, 
                                OUT ushort *invdsi, 
                                IN int height, 
                                IN int width, 
                                IN int dlength,
                                IN ushort bordercost,
                                IN int intp);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
