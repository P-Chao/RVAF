/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageRoadDetection.h 			                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: 路面检测                                                     */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _IMAGE_ROAD_DETECTION_H_
#define _IMAGE_ROAD_DETECTION_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#include "ImageMatchPostprocess.h"
#include "ImageIer.h"
#include "ImageMatchEadp.h"
#include "ImageMatchSgm.h"
#include "ImageRegionFeature.h"
#include "ImageRegionClassify.h"

typedef struct tagImageRdInfo
{
    IMAGE_S src[2];
    int dlength;

    int dispcal;
    int dispshow;
    int segshow;

    PIXEL *dispimg;
    PIXEL *segimg;
    PIXEL *lblimg;

    IerInfo ier;
    ImEadpInfo eadp;
    ImSgmInfo sgm;
    IrfFeaInfo feainfo;
    int *feature;
    int *label;

}ImageRdInfo;

extern void ImageRdPara(OUT ImageRdInfo *rd);
extern void ImageRdInfoset(OUT ImageRdInfo *rd, 
                           IN IMAGE_S *imageL, 
                           IN IMAGE_S *imageR,
                           IN int dlength);

extern void ImageRdInit(OUT ImageRdInfo *rd);
extern void ImageRdDestroy(OUT ImageRdInfo *rd);
extern void ImageRdProc(OUT ImageRdInfo *rd);

extern void TestMyData();

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
