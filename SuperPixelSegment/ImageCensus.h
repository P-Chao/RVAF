/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageCensus.c 	                                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2012/11/16                                                          */
/*                                                                           */
/* Description: 图像Census变换                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _IMAGE_CENSUS_H_
#define _IMAGE_CENSUS_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

#include "stdio.h"
#include "string.h"

extern void ImageCensus(IN IMAGE_S *srcImage, 
                        OUT IMAGE64_S *dstImage,
                        IN int m,
                        IN int n);
extern void ImageCensus32(IN IMAGE_S *srcImage, 
                          OUT IMAGE32_S *dstImage,
                          IN int m,
                          IN int n);

extern void ImageHamming(IN IMAGE64_S *I1,
                         IN IMAGE64_S *I2,
                         IN RECT_S *r1,
                         IN RECT_S *r2,
                         IN RECT_S *r,
                         OUT IMAGE_S *Sub);
extern void ImageHamming32(IN IMAGE32_S *I1,
                           IN IMAGE32_S *I2,
                           IN RECT_S *r1,
                           IN RECT_S *r2,
                           IN RECT_S *r,
                           OUT IMAGE_S *I);


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
