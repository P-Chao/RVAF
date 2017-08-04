/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageGradient.c 	                                             */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2012/11/16                                                          */
/*                                                                           */
/* Description: 图像梯度                                                     */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _IMAGE_GRADIENT_H_
#define _IMAGE_GRADIENT_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

#include "VectorBasic.h"

#include "stdio.h"
#include "string.h"


extern void ImageGradient1(IN IMAGE_S *srcImage, 
                           OUT IMAGE32_S *Gx, 
                           OUT IMAGE32_S *Gy,
                           IN int type);

extern void ImageGradientX2(IN IMAGE_S *srcImage,
                            OUT IMAGE32_S *Gx,
                            IN int type);

extern void ImageSobelHor(IN IMAGE_S *srcImage,
                          OUT IMAGE32_S *Gx,
                          IN int trunc,
                          IN int c);


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
