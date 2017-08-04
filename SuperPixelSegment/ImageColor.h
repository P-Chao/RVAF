/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageColor.h 			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/02/06                                                          */
/*                                                                           */
/* Description: 彩色图像变换                                                 */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _IMAGE_COLOR_H_
#define _IMAGE_COLOR_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

/* 色彩空间 */
typedef enum tagColorSpace
{
    COLOR_SPACE_RGB = 0,
    COLOR_SPACE_LAB,
    COLOR_SPACE_LUV,
    COLOR_SPACE_YUV,
    COLOR_SPACE_HSV,
    COLOR_SPACE_HSI,
    COLOR_SPACE_MAX

}ColorSpace;

/* YUV宏定义 */
typedef enum tagImageYuv
{
    IMAGE_COLOR_YUV_Y = 0,
    IMAGE_COLOR_YUV_U,
    IMAGE_COLOR_YUV_V,
    IMAGE_COLOR_YUV_MAX

}IMAGE_YUV_E;

extern void ImageRgb2Gray(IN IMAGE_S *srcImage, OUT IMAGE_S *dstImage, IN RECT_S *rect);
extern void Rgb2Yuv(IN PIXEL *rgb, OUT double *yuv);

extern void MatrixSeqInit(IN Matrix_S *matseq, IN int height, IN int width, IN int channel);
extern void MatrixSeqDestroy(IN Matrix_S *matseq);
extern void MatrixRgb2Seq(IN Matrix_S *srcImage, INOUT Matrix_S *dstImage);


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
