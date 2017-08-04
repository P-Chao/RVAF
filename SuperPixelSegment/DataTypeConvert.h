/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: DataTypeConvert.h                                               */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 数据类型转换                                                 */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _DATA_TYPE_CONVERT_H_
#define _DATA_TYPE_CONVERT_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

#include <string.h>

extern void DataPixel2Double(IN PIXEL *in_data, OUT double *out_data, IN int n);

extern void DataInt2Pixel(IN int *in_data, OUT PIXEL *out_data, IN int n);
extern void DataInt2Float(IN int *in_data, OUT float *out_data, IN int n);
extern void DataInt2Double(IN int *in_data, OUT double *out_data, IN int n);

extern void DataPixel2Int(IN PIXEL *in_data, OUT int *out_data, IN int n);
extern void DataDouble2Int(IN double *in_data, OUT int *out_data, IN int n);


extern void DataDouble2Pixel(IN double *in_data, OUT PIXEL *out_data, IN int n);
extern void DataFloat2Int(IN float *in_data, OUT int *out_data, IN int n);
extern void DataFloat2Pixel(IN float *in_data, OUT PIXEL *out_data, IN int n, IN BOOL type);
extern void DataFloat2Double(IN float *in_data, OUT double *out_data, IN int n);
extern void DataDouble2Float(IN double *in_data, OUT float *out_data, IN int n);

extern void Point2Matrix(IN POINT_S *in_data, OUT Matrix_S *out_data, IN int n);
extern void Point3D642Matrix(IN POINT3D64_S *in_data, OUT Matrix_S *out_data, IN int n);
extern void Matrix2Point3D64(IN Matrix_S *in_data, OUT POINT3D64_S *out_data);

extern int Image2Matrix(IN IMAGE_S *in_data, OUT Matrix_S *out_data, IN int buffer);
extern int Matrix2Image(IN Matrix_S *in_data, OUT IMAGE_S *out_data, IN int buffer);

extern void Point2D2Point3D(IN POINT64_S *in_data, OUT POINT3D64_S *out_data, IN int n, IN int s);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif

