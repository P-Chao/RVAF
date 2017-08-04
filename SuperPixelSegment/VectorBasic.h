/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: VectorBasic.c      			                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 向量操作                                                     */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef VECTOR_BASIC_H_
#define VECTOR_BASIC_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

#include "string.h"

// int
extern void VectorPlusInt(IN int *A, IN int *B, OUT int *C, IN int n);
extern void VectorPlusConstInt(IN int *A, IN int *B, OUT int t, IN int n);
extern void VectorMulInt(IN int *A, IN int *B, OUT int *C, IN int n);
extern void VectorMulConstInt(IN int *A, OUT int *B, IN int t, IN int n);
extern void VectorDivConstInt(IN int *A, OUT int *B, IN int t, IN int n);
extern void VectorSubInt(IN int *A, IN int *B, OUT int *C, IN int n);
extern void VectorPlusMulInt(IN int *A, IN int *B, IN int *C, OUT int *D, IN int n);
extern void VectorPlusMulConstInt(IN int *A, IN int *B, OUT int *C, IN int t, IN int n);
extern int VectorDotInt(IN int *A, IN int *B, IN int n);
extern void VectorCross3Int(IN int *A, IN int *B, OUT int *C);
extern int VectorNorm2Int(IN int *A, IN int n);
extern int VectorNormInfInt(IN int *A, IN int n);
extern int VectorDistInt(IN int *a, IN int *b, IN int n, IN int type);
extern int VectorCurcleDistInt(IN int *a, IN int *b, IN int circle, IN int n, IN int type);
extern void VectorReverseInt(IN int *A, OUT int *B, IN int n);

// float
extern void VectorPlusFloat(IN float *A, IN float *B, OUT float *C, IN int n);
extern void VectorPlusConstFloat(IN float *A, OUT float *B, IN float t, IN int n);
extern void VectorMulConstFloat(IN float *A, OUT float *B, IN float t, IN int n);
extern void VectorSubFloat(IN float *A, IN float *B, OUT float *C, IN int n);
extern void VectorPlusMulFloat(IN float *A, IN float *B, IN float *C, OUT float *D, IN int n);
extern void VectorPlusMulConstFloat(IN float *A, IN float *B, OUT float *C, IN float t, IN int n);
extern float VectorDotFloat(IN float *A, IN float *B, IN int n);
extern void VectorCross3Float(IN float *A, IN float *B, OUT float *C);
extern float VectorNorm2Float(IN float *A, IN int n);
extern float VectorNormInfFloat(IN float *A, IN int n);
extern float VectorDistFloat(IN float *a, IN float *b, IN int n, IN int type);
extern float VectorCurcleDistFloat(IN float *a, IN float *b, IN float circle, IN int n, IN int type);
extern void VectorReverseFloat(IN float *A, OUT float *B, IN int n);
extern void VectorUnit2Float(IN float nx, 
                             IN float ny,
                             OUT float *onx, 
                             OUT float *ony);
extern void VectorUnit3Float(IN float nx, 
                             IN float ny,
                             IN float nz,
                             OUT float *onx, 
                             OUT float *ony,
                             OUT float *onz);

// double
extern void VectorPlusDouble(IN double *A, IN double *B, OUT double *C, IN int n);
extern void VectorPlusConstDouble(IN double *A, OUT double *B, IN double t, IN int n);
extern void VectorMulDouble(IN double *A, IN double *B, OUT double *C, IN int n);
extern void VectorMulConstDouble(IN double *A, OUT double *B, IN double t, IN int n);
extern void VectorSubDouble(IN double *A, IN double *B, OUT double *C, IN int n);
extern void VectorPlusMulDouble(IN double *A, IN double *B, IN double *C, OUT double *D, IN int n);
extern void VectorPlusMulConstDouble(IN double *A, IN double *B, OUT double *C, IN double t, IN int n);
extern double VectorDotDouble(IN double *A, IN double *B, IN int n);
extern void VectorCross3Double(IN double *A, IN double *B, OUT double *C);
extern double VectorNorm2Double(IN double *A, IN int n);
extern double VectorNormInfDouble(IN double *A, IN int n);
extern double VectorDistDouble(IN double *a, IN double *b, IN int n, IN int type);
extern double VectorCurcleDistDouble(IN double *a, IN double *b, IN double circle, IN int n, IN int type);
extern void VectorReverseDouble(IN double *A, OUT double *B, IN int n);

// only for double
extern double SparseVectorDot(IN SparseVector_S *A, IN SparseVector_S *B);
extern double MixVectorDot(IN Vector_S *A, IN SparseVector_S *B);
extern void VectorSkew(IN Vector_S *A, IN Matrix_S *B);
extern int VectorDistPixelInt(IN PIXEL *a, IN PIXEL *b, IN int n, IN int type);
extern double VectorDistPixel(IN PIXEL *a, IN PIXEL *b, IN int n, IN int type);
extern double VectorDistPixdou(IN PIXEL *a, IN double *b, IN int n, IN int type);
extern void VectorGibbs(IN Vector_S *A, OUT Vector_S *B, IN int n, IN double sigma);

// for Pixel
extern void VectorMapPixel(IN PIXEL *A, 
                           OUT PIXEL *B, 
                           IN PIXEL *map, 
                           IN int n, 
                           IN int c);
extern void VectorMapInt(IN int *A, 
                         OUT int *B, 
                         IN int *map, 
                         IN int n, 
                         IN int c);


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
