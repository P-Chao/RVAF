/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: Stat.h    	    		                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 数据统计功能的实现                                           */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _STAT_H_
#define _STAT_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

// int
extern int StatMinInt(IN int a[], IN int n, IN int s);
extern int StatMaxInt(IN int a[], IN int n, IN int s);
extern void StatMinMaxInt(IN int a[], IN int n, IN int s, OUT int *mina, OUT int *maxa);
extern void StatMin12Int(IN int a[], IN int n, IN int s, OUT int *mina1, OUT int *mina2);
extern void StatMax12Int(IN int a[], IN int n, IN int s, OUT int *maxa1, OUT int *maxa2);
extern void StatMinIndexInt(IN int a[], IN int n, IN int s, OUT int *val, OUT int *index);
extern void StatMaxIndexInt(IN int a[], IN int n, IN int s, OUT int *val, OUT int *index);
extern void StatMinIndex12Int(IN int a[], 
                              IN int n, 
                              IN int s, 
                              OUT int *mina1, 
                              OUT int *mina2,
                              OUT int *index1,
                              OUT int *index2);
extern void StatMaxIndex12Int(IN int a[], 
                              IN int n, 
                              IN int s, 
                              OUT int *maxa1, 
                              OUT int *maxa2,
                              OUT int *index1,
                              OUT int *index2);
extern void StatMinPosInt(IN int a[], IN int *pos, IN int n, OUT int *val, OUT int *index);
extern void StatMaxPosInt(IN int a[], IN int *pos, IN int n, OUT int *val, OUT int *index);
extern void StatLocalMinInt(IN int a[], IN int n, IN int s, OUT int *pos, OUT int *num);
extern void StatLocalMaxInt(IN int a[], IN int n, IN int s, OUT int *pos, OUT int *num);
extern void StatLocalMaxThInt(IN int a[], IN int n, IN int s, IN int thre, OUT int *pos, OUT int *num);
extern void StatMaxDiscInt(IN int a[], IN int n, IN int s, OUT int *pos, OUT int *num);
extern int StatSumInt(IN int a[], IN int n, IN int s);
extern double StatVarnInt(IN int a[], IN int n, IN int s);
extern double StatEntropyInt(IN int a[], IN int n, IN int s);
extern int StatHistMedianInt(IN int h[], IN int acc[], OUT int *med, IN int n);

extern int StatSumPixel(IN PIXEL a[], IN int n, IN int s);

// float
extern float StatMinFloat(IN float a[], IN int n, IN int s);
extern float StatMaxFloat(IN float a[], IN int n, IN int s);
extern void StatMinMaxFloat(IN float a[], IN int n, IN int s, OUT float *mina, OUT float *maxa);
extern void StatMin12Float(IN float a[], IN int n, IN int s, OUT float *mina1, OUT float *mina2);
extern void StatMax12Float(IN float a[], IN int n, IN int s, OUT float *maxa1, OUT float *maxa2);
extern void StatMinIndexFloat(IN float a[], IN int n, IN int s, OUT float *val, OUT int *index);
extern void StatMaxIndexFloat(IN float a[], IN int n, IN int s, OUT float *val, OUT int *index);
extern void StatMinIndex12Float(IN float a[], 
                                IN int n, 
                                IN int s, 
                                OUT float *mina1, 
                                OUT float *mina2,
                                OUT int *index1,
                                OUT int *index2);
extern void StatMaxIndex12Float(IN float a[], 
                                 IN int n, 
                                 IN int s, 
                                 OUT float *maxa1, 
                                 OUT float *maxa2,
                                 OUT int *index1,
                                 OUT int *index2);
extern void StatMinPosFloat(IN float a[], IN int *pos, IN int n, OUT float *val, OUT int *index);
extern void StatMaxPosFloat(IN float a[], IN int *pos, IN int n, OUT float *val, OUT int *index);
extern void StatLocalMinFloat(IN float a[], IN int n, IN int s, OUT int *pos, OUT int *num);
extern void StatLocalMaxFloat(IN float a[], IN int n, IN int s, OUT int *pos, OUT int *num);
extern void StatLocalMaxThFloat(IN float a[], IN int n, IN int s, IN float thre, OUT int *pos, OUT int *num);
extern float StatSumFloat(IN float a[], IN int n, IN int s);
extern int StatHistMedianFloat(IN float h[], IN float acc[], OUT float *med, IN int n);

// double
extern double StatMinDouble(IN double a[], IN int n, IN int s);
extern double StatMaxDouble(IN double a[], IN int n, IN int s);
extern void StatMinMaxDouble(IN double a[], IN int n, IN int s, OUT double *mina, OUT double *maxa);
extern void StatMin12Double(IN double a[], IN int n, IN int s, OUT double *mina1, OUT double *mina2);
extern void StatMax12Double(IN double a[], IN int n, IN int s, OUT double *maxa1, OUT double *maxa2);
extern void StatMinIndexDouble(IN double a[], IN int n, IN int s, OUT double *val, OUT int *index);
extern void StatMaxIndexDouble(IN double a[], IN int n, IN int s, OUT double *val, OUT int *index);
extern void StatMinIndex12Double(IN double a[], 
                                 IN int n, 
                                 IN int s, 
                                 OUT double *mina1, 
                                 OUT double *mina2,
                                 OUT int *index1,
                                 OUT int *index2);
extern void StatMaxIndex12Double(IN double a[], 
                                 IN int n, 
                                 IN int s, 
                                 OUT double *maxa1, 
                                 OUT double *maxa2,
                                 OUT int *index1,
                                 OUT int *index2);
extern void StatMinPosDouble(IN double a[], IN int *pos, IN int n, OUT double *val, OUT int *index);
extern void StatMaxPosDouble(IN double a[], IN int *pos, IN int n, OUT double *val, OUT int *index);
extern void StatLocalMinDouble(IN double a[], IN int n, IN int s, OUT int *pos, OUT int *num);
extern void StatLocalMaxDouble(IN double a[], IN int n, IN int s, OUT int *pos, OUT int *num);
extern void StatLocalMaxThDouble(IN double a[], IN int n, IN int s, IN double thre, OUT int *pos, OUT int *num);
extern double StatSumDouble(IN double a[], IN int n, IN int s);
extern double StatAvgDouble(IN double a[], IN int n, IN int s);
extern double StatMomentDouble(IN double a[], IN int n, IN int s, IN int m);
extern double StatVarDouble(IN double a[], IN int n, IN int s);
extern double StatVarnDouble(IN double a[], IN int n, IN int s);
extern double StatCovMomentDouble(IN double a[], IN double b[], IN int n, IN int s, IN int ma, IN int mb);
extern double StatCovDouble(IN double a[], IN double b[], IN int n, IN int s);
extern double StatCovnDouble(IN double a[], IN double b[], IN int n, IN int s);
extern double StatCorrcoefDouble(IN double a[], IN double b[], IN int n, IN int s);
extern int StatHistMedianDouble(IN double h[], IN double acc[], OUT double *med, IN int n);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
