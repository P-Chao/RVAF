/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: BaseConstDef.h			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 定义常用数据宏                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*****************************************************************************/

#ifndef _BASE_CONST_DEF_H_
#define _BASE_CONST_DEF_H_
//#include <math.h>
/******************************************************************************
 *                              平台定义
******************************************************************************/
#define PLATFORM_VC6
#define PLATFORM_VS2010
#define PLATFORM_SSE
// #define PLATFORM_GCC
// #define PLATFORM_CCS

#ifdef PLATFORM_SSE
#include <emmintrin.h>
#endif

/******************************************************************************
 *                              浮点类型定义
******************************************************************************/

// 一般情况下的浮点类型(1 float or 2 double)
#define FLOAT_TYPE_GEN              2

// 分割模块的IQ优化
#define OPT_SEG_IQ                  0
#define FLOAT_TYPE_SEG              2
#if OPT_SEG_IQ == 1
#undef FLOAT_TYPE_SEG
#define FLOAT_TYPE_SEG              0
#endif
#define QFACTOR_REGION_WEIGHT       23

// cost模块的IQ优化(暂时不支持)，采用float与double结果一致
#define OPT_COST_IQ                 0
#define FLOAT_TYPE_COST             1
#if OPT_COST_IQ == 1
#undef FLOAT_TYPE_COST
#define FLOAT_TYPE_COST             0
#endif
#define QFACTOR_COST_WEIGHT         23

// fitting模块的IQ优化(暂时不支持)，采用float与double结果一致
#define OPT_FITTING_IQ              0
#define FLOAT_TYPE_FITTING          1
#if OPT_FITTING_IQ == 1
#undef FLOAT_TYPE_FITTING
#define FLOAT_TYPE_FITTING          0
#endif
#define QFACTOR_FITTING_WEIGHT      23

// disp的IQ优化(暂时不支持)，采用float与double结果一致
#define OPT_DISP_IQ                 0
#define FLOAT_TYPE_DISP             1
#if OPT_DISP_IQ == 1
#undef FLOAT_TYPE_DISP
#define FLOAT_TYPE_DISP             0
#endif
#define QFACTOR_DISP_WEIGHT         23


// 其他模块

// weight = avg(d) - T，大于0的直接排除，T <= 1     >>>>>  范围 [-1 0)
#define QFACTOR_LAYER_WEIGHT        30

// weight = cost的变化，[-n n]                      >>>>>  范围 (-100 100)
#define QFACTOR_FRAGMENT_WEIGHT     24

// weight = cost的变化 [-n 0) (end的n较大，base的n < 100)
#define QFACTOR_ENDRELABEL_WEIGHT       24
#define QFACTOR_BASERELABEL_WEIGHT      24


// weight = -soweight (极限范围不好确定，实际应该较小)
#define QFACTOR_BORDER_WEIGHT       24

/******************************************************************************
 *                  下面两个宏说明函数参数的输入输出特性
******************************************************************************/

/* 输入参数 */
#ifndef IN
#define IN
#endif

/* 输出参数 */
#ifndef OUT
#define OUT
#endif

/* 既输入也输出参数 */
#ifndef INOUT
#define INOUT
#endif

/******************************************************************************
 *                              函数类型定义
******************************************************************************/
#ifndef INLINE
#define INLINE  static __inline
#endif

#ifndef EXPORT
#define EXPORT  extern
#endif

/******************************************************************************
 *                              定义常用返回值
******************************************************************************/
#ifndef TRUE
#define TRUE                1
#endif

#ifndef FALSE
#define FALSE               0
#endif

/******************************************************************************
 *                              定义常用数据宏
******************************************************************************/

/* 0指针 */
#ifndef NULL
#define NULL            0
#endif

/* 无效索引 */
#ifndef INDEX_NULL
#define INDEX_NULL      -1
#endif

/* 圆周率π */
#ifndef MATH_PI
#define MATH_PI         3.14159265358979
#endif

/* 2π */
#ifndef MATH_2PI
#define MATH_2PI        6.28318530717958
#endif

/* 自然对数e */
#ifndef MATH_E
#define MATH_E          2.71828182845904
#endif

/* 对数ln(2) */
#ifndef MATH_LN2
#define MATH_LN2        0.693147180559945
#endif

/* sqrt(2) */
#ifndef MATH_SQRT2
#define MATH_SQRT2      1.414213562373095
#endif

/* sqrt(π) */
#ifndef MATH_SQRTPI
#define MATH_SQRTPI     1.772453850905516
#endif

/* 浮点精度(零) */
#ifndef FLOAT_EPS
#define FLOAT_EPS       1.0e-6
#endif
#ifndef DOUBLE_EPS
#define DOUBLE_EPS      1.0e-15
#endif

/* 无穷大数 */
#ifndef DOUBLE_INF
#define DOUBLE_INF      1.0e38
#endif

/* 无穷大数 */
#ifndef INT_INF
#define INT_INF         0x7FFFFFFF
#endif

#ifndef INT16_INF
#define INT16_INF       0x7FFF
#endif

#ifndef INT16_INF2
#define INT16_INF2      0x7FFF7FFF
#endif

/* memset的无穷大数，适用于int, float, double等类型 */
#ifndef MEMSET_INF
#define MEMSET_INF      0x7f
#endif

/******************************************************************************
 *                                  图像色彩
******************************************************************************/

/* 图像灰度级 */
#ifndef IMAGE_LEVEL
#define IMAGE_LEVEL     256
#endif

/* 灰度图像黑色 */
#ifndef IMAGE_BLACK
#define IMAGE_BLACK     0
#endif

/* 灰度图像白色 */
#ifndef IMAGE_WHITE
#define IMAGE_WHITE     255
#endif

/* 灰度图像灰色 */
#ifndef IMAGE_GREY
#define IMAGE_GREY      128
#endif

/* RGB图像黑色 */
#ifndef IMAGE_CVALUE_BLACK
#define IMAGE_CVALUE_BLACK      0,0,0
#endif

/* RGB图像白色 */
#ifndef IMAGE_CVALUE_WHITE
#define IMAGE_CVALUE_WHITE      255,255,255
#endif

/* RGB图像蓝色 */
#ifndef IMAGE_CVALUE_BLUE
#define IMAGE_CVALUE_BLUE       255,0,0
#endif

/* RGB图像绿色 */
#ifndef IMAGE_CVALUE_GREEN
#define IMAGE_CVALUE_GREEN      0,255,0
#endif

/* RGB图像红色 */
#ifndef IMAGE_CVALUE_RED
#define IMAGE_CVALUE_RED        0,0,255
#endif

/* RGB图像紫色 */
#ifndef IMAGE_CVALUE_PURPLE
#define IMAGE_CVALUE_PURPLE     255,0,255
#endif

/* RGB图像黄色 */
#ifndef IMAGE_CVALUE_YELLOW
#define IMAGE_CVALUE_YELLOW     0,255,255
#endif

/* RGB图像青色 */
#ifndef IMAGE_CVALUE_CYAN
#define IMAGE_CVALUE_CYAN       255,255,0
#endif

#endif
