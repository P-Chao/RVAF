/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: BaseFuncDef.h 			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 定义常用宏函数                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*****************************************************************************/

#ifndef _BASE_FUNC_DEF_H_
#define _BASE_FUNC_DEF_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
//#include <math.h>

#ifndef PLATFORM_CCS
#include <malloc.h>
#pragma comment(lib,"IQmath_pc.lib")
#endif
/******************************************************************************
 *                              定义常用宏函数
******************************************************************************/

// 拼凑字符串: 比如函数名，变量名
#define DefStrcat(A,B)      A ## B
#define DefStrcat2(A,B)     DefStrcat(A,B)
#define DefStrcat3(A,B,C)   DefStrcat2(DefStrcat2(A,B),C)

/******************************************************************************
 *                                数学类函数
******************************************************************************/

#if 0
// 优化 round(float*int)
IQfloat = _FtoIQN(float,Q);
ret = _IQNint(IQfactor*int+IQ0_5,Q);
#endif

/* 浮点数的四舍五入，考虑负数 */
/* ceil(-1.8) = -2, ceil(1.2) = 2 */
#ifndef round2
#define round2(x)               ((x) < 0 ? ((int)((x)-0.5)) : ((int)((x)+0.5)))
#endif

/* 两整数相除的四舍五入 */
#ifndef roundIntDivInt
#define roundIntDivInt(x,y)     ((2*(x)+(y))/(2*(y)))
#endif

/* x的浮点数去除，浮点数据向里收缩 */
/* c语言类型转换int */
/* int(-1.2) = -1, int(1.2) = 1 */

/* 刚大于x的最小整数，浮点数据向右移动 */
/* c语言函数ceil */
/* ceil(-1.2) = -1, ceil(1.2) = 2 */

/* 两整数相除的ceil */

// 4中带返回值的宏函数实现

// 方式1: 低效
#ifndef ceilIntDivInt
#define ceilIntDivInt(x,y)  ((x)/(y)*(y) == (x) ? (x)/(y):((x)/(y)+1))
#endif

#if 0
// 方式2: 返回值在参量中
#ifndef ceilIntDivInt
#define ceilIntDivInt(x,y,z){int temp = (x)/(y); z = (temp*(y) == (x)) ? temp:(temp+1);}
#endif

// 方式3: 只支持GCC编译器
#ifndef ceilIntDivInt
#define ceilIntDivInt(x,y)  ({int temp = (x)/(y); (temp*(y) == (x)) ? temp:(temp+1);})
#endif

// 方式4: 内联
INLINE int ceilIntDivInt(int x,int y)   {int temp = x/y; return (temp*y == x) ? temp:(temp+1);}
#endif

/* 刚大于x的最小整数，浮点数据向左移动 */
/* c语言函数floor */
/* floor(-1.2) = -2, floor(1.2) = 1 */
/*
#ifndef floor
#define floor(x,y)                                  \
{                                                   \
    int xiM = (int)(x);                             \
    if (x >= 0 || xiM-(x) < FLOAT_EPS) y = xiM;     \
    else y = xiM-1;                                 \
}
#endif
*/

/* 符号函数 */
#ifndef sgn
#define sgn(x)                  ((x) >= 0 ? 1 : -1)
#endif

/* 边界函数 */
#ifndef bound
#define bound(x,mina,maxa)      (x < mina ? mina : (x > maxa ? maxa : x))
#endif

/* 角度转换 */
#ifndef deg2rad
#define deg2rad(x)              ((x)*MATH_PI/180)
#endif

#ifndef rad2deg
#define rad2deg(x)              ((x)*180/MATH_PI)
#endif

#if 0
// c语言函数
/* 双曲正弦函数 */
#ifndef sinh
#define sinh(x)                ((exp(x)-exp(-x))/2)
#endif

/* 双曲余弦函数 */
#ifndef cosh
#define cosh(x)                ((exp(x)+exp(-x))/2)
#endif
#endif



/* 根据x和y方向的梯度求倾角(0 2π), arctan(cos(a), sin(a), a) */
#define arctan(dxM, dyM, angleM)                    \
{                                                   \
    angleM = atan((dyM)/(dxM));                     \
    if ((dxM) < 0)              angleM += MATH_2PI;  \
    else if ((dyM) < 0)         angleM += MATH_2PI; \
}

/* 两数据较大值 */
//#ifndef max
//#define max(a,b)                (((a) > (b)) ? (a) : (b))
//#endif

/* 两数据较小值 */
//#ifndef min
//#define min(a,b)                (((a) < (b)) ? (a) : (b))
//#endif

/* 两数据较大和较小值 */
#ifndef maxmin
#define maxmin(x0, x1, xmin, xmax, flag)            \
{                                                   \
    if (x0 < x1)                                    \
    {                                               \
        xmin = x0;                                  \
        xmax = x1;                                  \
        flag = 0;                                   \
    }                                               \
    else                                            \
    {                                               \
        xmin = x1;                                  \
        xmax = x0;                                  \
        flag = 1;                                   \
    }                                               \
}
#endif

/* 两数据较小值 */
#ifndef gabs
#define gabs(a)                 (((a) > 0) ? (a) : (-(a)))
#endif

/* 数据的最大最小归一化 */
//#ifndef scale
//#define scale(data,maxM,minM)   (((data)-(minM))/((maxM)-(minM)))
//#endif

/* 两数据交换 */
#ifndef swapAB
#define swapAB(a,b,temp)        \
{                               \
    temp = a;                   \
    a = b;                      \
    b = temp;                   \
}
#endif

/* 判断浮点数为零 */
#ifndef isFloatZero
#define isFloatZero(x)          (fabs(x) < FLOAT_EPS)
#endif
#ifndef isDoubleZero
#define isDoubleZero(x)         (fabs(x) < DOUBLE_EPS)
#endif
#ifndef isNumericZero
#define isNumericZero(x,eps)    (fabs(x) < (eps))
#endif
#ifndef isFloatInt
#define isFloatInt(x)           (fabs(x-(int)x) < FLOAT_EPS)
#endif


#ifdef PLATFORM_SSE
extern __m128 mm_absfloat_mask;
extern void mmAbsFloatMaskInit();
#define _mm_abs_ps(x) _mm_and_ps(x, mm_absfloat_mask)
#endif

extern int digitInt(IN int x, IN int n);
extern int squareSum(IN int n);
extern int BitCountU64(IN int64 x);
extern int BitCountU32(IN int x);

extern double gibbs(double x, double sigma);
extern double gaussian(double x, double sigma);
extern void memsetint(int *a, int n, int x);
extern void Tabel1ExpFloat(OUT float *table, IN int length, IN float sigma);
extern void Tabel2ExpFloat(OUT float *table, IN int len1, IN int len2, IN float sigma);



/******************************************************************************
 *                               内存类函数
******************************************************************************/

#ifndef PLATFORM_CCS

#ifndef MALLOC
#define MALLOC(size)            malloc(size)
#endif

#ifndef FREE
#define FREE(p,size)            free(p)
#endif

#else

extern int DDR2;
extern int IRAM;
extern int L1DSRAM;

#ifndef MALLOC
#define MALLOC(size)            MEM_alloc(DDR2, size, 4)
#endif

#ifndef FREE
#define FREE(p, size)           MEM_free(DDR2, p, size)
#endif


#endif


/* 申请type类型的内存 */
#ifndef MallocType
#define MallocType(type, size)      (type* )MALLOC((size)*sizeof(type))
#endif

#ifndef FreeType
#define FreeType(p, type, size)     FREE(p, (size)*sizeof(type))
#endif


/* 字节对齐内存申请 */
#ifdef PLATFORM_SSE

#ifndef mm_MallocType
#define mm_MallocType(type, size)       (type *)_mm_malloc(size*sizeof(type), 16)
#endif

#ifndef mm_FreeType
#define mm_FreeType(p, type, size)      _mm_free(p);
#endif

#else

#ifndef mm_MallocType
#define mm_MallocType(type, size)       MallocType(type, size)
#endif

#ifndef mm_FreeType
#define mm_FreeType(p, type, size)      FreeType(p, type, size)
#endif

#endif


/* cache内存申请 */
#ifdef PLATFORM_CCS

#ifndef ram_MallocType
#define ram_MallocType(type, size)      (type *)MEM_alloc(IRAM, size*sizeof(type), 4)
#endif

#ifndef ram_FreeType
#define ram_FreeType(p, type, size)     MEM_free(IRAM, p, size*sizeof(type));
#endif

#else

#ifndef ram_MallocType
#define ram_MallocType(type, size)      mm_MallocType(type, size)
#endif

#ifndef ram_FreeType
#define ram_FreeType(p, type, size)     mm_FreeType(p, type, size)
#endif

#endif

/******************************************************************************
 *                               图像类函数
******************************************************************************/

#ifndef RectEnsure
#define RectEnsure(r)               \
{                                   \
    if (r == NULL)                  \
    {                               \
        left = 0;                   \
        right = width-1;            \
        top = 0;                    \
        bottom = height-1;          \
    }                               \
    else                            \
    {                               \
        left = r->left;             \
        right = r->right;           \
        top = r->top;               \
        bottom = r->bottom;         \
    }                               \
}
#endif

/* 图像点的8邻域方位 */
#ifndef IMAGE_TOP
#define IMAGE_TOP(pImage,width)         ((pImage)-(width))
#endif
#ifndef IMAGE_BOTTOM
#define IMAGE_BOTTOM(pImage,width)      ((pImage)+(width))
#endif
#ifndef IMAGE_LEFT
#define IMAGE_LEFT(pImage,width)        ((pImage)-1)
#endif
#ifndef IMAGE_RIGHT
#define IMAGE_RIGHT(pImage,width)       ((pImage)+1)
#endif
#ifndef IMAGE_LEFTTOP
#define IMAGE_LEFTTOP(pImage,width)     ((pImage)-(width)-1)
#endif
#ifndef IMAGE_RIGHTTOP
#define IMAGE_RIGHTTOP(pImage,width)    ((pImage)-(width)+1)
#endif
#ifndef IMAGE_LEFTBOTTOM
#define IMAGE_LEFTBOTTOM(pImage,width)  ((pImage)+(width)-1)
#endif
#ifndef IMAGE_RIGHTBOTTOM
#define IMAGE_RIGHTBOTTOM(pImage,width) ((pImage)+(width)+1)
#endif

// 图像点的平移
#define POINT_MOVE(src, dst, x, y)  \
{                                   \
    dst.x = src.x + x;              \
    dst.y = src.y + y;              \
}

// 图像点的旋转
// IN POINT_S src: 待旋转的点坐标
// IN POINT64_S dst: 旋转后的点坐标
// IN POINT_S origin: 旋转轴点
// IN double angle: 旋转角
#define POINT_ROTATE(src, dst, origin, angle)                   \
{                                                               \
    int widthM = src.x-origin.x;                                \
    int heightM = src.y-origin.y;                               \
    dst.x = origin.x + widthM*cos(angle) - heightM*sin(angle);  \
    dst.y = origin.y + widthM*sin(angle) + heightM*cos(angle);  \
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
