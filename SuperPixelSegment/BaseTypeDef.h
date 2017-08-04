/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: BaseTypeDef.h 			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 定义常用类型结构                                             */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*****************************************************************************/

#ifndef _BASE_TYPE_DEF_H_
#define _BASE_TYPE_DEF_H_

#include "BaseConstDef.h"
/******************************************************************************
 *                              常用类型简写
******************************************************************************/
// 注: 在当前pc(32位)上，int为4个字节(与long一样)
typedef int                 BOOL;
typedef unsigned char       uchar;
typedef unsigned short      ushort;
typedef unsigned long       ulong;

typedef unsigned char       unit8;
typedef unsigned short      unit16;
typedef unsigned int        unit32;

#ifdef PLATFORM_VC6
typedef unsigned __int64    unit64;
#elif defined(PLATFORM_GCC)
typedef unsigned long long  unit64;
#elif defined(PLATFORM_CCS)
typedef unsigned long long  unit64;
#endif

#ifdef PLATFORM_VC6
typedef __int64             int64;
#elif defined(PLATFORM_GCC)
typedef long long           int64;
#elif defined(PLATFORM_CCS)
typedef long long           int64;
#endif

typedef union tagmix32
{
    float f32;
    int i32;
    unit32 ui32;
    ushort ui16[2]; 
    unit8 ui8[4]; 

}mix32;

typedef union tagmix64
{
    double f64;
    int64 i64;
    unit64 ui64;
    struct tagdi32
    {
        unit32 i32low;
        int i32high;
    }di32;
    struct tagdu32
    {
        unit32 ui32low;
        unit32 ui32high;
    }du32;

}mix64;


/******************************************************************************
 *                               浮点类型
******************************************************************************/

// 系统一般情况下的浮点
#if FLOAT_TYPE_GEN == 0
    typedef _iq             floatgen;
#elif FLOAT_TYPE_GEN == 1
    typedef float           floatgen;
#else
    typedef double          floatgen;
#endif

// 分割模块浮点
#if FLOAT_TYPE_SEG == 0
    typedef _iq             floatseg;
    #define VectorDistFloatSeg      VectorDistInt
#elif FLOAT_TYPE_SEG == 1
    typedef float           floatseg;
    #define VectorDistFloatSeg      VectorDistFloat
#else
    typedef double          floatseg;
    #define VectorDistFloatSeg      VectorDistDouble
#endif

// cost模块浮点(暂时不支持IQ优化)
#if FLOAT_TYPE_COST == 0
    typedef _iq                     floatcost;
    #define expff(x)                exp(x)

    #define VectorDistFloatCost     VectorDistInt
    #define VectorPlusFloatCost     VectorPlusInt
    #define VectorSubFloatCost      VectorSubInt
    #define VectorMulConstFloatCost VectorMulConstInt
    #define VectorPlusMulConstFloatCost VectorPlusMulConstInt
    #define StatSumFloatCost        StatSumInt
    #define StatMinIndexFloatCost   StatMinIndexInt
    #define StatMinIndex12FloatCost StatMinIndex12Int
#elif FLOAT_TYPE_COST == 1
    typedef float                   floatcost;
    #define expff(x)                (float)exp(x)

    #define VectorDistFloatCost     VectorDistFloat
    #define VectorPlusFloatCost     VectorPlusFloat
    #define VectorSubFloatCost      VectorSubFloat
    #define VectorMulConstFloatCost VectorMulConstFloat
    #define VectorPlusMulConstFloatCost VectorPlusMulConstFloat
    #define StatSumFloatCost        StatSumFloat
    #define StatMinIndexFloatCost   StatMinIndexFloat
    #define StatMinIndex12FloatCost StatMinIndex12Float
#else
    typedef double                  floatcost;
    #define expff(x)                exp(x)

    #define VectorDistFloatCost     VectorDistDouble
    #define VectorPlusFloatCost     VectorPlusDouble
    #define VectorSubFloatCost      VectorSubDouble
    #define VectorMulConstFloatCost VectorMulConstDouble
    #define VectorPlusMulConstFloatCost VectorPlusMulConstDouble
    #define StatSumFloatCost        StatSumDouble
    #define StatMinIndexFloatCost   StatMinIndexDouble
    #define StatMinIndex12FloatCost StatMinIndex12Double
#endif

// fitting模块的浮点(暂时不支持IQ优化)
#if FLOAT_TYPE_FITTING == 0
    typedef _iq             floatfit;
    #define StatMaxIndexFloatFit    StatMaxIndexInt
    #define StatMinMaxFloatFit      StatMinMaxInt
#elif FLOAT_TYPE_FITTING == 1
    typedef float           floatfit;
    #define StatMaxIndexFloatFit    StatMaxIndexFloat
    #define StatMinMaxFloatFit      StatMinMaxFloat
#else
    typedef double          floatfit;
    #define StatMaxIndexFloatFit    StatMaxIndexDouble
    #define StatMinMaxFloatFit      StatMinMaxDouble
#endif

// disp模块的浮点(暂时不支持IQ优化)
#if FLOAT_TYPE_DISP == 0
    typedef _iq             floatdisp;
#elif FLOAT_TYPE_DISP == 1
    typedef float           floatdisp;    
    #define PointDisp       POINT32f_S
    #define sqrtff(x)       (float)sqrt(x)

    #define MatrixMulVectorCoeff    MatrixMulVectorf
#else
    typedef double          floatdisp;
    #define PointDisp       POINT64_S
    #define sqrtff(x)       sqrt(x)

    #define MatrixMulVectorCoeff    MatrixMulVector
#endif

/******************************************************************************
 *                              常用图像处理类型
******************************************************************************/

/* 像素 */
typedef unsigned char   PIXEL;

/* 坐标点 */
typedef struct tagPoint
{
	int x;
	int y;

}POINT_S;

typedef struct tagPoint32f
{
	float x;
	float y;

}POINT32f_S;

typedef struct tagPoint64
{
	double x;
	double y;

}POINT64_S;

/* 3D坐标点 */
typedef struct tagPoint3D
{
	int x;
	int y;
    int s;  // 深度或尺度

}POINT3D_S;
typedef struct tagPoint3D64
{
	double x;
	double y;
    double s;  // 深度或尺度

}POINT3D64_S;


/* 矩形区 */
typedef struct tagRect
{
    int left;
	int top;
	int right;
	int bottom;

}RECT_S;

typedef struct tagRect2
{
    int x;
	int y;
	int width;
	int height;
    int s;          // 尺度

}RECT2_S;

typedef struct tagRect64
{
    double left;
	double top;
	double right;
	double bottom;

}RECT64_S;

/* 图像结构 */
typedef struct tagImage
{
    PIXEL *data;    // 数据区
    int width;      // 宽度
    int height;     // 高度
    int channel;    // 通道数 1, 灰度   3, RGB

}IMAGE_S;

typedef struct tagImage16
{
    short *data;    // 数据区
    int width;      // 宽度
    int height;     // 高度
    int channel;

}IMAGE16_S;

typedef struct tagImage32
{
    int *data;      // 数据区
    int width;      // 宽度
    int height;     // 高度
    int channel;

}IMAGE32_S;

typedef struct tagImage32f
{
    float *data;    // 数据区
    int width;      // 宽度
    int height;     // 高度
    int channel;

}IMAGE32F_S;

typedef struct tagImage64
{
    int64 *data;   // 数据区
    int width;      // 宽度
    int height;     // 高度
    int channel;

}IMAGE64_S;

/* 图像序列 */
typedef struct tagImageSeq
{
    IMAGE_S *M;         // 图像结构
    PIXEL *buffer;      // 数据缓冲区
    int count;          // 图像个数
    int size;           // 缓冲区大小

}Image_Seq_S;

typedef struct tagImage32Seq
{
    IMAGE32_S *M;       // 图像结构
    int *buffer;        // 数据缓冲区
    int count;          // 图像个数
    int size;           // 缓冲区大小

}Image32_Seq_S;

/* 矩阵结构 */
typedef struct tagMatrix
{
    double *data;   // 数据区
    int width;      // 宽度
    int height;     // 高度
    int channel;

}Matrix_S;

/* 向量结构 */
typedef double Vector_S;

/* 矩阵序列 */
typedef struct tagMatrixSeq
{
    Matrix_S *M;        // 矩阵结构
    double *buffer;     // 数据缓冲区
    int count;          // 矩阵个数
    int size;           // 缓冲区大小

}Matrix_Seq_S;

/* 稀疏向量 */
typedef struct tagSparseVector
{
	int *index;
	double *val;
	int length;

}SparseVector_S;

/* 稀疏矩阵 */
typedef struct tagSparseMatrix
{
    SparseVector_S *data;   // 数据区
    int width;              // 宽度
    int height;             // 高度

}SparseMatrix_S;

/* CCS数据格式 */
typedef union tagCcsData
{
	PIXEL pixel[4];
	ushort pixel16[2];
	ulong item;
	int data;       // 也是4字节

}CcsData;

/* 等价的标签链表 */
typedef struct tagEqualMarkList
{
    int mark;
    struct tagEqualMarkList *next;
    struct tagEqualMarkList *head;
    struct tagEqualMarkList *tail;

}EqualMarkList;

/* 映射关系 */
typedef struct tagRegionMap
{
    int *MarkNumber;            // 等价区域数目
    int *MarkPos;               // 等价区域在MarkSeq的起始索引
    int *MarkSeq;               // 等价区域
    int *MarkLabel;             // 与marklist等价(只有映射)

}RegionMap;

/* RGB宏定义 */
typedef enum tagImageRgb
{
    IMAGE_COLOR_BLUE = 0,
    IMAGE_COLOR_GREEN,
    IMAGE_COLOR_RED,
    IMAGE_COLOR_MAX

}IMAGE_RGB_E;

/* 方向定义 */
typedef enum tagImageDirect
{
    IMAGE_DIRECT_LEFT = 0,
    IMAGE_DIRECT_RIGHT,
    IMAGE_DIRECT_TOP,
    IMAGE_DIRECT_BOTTOM,
    IMAGE_DIRECT_MAX

}IMAGE_DIRECT_E;

/* 复数宏定义 */
typedef enum tagDataComplex
{
    DATA_COMPLEX_RE = 0,
    DATA_COMPLEX_IM,
    DATA_COMPLEX_MAX

}DATA_COMPLEX_E;

/* 向量距离类型定义 */
typedef enum tagVectorDist
{
    VECTOR_DIST_MANHATTAN = 1,
    VECTOR_DIST_EUCLIDEAN,
    VECTOR_DIST_MINKOWSKI,
    VECTOR_DIST_INF,
    VECTOR_DIST_MANHATTAN2,
    VECTOR_DIST_MAX

}VECTOR_DIST_E;

#endif
