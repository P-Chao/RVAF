/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageIer.h 	                                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2013/07/02                                                          */
/*                                                                           */
/* Description: 图像分割算法 Ier                                             */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifndef _IMAGE_IER_H_
#define _IMAGE_IER_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#include "VectorBasic.h"

// 类型、函数定义
typedef double              floatier;
#define IER_INF             DOUBLE_INF
#define VectorDistIer       VectorDistDouble
#define VectorPlusIer       VectorPlusDouble
#define VectorSubIer        VectorSubDouble
#define VectorMulConstIer   VectorMulConstDouble


#define IERI_INF            INT_INF
#define VectorDistIeri      VectorDistInt
#define VectorPlusIeri      VectorPlusInt
#define VectorSubIeri       VectorSubInt
#define VectorMulConstIeri  VectorMulConstInt
/*
    Q 因子说明
    Q(distc) = Q(distxy)+Q(distwxy)
    Q(distc) = 2*Q(avgc), Q(distxy) = 2*Q(avgxy)
    max(distc) = 16 >>> Q(distc) = 31-16 = 15 >>> Q(avgc) = 7
    >>> Q(avgxy) = 3, Q(distwxy) = 8
*/
#define IER_Q_AVGC              7
#define IER_Q_AVGXY             3
#define IER_Q_AVGWXY            8
#define IER_S_AVGC              128
#define IER_S_AVGXY             8
#define IER_S_AVGWXY            256

#define IER_MAX_NBNUM           5


// 边缘像素队列
typedef struct tagIerEdgeQueue
{
    POINT3D_S *curQueue;
    POINT3D_S *nextQueue;
    int curQueueNum;
    PIXEL *queueflag;
    int *labelflag;
    int *changeflag;
    int *changeIndex;
    int changeNum;

    int *nbIndex;
    int *nbNumber;

}IerEdgeQueue;

// 小区域连通处理
typedef struct tagIerConnectInfo
{
    int *conx;
    int *cony;
    int *conxy;

}IerConnectInfo;

typedef struct tagMeshSplitInfo
{
    int S;
    int n;
    int n1;
    int n2;

}MeshSplitInfo;

// ier结构
typedef struct tagIerInfo
{
    // 外部输入信息
    int width;                  // 图像宽度
    int height;                 // 图像高度
    int channel;                // 彩色图像通道数
    floatier *imagecolor;       // 彩色图像
    PIXEL *imagecolori;         // 彩色图像
    
    // 参数
    int optint;
    int K;                  // Superpixel number
    int S;                  // sqrt(Superpixel size) 聚类中心采样间隔
    floatier normC;         // 颜色归一化因子 (1-40)
    floatier normS;         // 空间归一化因子
    int maxiter;

    // 输出标记信息
    int *label;

    // 聚类中心
    int num;
    int *sumn;
    int *sumx;
    int *sumy;

    floatier *sumc;
    floatier *avgc;
    floatier *avgx;
    floatier *avgy;
    floatier wxy;
    
    int *avgxi;
    int *avgyi;
    int *sumci;
    int *avgci;
    int wxyi;

    // 内部临时内存
    int *newlabel;
    IerEdgeQueue edgeQueue;
    IerConnectInfo connect;

    // 内存相关
    int maxK;                   // 最大分割区域数目

}IerInfo;


extern void ImageIerPara(OUT IerInfo *ier);
extern void ImageIerInit(OUT IerInfo *ier);
extern void ImageIerDestroy(OUT IerInfo *ier);
extern void IerMemoryAdapt0(IN IerInfo *ier, IN int K, IN int type);
extern void IerStatInit(OUT IerInfo *ier);
extern void IerStatDestroy(OUT IerInfo *ier);
extern void IerEdgeQueueInit(OUT IerEdgeQueue *Q, IN int size, IN int K);
extern void IerEdgeQueueDestroy(OUT IerEdgeQueue *Q, IN int size, IN int K);
extern void IerEdgeQueueInit0(OUT IerEdgeQueue *Q, IN int K);
extern void IerEdgeQueueDestroy0(OUT IerEdgeQueue *Q, IN int K);
extern void IerConnectInfoInit(OUT IerConnectInfo *connect, IN int size, IN int K);
extern void IerConnectInfoDestroy(OUT IerConnectInfo *connect, IN int size, IN int K);


extern void IerPixelInit(IN IerInfo *ier);
extern void IerPixelRefine(IN IerInfo *ier);
extern void IerWeight(IN IerInfo *ier);
extern void IerEdgeInitDetect(IN IerInfo *ier, IN int off);
extern void IerEdgeReDetect(IN IerInfo *ier);
extern void IerEdgeRefine(IN IerInfo *ier);
extern void IerEdgeRefineInt(IN IerInfo *ier);
extern void IerEnforceConnectivity(IN IerInfo *ier);


extern void MeshSplitSize(INOUT MeshSplitInfo *sp, IN int S0, IN int width);
extern void IerMeshStat(INOUT IerInfo *ier, 
                        IN int Sx,
                        IN int Sy,
                        IN int nx,
                        IN int ny,
                        IN int nx1,
                        IN int ny1);
extern void IerMeshStatInt(INOUT IerInfo *ier, 
                           IN int Sx,
                           IN int Sy,
                           IN int nx,
                           IN int ny,
                           IN int nx1,
                           IN int ny1);

extern void IerFlagInit(PIXEL *label, int width, int height, PIXEL neg);
extern void IerLabelInit(int *label, int width, int height, int neg);
extern void IerLabelExtend(int *inl, int *outl, int width, int height);
extern void IerLabelDeextend(int *inl, int *outl, int width, int height);


extern void ImageSegShow(IN IMAGE_S *srcImage, 
                         IN int *flag, 
                         IN PIXEL *contour,
                         OUT IMAGE_S *showImage);

extern void IerEdgeRefineNeighbor1(IN IerInfo *ier);
extern void IerEdgeRefineIntChannel1(IN IerInfo *ier);
extern void IerEdgeRefineIntColor1(IN IerInfo *ier);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
