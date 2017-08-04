/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageRegionFeature.h 			                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: 图像区域特征                                                 */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _IMAGE_REGION_FEATURE_H_
#define _IMAGE_REGION_FEATURE_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

#include "DataTypeConvert.h"
#include "ImageColor.h"
#include "VectorBasic.h"


#define FEA_NUM_STAT    4
#define FEA_NUM_GEO     11
#define FEA_NUM_LOC     4


#define FEA_Q_VEC               16
#define FEA_S_VEC               65536
#define FEA_Q_STAT              8
#define FEA_S_STAT              256

#define FEA_MAX_STAT_DIFF       0.125
#define FEA_MAX_STAT_DIFF_Q     8192
#define FEA_MAX_STAT_NUM        2048


// 矩
typedef struct tagMomentIntInfo
{
    int m00;        // 零阶矩
    int m01;        // x方向一阶矩
    int m10;        // y方向一阶矩
    int64 m11;      // xy方向的二阶矩
    int64 m02;      // x方向的二阶矩
    int64 m20;      // y方向的二阶矩

}MomentIntInfo;

// 区域中的直线
typedef struct tagImageRegionLine
{
    int rowcol;                         // 行或列
    int start;                          // 起始位置
    int end;                            // 截止位置
    int mark;                           // 所属区域标记

    struct tagImageRegionLine *next;

}ImageRegionLine;

// image line info
typedef struct tagImageLineInfo
{
    int *LineCount;             // 每个区域中直线个数
    int *LineNumber;            // 每行或列上直线个数
    int *LinePos;               // 每行或列上起始直线索引
    ImageRegionLine *LineList;  // 直线(按扫描方向存放)
    ImageRegionLine **LineHead; // 每个区域对应的直线链表头
    ImageRegionLine **LineTail; // 每个区域对应的直线链表尾

}ImageLineInfo;

// 图像统计信息
typedef struct tagIrfStatInfo
{
	double mean;        // 均值
	double var;         // 标准差
	double skew;        // 偏度系数
	double kurt;        // 峰度系数

}IrfStatInfo;

typedef struct tagIrfStatInfoInt
{
	int mean;       // 均值
	int var;        // 标准差
	int skew;       // 偏度系数
	int kurt;       // 峰度系数

}IrfStatInfoInt;

// 几何形状信息
typedef struct tagIrfGeoInfo
{
	double area;            // 面积
	double perimeter;       // 标准差
	double circularity1;    // 圆度1(周长面积比)
	double circularity2;    // 圆度2(矩计算)

	// 二阶中心矩
	double m11;
	double m02;
	double m20;

    // 椭圆惯性矩
    double longaxis;        // 长轴
    double shortaxis;       // 短轴
    double eccentricity;    // 偏心率
    double angle;           // 方向角

}IrfGeoInfo;

// 位置信息: 以图像中心为原点
typedef struct tagIrfLocationInfo
{
	double x;
	double y;
	double d;   // 距离
	double a;   // 方向

}IrfLocationInfo;

// 特征提取结构
typedef struct tagIrfFeaInfo
{
    // 参数
    BOOL isRgbStat;
    BOOL isGeo;
    BOOL isLoc;

    // 图像信息
    Matrix_S grayImage;
    Matrix_S matseq[3];

    // 区域信息
    int *pointnum;
    int *pointpos;
    POINT3D_S *point;
    ImageLineInfo horline;
    ImageLineInfo verline;
    MomentIntInfo *mom;
    double *color;
    IMAGE32_S edgeImage;
    int *edgenum1;          // 重复边缘
    int *edgenum2;          // 不重复边缘

    // 统计特征
    IrfStatInfo *stat;
    IrfStatInfoInt *statint;
    IrfGeoInfo *geo;
    IrfLocationInfo *loc;

    // 图像内存信息
    int maxHeight;
    int maxWidth;
    int maxChannel;
    int maxRegionnum;

}IrfFeaInfo;


// 接口函数
extern void IrfFeaturePara(OUT IrfFeaInfo *feainfo);
extern void IrfFeatureInit(OUT IrfFeaInfo *feainfo);
extern void IrfFeatureDestroy(OUT IrfFeaInfo *feainfo);
extern void IrfFeatureMemoryAdapt(OUT IrfFeaInfo *feainfo, 
                                  IN int height, 
                                  IN int width,
                                  IN int regionnum);
extern void IrfFeatureProc(IN IrfFeaInfo *feainfo,
                           IN IMAGE32_S *markImage,
                           IN Matrix_S *colorImage, 
                           IN int regionnum,
                           OUT double *feature,
                           IN int feanum);
extern int IrfFeatureNum(IN IrfFeaInfo *feainfo, IN int channel);


// 内部函数
extern void IrfStat2Vector(IN IrfStatInfo *stat, 
                           IN int regionnum,
                           IN double *feature,
                           IN int step);
extern void IrfGeo2Vector(IN IrfGeoInfo *geo, 
                          IN int regionnum,
                          IN double *feature,
                          IN int step);
extern void IrfLoc2Vector(IN IrfLocationInfo *loc, 
                          IN int regionnum,
                          IN double *feature,
                          IN int step);

extern void IrfFeatureGeoLoc(IN int *edgenum,
                             IN MomentIntInfo *geoMoment,
                             OUT IrfGeoInfo *geoStat,
                             OUT IrfLocationInfo *locStat,
                             IN int height,
                             IN int width,
                             IN int regionnum);
extern void IrfFeatureStatistics(IN double *image,
                                 IN POINT3D_S *point,
                                 IN int *pointnum,
                                 IN int regionnum,
                                 OUT IrfStatInfo *stat);
extern void IrfStatOne(IN double *image, 
                       IN POINT3D_S *point,
                       IN int pointnum,
                       OUT IrfStatInfo *stat);


extern void IrfRegionInfo(IN IrfFeaInfo *feainfo,
                          IN IMAGE32_S *markImage,
                          IN int regionnum);
extern void IrfArea(IN IMAGE32_S *markImage, OUT int *area, IN int regionnum);
extern void IrfPoint(IN IMAGE32_S *markImage, 
                     IN int *pos,
                     OUT POINT3D_S *point,
                     OUT int *area, 
                     IN int regionnum);
extern void IrfLine(IN IMAGE32_S *markImage,
                    IN int regionNum,
                    OUT ImageLineInfo *line, 
                    IN int direct);
extern void IrfEdgeFromLine(IN ImageLineInfo *line, 
                            IN int regionNum,
                            INOUT IMAGE32_S *edgeImage,
                            OUT int *edgenum1,
                            OUT int *edgenum2,
                            IN int direct);

extern void IrfGeomomentFromLine(IN ImageLineInfo *line, 
                                 OUT MomentIntInfo *geoMoment, 
                                 IN int regionNum,
                                 IN int height);

extern void RegionLineInit(OUT ImageLineInfo *L, 
                           IN int maxVertexNum,
                           IN int maxLineNum,
                           IN int maxLineGroup);
extern void RegionLineDestroy(OUT ImageLineInfo *L, 
                              IN int maxVertexNum,
                              IN int maxLineNum,
                              IN int maxLineGroup);

extern void HistAccum(IN int *h1, OUT int *h2, IN int n);



extern void IrfFeatureProcInt(IN IrfFeaInfo *feainfo,
                              IN IMAGE32_S *markImage,
                              IN IMAGE_S *colorImage, 
                              IN int regionnum,
                              OUT int *feature,
                              IN int feanum);
extern void IrfFeatureStatisticsInt(IN PIXEL *image,
                                    IN int channel,
                                    IN POINT3D_S *point,
                                    IN int *pointnum,
                                    IN int regionnum,
                                    OUT IrfStatInfoInt *stat);
extern void IrfStatOneInt(IN PIXEL *image, 
                          IN int channel,
                          IN POINT3D_S *point,
                          IN int pointnum,
                          OUT IrfStatInfoInt *stat);
extern void IrfStat2VectorInt(IN IrfStatInfoInt *stat, 
                              IN int regionnum,
                              IN int *feature,
                              IN int step);
extern void IrfGeo2VectorInt(IN IrfGeoInfo *geo, 
                             IN int regionnum,
                             IN int *feature,
                             IN int step);
extern void IrfLoc2VectorInt(IN IrfLocationInfo *loc, 
                             IN int regionnum,
                             IN int *feature,
                             IN int step);


#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
