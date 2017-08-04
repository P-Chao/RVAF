/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageRegionFeature.c 			                                 */
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

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "ImageRegionFeature.h"
#include "math.h"

void IrfFeaturePara(OUT IrfFeaInfo *feainfo)
{
    feainfo->maxHeight = 500;
    feainfo->maxWidth = 500;
    feainfo->maxChannel = 3;
    feainfo->maxRegionnum = 1000;

    feainfo->isRgbStat = TRUE;
    feainfo->isGeo = TRUE;
    feainfo->isLoc = TRUE;
}

void IrfFeatureInit(OUT IrfFeaInfo *feainfo)
{
    int height = feainfo->maxHeight;
    int width = feainfo->maxWidth;
    int channel = feainfo->maxChannel;
    int regionnum = feainfo->maxRegionnum;
    int imagesize = height*width;

    feainfo->grayImage.data = MallocType(double, imagesize);
    MatrixSeqInit(feainfo->matseq, height, width, channel);

    feainfo->pointnum = MallocType(int, regionnum);
    feainfo->pointpos = MallocType(int, regionnum+1);
    feainfo->point = MallocType(POINT3D_S, imagesize);
    feainfo->mom = MallocType(MomentIntInfo, regionnum);
    feainfo->color = MallocType(double, channel*regionnum);
    RegionLineInit(&feainfo->horline, regionnum, imagesize, height);
    RegionLineInit(&feainfo->verline, regionnum, imagesize, width);
    feainfo->edgeImage.data = MallocType(int, imagesize);
    feainfo->edgenum1 = MallocType(int, regionnum);
    feainfo->edgenum2 = MallocType(int, regionnum);

    feainfo->stat = MallocType(IrfStatInfo, regionnum);
    feainfo->statint = MallocType(IrfStatInfoInt, regionnum);
    feainfo->geo = MallocType(IrfGeoInfo, regionnum);
    feainfo->loc = MallocType(IrfLocationInfo, regionnum);

    return;
}
void IrfFeatureDestroy(OUT IrfFeaInfo *feainfo)
{
    int height = feainfo->maxHeight;
    int width = feainfo->maxWidth;
    int channel = feainfo->maxChannel;
    int regionnum = feainfo->maxRegionnum;
    int imagesize = height*width;
    
    FreeType(feainfo->grayImage.data, double, imagesize);
    MatrixSeqDestroy(feainfo->matseq);

    FreeType(feainfo->pointnum, int, regionnum);
    FreeType(feainfo->pointpos, int, regionnum+1);
    FreeType(feainfo->point, POINT3D_S, imagesize);
    FreeType(feainfo->mom, MomentIntInfo, regionnum);
    FreeType(feainfo->color, double, channel*regionnum);
    RegionLineDestroy(&feainfo->horline, regionnum, imagesize, height);
    RegionLineDestroy(&feainfo->verline, regionnum, imagesize, height);
    FreeType(feainfo->edgeImage.data, int, imagesize);
    FreeType(feainfo->edgenum1, int, regionnum);
    FreeType(feainfo->edgenum2, int, regionnum);

    FreeType(feainfo->stat, IrfStatInfo, regionnum);
    FreeType(feainfo->statint, IrfStatInfoInt, regionnum);
    FreeType(feainfo->geo, IrfGeoInfo, regionnum);
    FreeType(feainfo->loc, IrfLocationInfo, regionnum);
}
void IrfFeatureMemoryAdapt(OUT IrfFeaInfo *feainfo, 
                           IN int height, 
                           IN int width,
                           IN int regionnum)
{
    int maxRegionNum = feainfo->maxRegionnum;
    int maxHeight = feainfo->maxHeight;
    int maxWidth = feainfo->maxWidth;

    if (regionnum > maxRegionNum || height > maxHeight || width > maxWidth)
    {
        IrfFeatureDestroy(feainfo);
        feainfo->maxRegionnum = (regionnum > maxRegionNum) ? regionnum : maxRegionNum;
        feainfo->maxHeight = (height > maxHeight) ? height : maxHeight;
        feainfo->maxWidth = (width > maxWidth) ? width : maxWidth;
        IrfFeatureInit(feainfo);
    }
}

void IrfFeatureProc(IN IrfFeaInfo *feainfo,
                    IN IMAGE32_S *markImage,
                    IN Matrix_S *colorImage, 
                    IN int regionnum,
                    OUT double *feature,
                    IN int feanum)
{
    double *fea = feature;

    int height = colorImage->height;
    int width = colorImage->width;
    int channel = colorImage->channel;
    int imagesize = height*width;

    int *pointnum = feainfo->pointnum;
    POINT3D_S *point = feainfo->point;

    IrfStatInfo *stat = feainfo->stat;
    IrfGeoInfo *geo = feainfo->geo;
    IrfLocationInfo *loc = feainfo->loc;

    int i;
    double *image;

    // 提取区域信息
    IrfRegionInfo(feainfo, markImage, regionnum);

////////////////////////////////////////////////////////////////////////////////////////////////////

//                                     颜色/灰度特征

////////////////////////////////////////////////////////////////////////////////////////////////////
   
    if (channel == 3)
    {
        Matrix_S *matseq = feainfo->matseq;

        // RGB空间统计
        if (feainfo->isRgbStat == TRUE)
        {
            MatrixRgb2Seq(colorImage, matseq);
            for (i = 0;i < channel;i++)
            {
                image = matseq[i].data;
                VectorMulConstDouble(image, image, 1.0/256, imagesize);
                IrfFeatureStatistics(image, point, pointnum, regionnum, stat);
                IrfStat2Vector(stat, regionnum, fea, feanum);
                fea += FEA_NUM_STAT;
            }
        }
    }
    else
    {
        Matrix_S *grayImage = &feainfo->grayImage;

        VectorMulConstDouble(colorImage->data, grayImage->data, 1.0/256, imagesize);
        if (feainfo->isRgbStat == TRUE)
        {
            IrfFeatureStatistics(grayImage->data, point, pointnum, regionnum, stat);
            IrfStat2Vector(stat, regionnum, fea, feanum);
            fea += FEA_NUM_STAT;
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////

//                                    几何位置特征

////////////////////////////////////////////////////////////////////////////////////////////////////

    IrfFeatureGeoLoc(feainfo->edgenum2, feainfo->mom, geo, loc, height, width, regionnum);
    if (feainfo->isGeo == TRUE)
    {
        IrfGeo2Vector(geo, regionnum, fea, feanum);
        fea += FEA_NUM_GEO;
    }
    if (feainfo->isLoc == TRUE)
    {
        IrfLoc2Vector(loc, regionnum, fea, feanum);
        fea += FEA_NUM_LOC;
    }

    return;
}

int IrfFeatureNum(IN IrfFeaInfo *feainfo, IN int channel)
{
    int feanum = 0;

    if (feainfo->isRgbStat == TRUE)
    {
        feanum += channel*FEA_NUM_STAT;
    }
    if (feainfo->isGeo == TRUE)
    {
        feanum += FEA_NUM_GEO;
    }
    if (feainfo->isLoc == TRUE)
    {
        feanum += FEA_NUM_LOC;
    }
    
    return feanum;
}

void IrfStat2Vector(IN IrfStatInfo *stat, 
                    IN int regionnum,
                    IN double *feature,
                    IN int step)
{
    int i;
    double *f;
    IrfStatInfo *s;

    s = stat;
    f = feature;
    for (i = 0;i < regionnum;i++, f += step, s++)
    {
        f[0] = s->mean;
        f[1] = s->var;
        f[2] = s->skew;
        f[3] = s->kurt;
    }
}

void IrfGeo2Vector(IN IrfGeoInfo *geo, 
                   IN int regionnum,
                   IN double *feature,
                   IN int step)
{
    int i;
    double *f;
    IrfGeoInfo *g;

    g = geo;
    f = feature;
    for (i = 0;i < regionnum;i++, f += step, g++)
    {
        f[0] = g->area;
        f[1] = g->perimeter;
        f[2] = g->circularity1;
        f[3] = g->circularity2;

        f[4] = g->m11;
        f[5] = g->m02;
        f[6] = g->m20;

        f[7] = g->longaxis;
        f[8] = g->shortaxis;
        f[9] = g->eccentricity;
        f[10] = g->angle;
    }
}

void IrfLoc2Vector(IN IrfLocationInfo *loc, 
                   IN int regionnum,
                   IN double *feature,
                   IN int step)
{
    int i;
    double *f;
    IrfLocationInfo *l;

    l = loc;
    f = feature;
    for (i = 0;i < regionnum;i++, f += step, l++)
    {
        f[0] = l->x;
        f[1] = l->y;
        f[2] = l->d;
        f[3] = l->a;
    }
}

void IrfFeatureGeoLoc(IN int *edgenum,
                      IN MomentIntInfo *geoMoment,
                      OUT IrfGeoInfo *geoStat,
                      OUT IrfLocationInfo *locStat,
                      IN int height,
                      IN int width,
                      IN int regionnum)
{
    int i;
    IrfGeoInfo *geo;
    IrfLocationInfo *loc;
    MomentIntInfo *mom;
    double norma, norml;
    double normh, normw;
    double perimeter;
    double m00, m01, m10;
    double m11, m02, m20;
    double norm;
    double u01, u10;
    double u11, u02, u20;
    double temp, temp1, temp2;

    norma = 1.0/(double)(height*width);
    norml = sqrt(norma);
    normh = 1.0/(double)height;
    normw = 1.0/(double)width;

    // 计算统计特征
    mom = geoMoment;
    geo = geoStat;
    loc = locStat;    
    for (i = 0;i < regionnum;i++, mom++, geo++, loc++)
    {
        if (mom->m00 == 0)
        {
            memset(geo, 0, sizeof(IrfGeoInfo));
            memset(loc, 0, sizeof(IrfGeoInfo));
            continue;
        }
        
        // 几何矩
        m00 = (double)mom->m00;
        m01 = (double)mom->m01;
        m10 = (double)mom->m10;
        m11 = (double)mom->m11;
        m02 = (double)mom->m02;
        m20 = (double)mom->m20;
        perimeter = (double)edgenum[i];

        // 中心矩
        norm = 1.0/m00;
        u01 = m01*norm;
        u10 = m10*norm;
        u02 = m02*norm - u01*u01;
        u20 = m20*norm - u10*u10;
        u11 = m11*norm - u10*u01;

        // 面积
        geo->area = m00 * norma;

        // 周长
        geo->perimeter = perimeter * norml;

        // 圆度1
        temp = 4*MATH_PI*m00/(perimeter*perimeter);
        geo->circularity1 = (1.0 < temp) ? 1.0 : temp;

        // 圆度2
        temp = m00/(2*MATH_PI*(u02+u20));
        geo->circularity2 = (1.0 < temp) ? 1.0 : temp;

        // 中心矩
        geo->m02 = u02*norma;
        geo->m20 = u20*norma;
        geo->m11 = u11*norma;

        // 椭圆惯性矩 && 方向角
        temp = u02 - u20;
        temp2 = u02 + u20;
        temp1 = sqrt(0.5*(4*u11*u11+temp*temp));
        if (fabs(temp) < FLOAT_EPS) geo->angle = MATH_PI/4;
        else                        geo->angle = 0.5*atan(u11/temp);
        geo->longaxis = 2*sqrt(temp2 + temp1) * norml;
        temp = temp2 - temp1;
        if (temp <= FLOAT_EPS)
        {
            geo->shortaxis = 0;
            geo->eccentricity = 100;
        }
        else
        {
            geo->shortaxis = 2*sqrt(temp2 - temp1) * norml;
            geo->eccentricity = geo->longaxis/geo->shortaxis;
        }

        // 位置 (以图像中心为原点)
        temp1 = 2*(u01 * normw - 0.5);
        temp2 = 2*(u10 * normh - 0.5);
        loc->x = temp1;
        loc->y = temp2;

        // 距离和角度
        arctan(temp1, temp2, temp);
        loc->d = temp1*temp1 + temp2*temp2;
        loc->a = temp;
    }

    return;
}

void IrfFeatureStatistics(IN double *image,
                          IN POINT3D_S *point,
                          IN int *pointnum,
                          IN int regionnum,
                          OUT IrfStatInfo *stat)
{
    int i, n;
    POINT3D_S *p;
    IrfStatInfo *s;

    s = stat;
    p = point;
    for (i = 0;i < regionnum;i++, s++)
    {
        n = pointnum[i];
        IrfStatOne(image, p, n, s);
        p += n;
    }

    return;
}

void IrfStatOne(IN double *image, 
                IN POINT3D_S *point,
                IN int pointnum,
                OUT IrfStatInfo *stat)
{
    int i, j, n;
    POINT3D_S *p;
    double v, sum, svar;
    double mean, var, skew, kurt;
    double d1, d2, d3, d4;

    if (pointnum <= 0)
    {
        memset(stat, 0, sizeof(IrfStatInfo));
        return;
    }

    // 统计均值
    sum = 0;
    p = point;
    for(i = 0;i < pointnum;i++, p++)
    {
        j = p->s;
        sum += image[j];
    }
    mean = sum/pointnum;

    // 统计其他统计量
    var = 0;
    skew = 0;
    kurt = 0;
    p = point;
    for(i = 0;i < pointnum;i++, p++)
    {
        j = p->s;
        v = image[j];
        d1 = v - mean;
        d1 = fabs(d1);
		d1 = (FEA_MAX_STAT_DIFF < d1) ? FEA_MAX_STAT_DIFF : d1;
        d2 = d1*d1;
        d3 = d1*d2;
        d4 = d2*d2;
        var += d2;
        skew += d3;
        kurt += d4;
    }
    n = pointnum-1;
    if (n == 0)     var = 0;
    else            var /= n;
    svar = sqrt(var);

    stat->mean = mean;
    stat->var = svar;
    if (svar < FLOAT_EPS)
    {
        stat->skew = 0;
        stat->kurt = 0;
    }
    else
    {
        stat->skew = skew / (pointnum * var * svar);
        stat->kurt = kurt / (pointnum * var * var);
    }

    return;
}

void IrfRegionInfo(IN IrfFeaInfo *feainfo,
                   IN IMAGE32_S *markImage,
                   IN int regionnum)
{
    int height = markImage->height;
    int width = markImage->width;
    int size = height * width;

    int *pointnum = feainfo->pointnum;
    int *pointpos = feainfo->pointpos;
    POINT3D_S *point = feainfo->point;
    ImageLineInfo *horline = &feainfo->horline;
    ImageLineInfo *verline = &feainfo->verline;
    MomentIntInfo *mom = feainfo->mom;
    IMAGE32_S *edgeImage = &feainfo->edgeImage;
    int *edgenum1 = feainfo->edgenum1;
    int *edgenum2 = feainfo->edgenum2;

    // 提取区域点
    IrfArea(markImage, pointnum, regionnum);
    pointpos[0] = 0;
    HistAccum(pointnum, pointpos + 1, regionnum);
    IrfPoint(markImage, pointpos, point, pointnum, regionnum);

    // 提取直线
    IrfLine(markImage, regionnum, horline, TRUE);
    IrfLine(markImage, regionnum, verline, FALSE);

    // 统计边缘
    edgeImage->height = height;
    edgeImage->width = width;
    memset(edgeImage->data, 0, size*sizeof(int));
    memset(edgenum1, 0, regionnum*sizeof(int));
    memset(edgenum2, 0, regionnum*sizeof(int));
    IrfEdgeFromLine(horline, regionnum, edgeImage, edgenum1, edgenum2, TRUE);
    IrfEdgeFromLine(verline, regionnum, edgeImage, edgenum1, edgenum2, FALSE);

    // 统计几何矩
    IrfGeomomentFromLine(horline, mom, regionnum, height);

    return;
}

void IrfArea(IN IMAGE32_S *markImage, OUT int *area, IN int regionnum)
{
    int height = markImage->height;
    int width = markImage->width;
    int *mark = markImage->data;
    int i, j, k;

    // 统计每个区域点的个数
    memset(area, 0, regionnum*sizeof(int));
    for(i = 0;i < height;i++, mark += width)
	{
		for(j = 0;j < width;j++)
        {
            k = mark[j];
            area[k]++;
        }
    }

    return;
}

void IrfPoint(IN IMAGE32_S *markImage, 
              IN int *pos,
              OUT POINT3D_S *point,
              OUT int *area, 
              IN int regionnum)
{
    int height = markImage->height;
    int width = markImage->width;
    int *mark = markImage->data;

    int i, j, k;
    int flag;
    int offset;
    POINT3D_S *p;

    flag = FALSE;
    if (area == NULL)
    {
        flag = TRUE;
        area = MallocType(int, regionnum);
    }

    // 统计每个区域点的个数
    memset(area, 0, regionnum*sizeof(int));
    for(i = 0;i < height;i++, mark += width)
	{
	    offset = i*width;
		for(j = 0;j < width;j++)
        {
            k = mark[j];
            p = point + pos[k] + area[k];
            p->x = j;
            p->y = i;
            p->s = offset+j;
            area[k]++;
        }
    }

    if (flag == TRUE)
    {
        FreeType(area, int, regionnum);
    }

    return;
}

void IrfLine(IN IMAGE32_S *markImage,
             IN int regionNum,
             OUT ImageLineInfo *line, 
             IN int direct)
{
    int height = markImage->height;
    int width = markImage->width;
    int *mark = markImage->data;
    ImageRegionLine *LineList = line->LineList;
    ImageRegionLine **LineHead = line->LineHead;
    ImageRegionLine **LineTail = line->LineTail;
    int *LineCount = line->LineCount;
    int *LineNumber = line->LineNumber;
    int *LinePos = line->LinePos;

    int i, j, k, l;
    int inc, inch;
    int leni, lenj;
    int n;
    int *src, *srchead;
    ImageRegionLine *Line;

#define RegionNewLine()                                         \
{                                                               \
    Line->start = j;                                            \
    Line->rowcol = i;                                           \
    Line->mark = k;                                             \
    if (LineHead[k] == NULL)    LineHead[k] = Line;             \
    else                        LineTail[k]->next = Line;       \
    LineCount[k]++;                                             \
    LineTail[k] = Line;                                         \
}

    // 调整方向
    if (direct == TRUE)
    {
        // 水平方向
        inc = 1;
        inch = width;
        leni = height;
        lenj = width;
    }
    else
    {
        // 垂直方向
        inc = width;
        inch = 1;
        leni = width;
        lenj = height;
    }

    // 提取每个区域点的位置
    memset(LineHead, NULL, regionNum*sizeof(ImageRegionLine *));
    memset(LineCount, 0, regionNum*sizeof(int));
    memset(LineNumber, 0, leni*sizeof(int));
    LinePos[0] = 0;
    Line = LineList;

    srchead = mark;
    for(i = 0;i < leni;i++, srchead += inch)
	{
	    // 首条直线
	    src = srchead;
	    n = 1;
	    j = 0;
        k = *src;
	    RegionNewLine();

		for(j++, src += inc;j < lenj;j++, src += inc)
		{
		    l = *src;
            if (k != l)
            {
                Line->end = j-1;
                Line++;
                n++;

                // 新的直线
                k = l;
        	    RegionNewLine();
            }
		}
		Line->end = lenj-1;
        Line++;

		LineNumber[i] = n;
		LinePos[i+1] = LinePos[i]+n;
    }

    // 尾部next置空
    for(i = 0;i < regionNum;i++)
    {
        if (LineCount[i] == 0)
        {
            LineTail[i] = NULL;
            continue;
        }
        LineTail[i]->next = NULL;
    }

#undef RegionNewLine

    return;
}

void IrfEdgeFromLine(IN ImageLineInfo *line, 
                     IN int regionNum,
                     INOUT IMAGE32_S *edgeImage,
                     OUT int *edgenum1,
                     OUT int *edgenum2,
                     IN int direct)
{
    int height = edgeImage->height;
    int width = edgeImage->width;
    int *edgemap = edgeImage->data;
    ImageRegionLine *LineList = line->LineList;
    int *LineNumber = line->LineNumber;
    int *LinePos = line->LinePos;

    int i, j, k, n;
    int s, e;
    int inc, inch, len;
    int replicate;
    int *edge;
    ImageRegionLine *Line;

    // 调整方向
    if (direct == TRUE)
    {
        // 水平方向
        inc = 1;
        inch = width;
        len = height;
    }
    else
    {
        // 垂直方向
        inc = width;
        inch = 1;
        len = width;
    }

    for(i = 0;i < len;i++, edgemap += inch)
    {
        n = LineNumber[i];
        Line = LineList + LinePos[i];
        for(j = 0;j < n;j++, Line++)
        {
            k = Line->mark;
            edgenum1[k] += 2;

            s = Line->start;
            edge = edgemap + s*inc;
            replicate = *edge;
            if (replicate == 0) edgenum2[k]++;
            *edge = replicate+1;

            e = Line->end;
            edge = edgemap + e*inc;
            replicate = *edge;
            if (replicate == 0) edgenum2[k]++;
            *edge = replicate+1;
        }
    }

    return;
}

void IrfGeomomentFromLine(IN ImageLineInfo *line, 
                          OUT MomentIntInfo *geoMoment, 
                          IN int regionNum,
                          IN int height)
{
    ImageRegionLine *LineList = line->LineList;
    int *LineNumber = line->LineNumber;
    int *LinePos = line->LinePos;

    int i, j, k, n;
    int s, e;
    int m00, m01, m02;
    ImageRegionLine *Line;
    MomentIntInfo *geo;

    memset(geoMoment, 0, regionNum*sizeof(MomentIntInfo));
    for(i = 0;i < height;i++)
    {
        n = LineNumber[i];
        Line = LineList + LinePos[i];
        for(j = 0;j < n;j++, Line++)
        {
            s = Line->start;
            e = Line->end;
           
            m00 = e - s + 1;
            m01 = (s+e)*m00/2;
            m02 = squareSum(e) - squareSum(s-1);

            k = Line->mark; 
            geo = geoMoment + k;
            geo->m00 += m00;
            geo->m01 += m01;
            geo->m10 += m00*i;
            geo->m11 += m01*i;
            geo->m02 += m02;
            geo->m20 += m00*i*i;
        }
    }

	return;
}

void RegionLineInit(OUT ImageLineInfo *L, 
                    IN int maxVertexNum,
                    IN int maxLineNum,
                    IN int maxLineGroup)
{
    L->LineNumber = MallocType(int, maxLineGroup);
    L->LinePos = MallocType(int, maxLineGroup+1);
    L->LineCount = MallocType(int, maxVertexNum);
    L->LineList = MallocType(ImageRegionLine, maxLineNum);
    L->LineHead = MallocType(ImageRegionLine *, maxVertexNum);
    L->LineTail = MallocType(ImageRegionLine *, maxVertexNum);
}

void RegionLineDestroy(OUT ImageLineInfo *L, 
                       IN int maxVertexNum,
                       IN int maxLineNum,
                       IN int maxLineGroup)
{
    FreeType(L->LineNumber, int, maxLineGroup);
    FreeType(L->LinePos, int, maxLineGroup+1);
    FreeType(L->LineCount, int, maxVertexNum);
    FreeType(L->LineList, ImageRegionLine, maxLineNum);
    FreeType(L->LineHead, ImageRegionLine *, maxVertexNum);
    FreeType(L->LineTail, ImageRegionLine *, maxVertexNum);
}

void HistAccum(IN int *h1, OUT int *h2, IN int n)
{
	int i;
	int sum;

    sum = h2[0] = h1[0];
	for (i = 1;i < n;i++)
	{
	    sum += h1[i];
		h2[i] = sum;
	}

	return;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

//                                     整型数据运算

////////////////////////////////////////////////////////////////////////////////////////////////////

void IrfFeatureProcInt(IN IrfFeaInfo *feainfo,
                       IN IMAGE32_S *markImage,
                       IN IMAGE_S *colorImage, 
                       IN int regionnum,
                       OUT int *feature,
                       IN int feanum)
{
    int *fea = feature;

    int height = colorImage->height;
    int width = colorImage->width;
    int channel = colorImage->channel;

    IrfStatInfoInt *stat = feainfo->statint;
    IrfGeoInfo *geo = feainfo->geo;
    IrfLocationInfo *loc = feainfo->loc;

    // 提取区域信息
    IrfRegionInfo(feainfo, markImage, regionnum);

////////////////////////////////////////////////////////////////////////////////////////////////////

//                                     颜色/灰度特征

////////////////////////////////////////////////////////////////////////////////////////////////////

    if (feainfo->isRgbStat == TRUE)
    {
        PIXEL *image = colorImage->data;
        int *pointnum = feainfo->pointnum;
        POINT3D_S *point = feainfo->point;

        if (channel == 3)
        {
            int i;
            for (i = 0;i < channel;i++)
            {
                IrfFeatureStatisticsInt(image + i, channel, point, pointnum, regionnum, stat);
                IrfStat2VectorInt(stat, regionnum, fea, feanum);
                fea += FEA_NUM_STAT;
            }
        }
        else
        {
            IrfFeatureStatisticsInt(image, channel, point, pointnum, regionnum, stat);
            IrfStat2VectorInt(stat, regionnum, fea, feanum);
            fea += FEA_NUM_STAT;
        }
    }
    
////////////////////////////////////////////////////////////////////////////////////////////////////

//                                    几何位置特征

////////////////////////////////////////////////////////////////////////////////////////////////////

    IrfFeatureGeoLoc(feainfo->edgenum2, feainfo->mom, geo, loc, height, width, regionnum);
    if (feainfo->isGeo == TRUE)
    {
        IrfGeo2VectorInt(geo, regionnum, fea, feanum);
        fea += FEA_NUM_GEO;
    }
    if (feainfo->isLoc == TRUE)
    {
        IrfLoc2VectorInt(loc, regionnum, fea, feanum);
        fea += FEA_NUM_LOC;
    }

    return;
}


void IrfFeatureStatisticsInt(IN PIXEL *image,
                             IN int channel,
                             IN POINT3D_S *point,
                             IN int *pointnum,
                             IN int regionnum,
                             OUT IrfStatInfoInt *stat)
{
    int i, n;
    POINT3D_S *p;
    IrfStatInfoInt *s;

    s = stat;
    p = point;
    for (i = 0;i < regionnum;i++, s++)
    {
        n = pointnum[i];
        IrfStatOneInt(image, channel, p, n, s);
        p += n;
    }    

    return;
}

void IrfStatOneInt(IN PIXEL *image, 
                   IN int channel,
                   IN POINT3D_S *point,
                   IN int pointnum,
                   OUT IrfStatInfoInt *stat)
{
    int i, j, n;
    POINT3D_S *p;

    int v, d;
    int sum, mean;
    int64 var, skew, kurt;
    int64 d1, d2, d3, d4;

    int leftnum, curnum;
    double varf, svar;
    double skewf, kurtf;

    if (pointnum <= 0)
    {
        memset(stat, 0, sizeof(IrfStatInfo));
        return;
    }

    // 统计均值
    sum = 0;
    p = point;
    for(i = 0;i < pointnum;i++, p++)
    {
        j = p->s * channel;
        sum += image[j];
    }
    mean = (sum << FEA_Q_STAT) / pointnum;
    stat->mean = mean;

    // 统计其他统计量
    var = 0;
    skewf = 0;
    kurtf = 0;
    p = point;
    leftnum = pointnum;
    while (leftnum > 0)
    {
        skew = 0;
        kurt = 0;
		curnum = (leftnum < FEA_MAX_STAT_NUM) ? leftnum : FEA_MAX_STAT_NUM;
        leftnum -= curnum;
        for(i = 0;i < curnum;i++, p++)
        {
            // max_q(d1) = FEA_Q_STAT + Q(FEA_MAX_STAT_DIFF_Q) = 18
            // q(d4) = 4*q(d1)
            // q(curnum) + q(d4) <= 63
            j = p->s * channel;
            v = image[j];
            v = v << FEA_Q_STAT;
            d = v - mean;
            if (d < 0)  d = -d;
			d1 = (FEA_MAX_STAT_DIFF_Q < d) ? FEA_MAX_STAT_DIFF_Q : d;
            d2 = d1*d1;
            d3 = d1*d2;
            d4 = d2*d2;
            var += d2;
            skew += d3;
            kurt += d4;
        }
        skewf += (double)skew;
        kurtf += (double)kurt;
    }

    varf = (double)var;
    n = pointnum-1;
    if (n == 0)     varf = 0;
    else            varf /= n;
    svar = sqrt(varf);
    stat->var = (int)svar;

    if (svar < FLOAT_EPS)
    {
        stat->skew = 0;
        stat->kurt = 0;
    }
    else
    {
        stat->skew = (int)((FEA_S_VEC * skewf) / (pointnum * varf * svar));
        stat->kurt = (int)((FEA_S_VEC * kurtf) / (pointnum * varf * varf));
    }

    return;
}

void IrfStat2VectorInt(IN IrfStatInfoInt *stat, 
                       IN int regionnum,
                       IN int *feature,
                       IN int step)
{
    int i;
    int *f;
    IrfStatInfoInt *s;

    s = stat;
    f = feature;
    for (i = 0;i < regionnum;i++, f += step, s++)
    {
        f[0] = s->mean;
        f[1] = s->var;
        f[2] = s->skew;
        f[3] = s->kurt;
    }
}

void IrfGeo2VectorInt(IN IrfGeoInfo *geo, 
                      IN int regionnum,
                      IN int *feature,
                      IN int step)
{
    int i;
    int *f;
    IrfGeoInfo *g;

    g = geo;
    f = feature;
    for (i = 0;i < regionnum;i++, f += step, g++)
    {
        f[0] = (int)(g->area*FEA_S_VEC);
        f[1] = (int)(g->perimeter*FEA_S_VEC);
        f[2] = (int)(g->circularity1*FEA_S_VEC);
        f[3] = (int)(g->circularity2*FEA_S_VEC);

        f[4] = (int)(g->m11*FEA_S_VEC);
        f[5] = (int)(g->m02*FEA_S_VEC);
        f[6] = (int)(g->m20*FEA_S_VEC);

        f[7] = (int)(g->longaxis*FEA_S_VEC);
        f[8] = (int)(g->shortaxis*FEA_S_VEC);
        f[9] = (int)(g->eccentricity*FEA_S_VEC);
        f[10] = (int)(g->angle*FEA_S_VEC);
    }
}

void IrfLoc2VectorInt(IN IrfLocationInfo *loc, 
                      IN int regionnum,
                      IN int *feature,
                      IN int step)
{
    int i;
    int *f;
    IrfLocationInfo *l;

    l = loc;
    f = feature;
    for (i = 0;i < regionnum;i++, f += step, l++)
    {
        f[0] = (int)(l->x*FEA_S_VEC);
        f[1] = (int)(l->y*FEA_S_VEC);
        f[2] = (int)(l->d*FEA_S_VEC);
        f[3] = (int)(l->a*FEA_S_VEC);
    }
}



#ifdef __cplusplus
}
#endif /* end of __cplusplus */
