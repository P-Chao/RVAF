/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchPostprocess.c 			                             */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: 立体匹配后处理                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "ImageMatchPostprocess.h"
#include "math.h"
#include "ctmf.h"
#include "ctmfopt.h"
//#include "ImageMedian3x3.h"
#include "DataTypeConvert.h"

void ImppOptimizeWtaPixel(IN int *dsi, 
                          IN int dlength, 
                          IN int num, 
                          OUT PIXEL *label)
{
    int *pdsi = dsi;
    int i, j, disp;
    int val, minv;

    for (i = 0;i < num;i++, pdsi += dlength)
    {
        disp = 0;
        minv = pdsi[0];
        for (j = 1;j < dlength;j++)
        {
            val = pdsi[j];
            if (val < minv)
            {
                minv = val;
                disp = j;
            }
        }
        label[i] = (PIXEL)disp;
    }

    return;
}

void ImppOptimizeWta(IN int *dsi, 
                     IN int dlength, 
                     IN int num, 
                     OUT int *label)
{
    int *pdsi = dsi;
    int i, disp;
    int val;

    for (i = 0;i < num;i++, pdsi += dlength)
    {
        StatMinIndexInt(pdsi, dlength, 1, &val, &disp);
        label[i] = disp;
    }

    return;
}

void ImppOptimizeWta16Pixel(IN ushort *dsi, 
                            IN int dlength, 
                            IN int num, 
                            OUT PIXEL *label)
{
    ushort *pdsi = dsi;
    int i, j, disp;
    ushort val, minv;

    for (i = 0;i < num;i++, pdsi += dlength)
    {
        disp = 0;
        minv = pdsi[0];
        for (j = 1;j < dlength;j++)
        {
            val = pdsi[j];
            if (val < minv)
            {
                minv = val;
                disp = j;
            }
        }
        label[i] = (PIXEL)disp;
    }

    return;
}

void ImppOptimizeWta16(IN ushort *dsi, 
                       IN int dlength, 
                       IN int num, 
                       OUT int *label)
{
    ushort *pdsi = dsi;
    int i, j, disp;
    ushort val, minv;

    for (i = 0;i < num;i++, pdsi += dlength)
    {
        disp = 0;
        minv = pdsi[0];
        for (j = 1;j < dlength;j++)
        {
            val = pdsi[j];
            if (val < minv)
            {
                minv = val;
                disp = j;
            }
        }
        label[i] = disp;
    }

    return;
}

void ImppDisp2ImagePixel(OUT int *dispimg, 
                         IN PIXEL *label, 
                         IN int area,
                         IN float factor)
{
    int i;
    float val;
    for (i = 0; i < area; i++)
    {
        val = (float)label[i];
        if (val < 0)    dispimg[i] = 0xFFFF;
        else            dispimg[i] = (int)(val*factor+0.5);
        // val = max(0.0f, val);
        // dispimg[i] = (int)(val*factor+0.5);
    }
}

void ImppDisp2Image(OUT int *dispimg, 
                    IN int *label, 
                    IN int area,
                    IN float factor)
{
    int i;
    float val;
    for (i = 0; i < area; i++)
    {
        val = (float)label[i];
        if (val < 0)    dispimg[i] = 0xFFFF;
        else            dispimg[i] = (int)(val*factor+0.5);
        // val = max(0.0f, val);
        // dispimg[i] = (int)(val*factor+0.5);
    }
}

void ImppDisp2Image8(OUT PIXEL *dispimg, 
                     IN PIXEL *label, 
                     IN int area,
                     IN int factor)
{
    int i;
    for (i = 0; i < area; i++)
    {
        dispimg[i] = (PIXEL)(label[i]*factor);
    }
}

void ImppMedianFilterPixel(IN PIXEL *label, 
                           OUT PIXEL *outlabel, 
                           IN int height, 
                           IN int width, 
                           IN int radius)
{
    if (radius == 1)
    {
        IMAGE_S src;
        IMAGE_S filter;

        src.height = height;
        src.width = width;
        src.data = label;
        filter.data = outlabel;
        ImageGrayMedian3x3(&src, &filter);
    }
    else if (radius > 0)
    {
        ctmfopt(label, outlabel, height, width, 1, radius);
    }
}

void ImppMedianFilter(INOUT int *label, 
                      IN int height, 
                      IN int width, 
                      IN int radius)
{
    if (radius > 0)
    {
        PIXEL *pixlabel;
        PIXEL *pixlabel2;
        int area = height*width;

        pixlabel = MallocType(PIXEL, area);
        pixlabel2 = MallocType(PIXEL, area);
        DataInt2Pixel(label, pixlabel, area);
        ctmfopt(pixlabel, pixlabel2, height, width, 1, radius);
        DataPixel2Int(pixlabel2, label, area);
        FreeType(pixlabel, PIXEL, area);
        FreeType(pixlabel2, PIXEL, area);
    }
}

void ImppLrCheckPixel(IN PIXEL *labelL, 
                      IN PIXEL *labelR, 
                      IN int height, 
                      IN int width, 
                      IN int T,
                      OUT PIXEL *label)
{
    int xL, xR, y;
    int dL, dR, d;

    memcpy(label, labelL, height*width*sizeof(PIXEL));

    for (y = 0;y < height;y++, labelL += width, labelR += width, label += width)
    {
        for (xL = 0; xL < width; xL++)
        {
            dL = labelL[xL];
            xR = xL-dL;
            if (xR < 0)
            {
                // border
                label[xL] = 0xFF;
                continue;
            }

            dR = labelR[xR];
            d = dR-dL;
            if (d < 0)  d = -d;
            if (d > T)
            {
                // unstable
                label[xL] = 0xFF;
            }
        }
    }

    return;
}

void ImppLrCheckInt(IN int *labelL, 
                    IN int *labelR, 
                    IN int height, 
                    IN int width, 
                    IN int T,
                    OUT int *label)
{
    int xL, xR, y;
    int dL, dR, d;

    memcpy(label, labelL, height*width*sizeof(int));

    for (y = 0;y < height;y++, labelL += width, labelR += width, label += width)
    {
        for (xL = 0; xL < width; xL++)
        {
            dL = labelL[xL];
            if (dL < 0)
            {
                label[xL] = IMPP_CHECK_ERR_INVALID;
                continue;
            }

            xR = xL-dL;
            if (xR < 0)
            {
                // border
                label[xL] = IMPP_CHECK_ERR_BORDER;
                continue;
            }

            dR = labelR[xR];
            d = dR-dL;
            if (d < 0)  d = -d;
            if (d > T)
            {
                // unstable
                label[xL] = IMPP_CHECK_ERR_LR;
            }
        }
    }

    return;
}

void ImppLrCheckFloat(IN float *labelL, 
                      IN float *labelR, 
                      IN int height, 
                      IN int width, 
                      IN float T,
                      OUT float *label)
{
    int xL, xR, y;
    float dL, dR, d;

    memcpy(label, labelL, height*width*sizeof(float));

    for (y = 0;y < height;y++, labelL += width, labelR += width, label += width)
    {
        for (xL = 0; xL < width; xL++)
        {
            dL = labelL[xL];
            if (dL < 0)
            {
                label[xL] = IMPP_CHECK_ERR_INVALID;
                continue;
            }

            xR = xL-round(dL);
            if (xR < 0)
            {
                // border
                label[xL] = IMPP_CHECK_ERR_BORDER;
                continue;
            }

            dR = labelR[xR];
            d = dR-dL;
            if (d < 0)  d = -d;
            if (d > T)
            {
                // unstable
                label[xL] = IMPP_CHECK_ERR_LR;
            }
        }
    }

    return;
}

void ImppRangeCheckFloat(INOUT float *label, IN int size, IN float maxd)
{
    int i;

    for (i = 0;i < size;i++)
    {
        if (label[i] > maxd)
        {
            label[i] = IMPP_CHECK_ERR_INVALID;
        }
    }

    return;
}

void ImppInvalidInit(OUT ImppInvalidInfo *invalid, IN int maxnum)
{
    invalid->row = MallocType(int, maxnum);
    invalid->start = MallocType(int, maxnum);
    invalid->end = MallocType(int, maxnum);
}

void ImppInvalidDestroy(OUT ImppInvalidInfo *invalid, IN int maxnum)
{
    FreeType(invalid->row, int, maxnum);
    FreeType(invalid->start, int, maxnum);
    FreeType(invalid->end, int, maxnum);
}

void ImppBackgroundFillPixel(IN PIXEL *check, 
                             IN int height,
                             IN int width,
                             OUT PIXEL *label,
                             OUT ImppInvalidInfo *invalid)
{
    int *row = invalid->row;
    int *start = invalid->start;
    int *end = invalid->end;

    int x, y, i;
    int s, e, line;
    int right;
    int l;
    PIXEL v, v1, v2, inf;
    PIXEL *disp;

    if (label != check)
    {
        memcpy(label, check, height*width*sizeof(PIXEL));
    }

    line = 0;
    right = width-1;
    inf = 0xFF;
	for(y = 0, disp = label;y < height;y++, disp += width)
    {
        l = 0;
        for(x = 0;x < width;)
        {
            // 查找未标记直线
            while (x < width && l == 0)
            {
                l = (disp[x] == 0xFF);
                x++;
            }
            if (x == width && l == 0)       break;
            s = x-1;

            while (x < width && l != 0)
            {
                l = (disp[x] == 0xFF);
                x++;
            }
            if (x == width) e = x-1;
            else            e = x-2;

            // 记录直线
            row[line] = y;
            start[line] = s;
            end[line] = e;
            line++;

            // fill两侧的最小值
            if (s > 0)          v1 = disp[s-1];
            else                v1 = inf;
            if (e < right)      v2 = disp[e+1];
            else                v2 = inf;
            v = (v1 < v2) ? v1 : v2;
            for(i = s;i <= e;i++)
            {
                disp[i] = v;
            }
        }
    }

    invalid->linenum = line;

    return;
}

void ImppBackgroundFillInt(IN int *check, 
                           IN int height,
                           IN int width,
                           OUT int *label,
                           OUT ImppInvalidInfo *invalid)
{
    int *row = invalid->row;
    int *start = invalid->start;
    int *end = invalid->end;

    int x, y, i;
    int s, e, line;
    int right, inf;
    int l;
    int v, v1, v2;
    int *disp;

    if (label != check)
    {
        memcpy(label, check, height*width*sizeof(int));
    }

    line = 0;
    right = width-1;
    inf = height*width;
	for(y = 0, disp = label;y < height;y++, disp += width)
    {
        l = 0;
        for(x = 0;x < width;)
        {
            // 查找未标记直线
            while (x < width && l >= 0)
            {
                l = disp[x];
                x++;
            }
            if (x == width && l >= 0)       break;
            s = x-1;

            while (x < width && l < 0)
            {
                l = disp[x];
                x++;
            }
            if (x == width) e = x-1;
            else            e = x-2;

            // 记录直线
            row[line] = y;
            start[line] = s;
            end[line] = e;
            line++;

            // fill两侧的最小值
            if (s > 0)          v1 = disp[s-1];
            else                v1 = inf;
            if (e < right)      v2 = disp[e+1];
            else                v2 = inf;
            //v = min(v1, v2);
			v = v1 < v2 ? v1 : v2;
            for(i = s;i <= e;i++)
            {
                disp[i] = v;
            }
        }
    }

    invalid->linenum = line;

    return;
}

// Fast Cost-Volume Filtering for Visual Correspondence and Beyond
void ImppEafTablePara0(INOUT ImppEafPara *eaf)
{
    eaf->xr = 19;
    eaf->yr = 19;
    eaf->dg = 2;
    eaf->sg = 9;
    eaf->dc = 2;
    eaf->sc = 0.1*255;
}

// Patchmatch stereo-stereo matching with slanted support windows
void ImppEafTablePara1(INOUT ImppEafPara *eaf)
{
    eaf->xr = 17;
    eaf->yr = 17;
    eaf->dg = 1;
    eaf->sg = -1;
    eaf->dc = 1;
    eaf->sc = 3*10;
}

void ImppWinWmfInt(IN int *label,
                   IN IMAGE_S *guild,
                   IN ImppInvalidInfo *invalid,
                   IN ImppEafPara *eaf,
                   IN int dlength,
                   OUT int *filt)
{
    int height = guild->height;
    int width = guild->width;

    int *row = invalid->row;
    int *start = invalid->start;
    int *end = invalid->end;
    int linenum = invalid->linenum;

    int i;
    int x, y;
    int s, e;
    int median;
    int *dst;
    float *hist;
    float *acchist;

    hist = MallocType(float, dlength);
    acchist = MallocType(float, dlength+1);

    // wmf
    memcpy(filt, label, height*width*sizeof(int));
    for(i = 0;i < linenum;i++)
    {
        y = row[i];
        s = start[i];
        e = end[i];
        dst = filt + y*width;
        for(x = s;x <= e;x++)
        {
            median = ImppWinWmfOneInt(label, guild, eaf, x, y, hist, acchist, dlength);
            dst[x] = median;
        }
    }

    FreeType(hist, float, dlength);
    FreeType(acchist, float, dlength+1);

    return;
}

int ImppWinWmfOneInt(IN int *label,
                     IN IMAGE_S *guild,
                     IN ImppEafPara *eaf,
                     IN int cx,
                     IN int cy,
                     IN float *hist,
                     IN float *acchist,
                     IN int dlength)
{
    int rx = eaf->xr;
    int ry = eaf->yr;
    int dg = eaf->dg;
    int dc = eaf->dc;
    int trunc = eaf->trunc;
    float *tc = eaf->tablec;
    float *ts = eaf->tables;

    int height = guild->height;
    int width = guild->width;
    int channel = guild->channel;
    PIXEL *srchead = guild->data;
    int widthc = width * channel;
    int right = width-1;
    int bottom = height-1;

    int l, m;
    int x, y;
    int y1, y2;
    int x1, x2;
    int dx, dy;
    int dcolor, dxy;
    int disttype;
    float w;
    int *lbl;
    PIXEL *p, *q;
    PIXEL *src;

    // 确定范围
    y1 = cy-ry;
    //y1 = max(y1, 0);
	y1 = y1 > 0 ? y1 : 0;
    y2 = cy+ry;
    //y2 = min(y2, bottom);
	y2 = y2 < bottom ? y2 : bottom;
    x1 = cx-rx;
    //x1 = max(x1, 0);
	x1 = x1 > 0 ? x1 : 0;
    x2 = cx+rx;
    //x2 = min(x2, right);
	x2 = x2 < right ? x2 : right;

    if (dc == 1)        disttype = VECTOR_DIST_MANHATTAN;
    else if (dc == 2)   disttype = VECTOR_DIST_EUCLIDEAN;
    else                disttype = VECTOR_DIST_INF;

    // bilateral filter weight
    p = srchead + cy*widthc + cx*channel;
    src = srchead + y1*widthc;
    lbl = label + y1*width;
    memset(hist, 0, dlength*sizeof(float));
    for(y = y1;y <= y2;y++, src += widthc, lbl += width)
    {
        q = src + x1*channel;
        for(x = x1;x <= x2;x++, q += channel)
        {
            // 颜色权值
            dcolor = VectorDistPixelInt(p, q, channel, disttype);
			dcolor = dcolor < trunc ? dcolor : trunc;
            //dcolor = min(dcolor, trunc);
            w = tc[dcolor];

            // 几何权值
            if (ts != NULL)
            {
                dx = abs(x-cx);
                dy = abs(y-cy);
                if (dg == 1)        dxy = dx + dy;
                else if (dg == 2)   dxy = dx*dx + dy*dy;
				else                dxy = dx > dy ? dx : dy;//dxy = max(dx, dy);
                w *= ts[dxy];
            }

            l = lbl[x];
            hist[l] += w;
        }
    }

    // 找到acchist中刚刚过半对应的bin
    m = StatHistMedianFloat(hist, acchist, NULL, dlength);

    return m;
}

void ImppWinWmfFloat(IN float *label,
                     IN IMAGE_S *guild,
                     IN ImppInvalidInfo *invalid,
                     IN ImppEafPara *eaf,
                     IN int dlength,
                     OUT float *filt)
{
    int height = guild->height;
    int width = guild->width;

    int *row = invalid->row;
    int *start = invalid->start;
    int *end = invalid->end;
    int linenum = invalid->linenum;

    int i;
    int x, y;
    int s, e;
    float median;
    float *dst;
    float *hist;
    float *acchist;

    hist = MallocType(float, dlength+1);
    acchist = MallocType(float, dlength+1);

    // wmf
    memcpy(filt, label, height*width*sizeof(float));
    for(i = 0;i < linenum;i++)
    {
        y = row[i];
        s = start[i];
        e = end[i];
        dst = filt + y*width;
        for(x = s;x <= e;x++)
        {
            median = ImppWinWmfOneFloat(label, guild, eaf, x, y, hist, acchist, dlength, 1);
            dst[x] = median;
        }
    }

    FreeType(hist, float, dlength);
    FreeType(acchist, float, dlength+1);

    return;
}

float ImppWinWmfOneFloat(IN float *label,
                         IN IMAGE_S *guild,
                         IN ImppEafPara *eaf,
                         IN int cx,
                         IN int cy,
                         IN float *hist,
                         IN float *acchist,
                         IN int dlength,
                         IN int subpixel)
{
    int rx = eaf->xr;
    int ry = eaf->yr;
    int dg = eaf->dg;
    int dc = eaf->dc;
    int trunc = eaf->trunc;
    float *tc = eaf->tablec;
    float *ts = eaf->tables;

    int height = guild->height;
    int width = guild->width;
    int channel = guild->channel;
    PIXEL *srchead = guild->data;
    int widthc = width * channel;
    int right = width-1;
    int bottom = height-1;

    int li, m;
    int x, y;
    int y1, y2;
    int x1, x2;
    int dx, dy;
    int dcolor, dxy;
    int disttype;
    float w;
    float u, v, l;
    float *lbl;
    PIXEL *p, *q;
    PIXEL *src;
    float median;
    float disp;

    // 确定范围
    y1 = cy-ry;
    //y1 = max(y1, 0);
	y1 = y1 > 0 ? y1 : 0;
    y2 = cy+ry;
    //y2 = min(y2, bottom);
	y2 = y2 < bottom ? y2 : bottom;
    x1 = cx-rx;
    //x1 = max(x1, 0);
	x1 = x1 > 0 ? x1 : 0;
    x2 = cx+rx;
    //x2 = min(x2, right);
	x2 = x2 < right ? x2 : right;

    if (dc == 1)        disttype = VECTOR_DIST_MANHATTAN;
    else if (dc == 2)   disttype = VECTOR_DIST_EUCLIDEAN;
    else                disttype = VECTOR_DIST_INF;

    // bilateral filter weight
    p = srchead + cy*widthc + cx*channel;
    src = srchead + y1*widthc;
    lbl = label + y1*width;
    memset(hist, 0, dlength*sizeof(float));
    for(y = y1;y <= y2;y++, src += widthc, lbl += width)
    {
        q = src + x1*channel;
        for(x = x1;x <= x2;x++, q += channel)
        {
            // 颜色权值
            dcolor = VectorDistPixelInt(p, q, channel, disttype);
            dcolor = (dcolor < trunc) ? dcolor : trunc;
            w = tc[dcolor];

            // 几何权值
            if (ts != NULL)
            {
                dx = abs(x-cx);
                dy = abs(y-cy);
                if (dg == 1)        dxy = dx + dy;
                else if (dg == 2)   dxy = dx*dx + dy*dy;
                else                dxy = (dx > dy) ? dx : dy;
                w *= ts[dxy];
            }

            // 线性插值
            l = lbl[x];
            li = (int)l;
            u = l - li;
            v = 1 - u;
            hist[li] += v*w;
            hist[li+1] += u*w;
        }
    }

    // 找到acchist中刚刚过半对应的bin
    m = StatHistMedianFloat(hist, acchist, &median, dlength);
    disp = (float)m;

    // 亚精度插值
    if (subpixel)
    {
        float acc, past, all;
        float sub;

        // 超过中值的部分
        acc = acchist[m];
        past = acc - median;
        all = hist[m];
        sub = (float)(past/all-0.5f);
        disp -= sub;
    }

    return disp;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
