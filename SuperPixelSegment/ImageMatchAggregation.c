/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchAggregation.c 	                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/06/16                                                          */
/*                                                                           */
/* Description: 图像匹配值累积                                               */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "math.h"
#include "ImageMatchAggregation.h"

void ImaScanTreeIntAggrPara(ImaScanTreeIntInfo *aggr)
{
    aggr->mode = 0;
    aggr->sg = 25.0f;
    aggr->sc = 25.0f;
    aggr->P1 = 129;
    aggr->P2 = 2156;
    aggr->wtablelen = 3*IMAGE_WHITE;
}

void ImaScanTreeIntAggrInit(ImaScanTreeIntInfo *aggr)
{
    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;
    int Lplen = (dlength + IMA_STI_BUFEX) * 2;
    int wtablelen = aggr->wtablelen;
    floatcost sg = (floatcost)aggr->sg;

    int i;
    floatcost fg;

    for (i = 0;i < 4;i++)
    {
        aggr->wgt[i] = MallocType(int, height*width);
    }
    aggr->smoothcost = mm_MallocType(int, height*width*dlength);
    aggr->SoBuf = mm_MallocType(int, height*width*dlength);
    aggr->LpBuf = ram_MallocType(int, Lplen);
    aggr->wtable = MallocType(int, wtablelen);

    if (sg <= 0)    fg = 1.0f;
    else            fg = (floatcost)exp(-1.0/aggr->sg);
    StaWeightTabelInt(fg, aggr->sc, aggr->wtable, wtablelen);

    return;
}
void ImaScanTreeIntAggrDestroy(ImaScanTreeIntInfo *aggr)
{
    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;
    int Lplen = (dlength + IMA_STI_BUFEX) * 2;
    int wtablelen = aggr->wtablelen;

    int i;

    for (i = 0;i < 4;i++)
    {
        FreeType(aggr->wgt[i], int, height*width);
    }
    mm_FreeType(aggr->smoothcost, int, height*width*dlength);
    mm_FreeType(aggr->SoBuf, int, height*width*dlength);
    ram_FreeType(aggr->LpBuf, int, Lplen);
    FreeType(aggr->wtable, int, wtablelen);
}

void ImaScanTreeIntAggrProc(INOUT ImaScanTreeIntInfo *aggr)
{
    // int height = aggr->height;
    // int width = aggr->width;
    int dlength = aggr->dlength;
    int *imagecost = aggr->imagecost;
    int *smoothcost = aggr->smoothcost;
    int *SoBuf = aggr->SoBuf;
    int *LpBuf = aggr->LpBuf;
    int LpBufWidth = dlength + IMA_STI_BUFEX;

    int *cost, *socost;
    int i;
    int *Lp;

    // 初始化视差边界
    Lp = LpBuf + IMA_STI_BUFOFF;
    for (i = 0; i < 2; i++)
    {
        Lp[-1] = Lp[dlength] = INT_INF;
        Lp += LpBufWidth;
    }

    // 行方向
    cost = imagecost;
    socost = SoBuf;
    // memset(socost, 0, height*width*dlength*sizeof(int));
    ImaScanTreeIntAggrHor(aggr, cost, socost);

    // 列方向
    cost = SoBuf;
    socost = smoothcost;
    // memset(socost, 0, height*width*dlength*sizeof(floatcost));
    ImaScanTreeIntAggrVer(aggr, cost, socost);

    return;
}

void ImaScanTreeIntAggrHor(IN ImaScanTreeIntInfo *aggr, 
                           IN int *imagecost,
                           OUT int *smoothcost)
{
    int P1 = aggr->P1;
    int P2 = aggr->P2;
    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;
    int *LpBuf = aggr->LpBuf;
    int *wtable = aggr->wtable;
    int **wgt = aggr->wgt;
    int costwidth = width*dlength;    

    int i;
    int *cost;
    int *socost;
    int *weight1, *weight2;

////////////////////////////////////////////////////////////////////////////////////////////

    // 计算权值
    StaWeightLookupInt(aggr->Gx, wgt[IMAGE_DIRECT_LEFT], wgt[IMAGE_DIRECT_RIGHT], wtable, TRUE);

    // 行方向进行aggr
    weight1 = wgt[IMAGE_DIRECT_LEFT];
    weight2 = wgt[IMAGE_DIRECT_RIGHT];
    cost = imagecost;
    socost = smoothcost;
    for (i = 0; i < height; i++)
    { // 每次处理一行Line
#if 0
        StaLineDpaggrInt(cost, weight1, height, width, dlength, IMAGE_DIRECT_LEFT, 
                         P1, P2, socost, LpBuf);
        StaLineDpaggrInt(cost, weight2, height, width, dlength, IMAGE_DIRECT_RIGHT, 
                         P1, P2, socost, LpBuf);
        StaMoveCenterInt(cost, height, width, dlength, TRUE, socost);
#else
        StaLineDpaggrInt1(cost, weight1, height, width, dlength, IMAGE_DIRECT_LEFT, 
                          P1, P2, socost, LpBuf);
        StaLineDpaggrInt2(cost, weight2, height, width, dlength, IMAGE_DIRECT_RIGHT, 
                          P1, P2, socost, LpBuf);
#endif

        cost += costwidth;
        socost += costwidth;
        weight1 += width;
        weight2 += width;
    }

    return;
}

void ImaScanTreeIntAggrVer(IN ImaScanTreeIntInfo *aggr, 
                           IN int *imagecost,
                           OUT int *smoothcost)
{
    int P1 = aggr->P1;
    int P2 = aggr->P2;
    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;
    int *LpBuf = aggr->LpBuf;
    int *wtable = aggr->wtable;
    int **wgt = aggr->wgt;

    int i;
    int *cost;
    int *socost;
    int *weight1, *weight2;

////////////////////////////////////////////////////////////////////////////////////////////

    // 计算权值
    StaWeightLookupInt(aggr->Gy, wgt[IMAGE_DIRECT_TOP], wgt[IMAGE_DIRECT_BOTTOM], wtable, FALSE);

    // 行方向进行aggr
    weight1 = wgt[IMAGE_DIRECT_TOP];
    weight2 = wgt[IMAGE_DIRECT_BOTTOM];
    cost = imagecost;
    socost = smoothcost;
    for (i = 0; i < width; i++)
    {
#if 0
        StaLineDpaggrInt(cost, weight1, height, width, dlength, IMAGE_DIRECT_TOP, 
                         P1, P2, socost, LpBuf);
        StaLineDpaggrInt(cost, weight2, height, width, dlength, IMAGE_DIRECT_BOTTOM, 
                         P1, P2, socost, LpBuf);
        StaMoveCenterInt(cost, height, width, dlength, FALSE, socost);
#else
        StaLineDpaggrInt1(cost, weight1, height, width, dlength, IMAGE_DIRECT_TOP, 
                         P1, P2, socost, LpBuf);
        StaLineDpaggrInt2(cost, weight2, height, width, dlength, IMAGE_DIRECT_BOTTOM, 
                          P1, P2, socost, LpBuf);
#endif
        cost += dlength;
        socost += dlength;
        weight1 += 1;
        weight2 += 1;
    }

    return;
}

void StaLineDpaggrInt(IN int *imagecost, 
                      IN int *weight, 
                      IN int height, 
                      IN int width, 
                      IN int dlength, 
                      IN int begin, 
                      IN int P1,
                      IN int P2,
                      OUT int *aggrcost, 
                      OUT int *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int LpBufWidth;
    int *Lp0, *Lp1, *Lptemp;

    int *cost;
    int *aggr;
    int *pwg;

    int Lp1cost;
    int minLp0cost,minLp1cost;

    int inc, incd, incw;
    int startx, endx, startw;
    int c0, c1, c2;
    int c3, c4, c5;
    int cpd, w;
    int mulc;

    // 调整方向
    LpBufWidth = dlength+IMA_STI_BUFEX;
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        incd = dlength;
        incw = 1;
        startx = 0;
        endx = width;
        startw = startx;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        incd = -dlength;
        incw = -1;
        startx = width-1;
        endx = -1;
        startw = startx;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        incd = width*dlength;
        incw = width;
        startx = 0;
        endx = height;
        startw = startx*width;
    }
    else
    {
        // 从下到上
        inc = -1;
        incd = -width*dlength;
        incw = -width;
        startx = height-1;
        endx = -1;
        startw = startx*width;
    }

    // 第一个点
    pwg = weight + startw;
    cost = imagecost + startx*abs(incd);
    aggr = aggrcost + startx*abs(incd);

    Lp1 = LpBuf + IMA_STI_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;

    minLp1cost = INT_INF;
    for (d = 0;d < dlength;d++)
    {
        Lp1cost = cost[d];
        Lp1[d] = Lp1cost;
        aggr[d] += Lp1cost;
        if (Lp1cost < minLp1cost)
        {
            minLp1cost = Lp1cost;
        }
    }

    // 递归计算
    for (x = startx + inc; x != endx; x += inc)
    {
        swapAB(Lp0, Lp1, Lptemp);
        minLp0cost = minLp1cost;
        minLp1cost = INT_INF;
        cost += incd;
        aggr += incd;
        pwg += incw;
        w = *pwg;

        c1 = Lp0[-1];
        c0 = Lp0[0];
        c4 = minLp0cost + P2;
        Lptemp = Lp0 + 1;
        for (d = 0;d < dlength;d++)
        {
            // Lp1cost = cpd + w*min(Lp0[d], min(Lp0[d-1] + P1, min(Lp0[d+1] + P1, minLpcost + P2)));
            c2 = Lptemp[d];
            c3 = ((c1 < c2) ? c1 : c2) + P1;
            if (c0 > c4)    c0 = c4;
            c5 = ((c3 < c0) ? c3 : c0) - minLp0cost;

            mulc = w*c5;
            mulc = mulc >> IMA_STI_Q_WEIGHT;
            cpd = cost[d];
            Lp1cost = cpd + mulc;

            Lp1[d] = Lp1cost;
            aggr[d] += Lp1cost;
            if (Lp1cost < minLp1cost)
            {
                minLp1cost = Lp1cost;
            }
            c1 = c0;
            c0 = c2;
        }
    }

    return;
}

void StaLineDpaggrInt1(IN int *imagecost, 
                       IN int *weight, 
                       IN int height, 
                       IN int width, 
                       IN int dlength, 
                       IN int begin, 
                       IN int P1,
                       IN int P2,
                       OUT int *aggrcost, 
                       OUT int *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int LpBufWidth;
    int *Lp0, *Lp1, *Lptemp;

    int *cost;
    int *aggr;
    int *pwg;

    int Lp1cost;
    int minLp0cost,minLp1cost;

    int inc, incd, incw;
    int startx, endx, startw;
    int c0, c1, c2;
    int c3, c4, c5;
    int cpd, w;
    int mulc;

    // 调整方向
    LpBufWidth = dlength+IMA_STI_BUFEX;
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        incd = dlength;
        incw = 1;
        startx = 0;
        endx = width;
        startw = startx;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        incd = -dlength;
        incw = -1;
        startx = width-1;
        endx = -1;
        startw = startx;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        incd = width*dlength;
        incw = width;
        startx = 0;
        endx = height;
        startw = startx*width;
    }
    else
    {
        // 从下到上
        inc = -1;
        incd = -width*dlength;
        incw = -width;
        startx = height-1;
        endx = -1;
        startw = startx*width;
    }

    // 第一个点
    pwg = weight + startw;
    cost = imagecost + startx*abs(incd);
    aggr = aggrcost + startx*abs(incd);

    Lp1 = LpBuf + IMA_STI_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;

    minLp1cost = INT_INF;
    for (d = 0;d < dlength;d++)
    {
        Lp1cost = cost[d];
        Lp1[d] = Lp1cost;
        aggr[d] = Lp1cost;
        if (Lp1cost < minLp1cost)
        {
            minLp1cost = Lp1cost;
        }
    }

    // 递归计算
    for (x = startx + inc; x != endx; x += inc)
    {
        swapAB(Lp0, Lp1, Lptemp);
        minLp0cost = minLp1cost;
        minLp1cost = INT_INF;
        cost += incd;
        aggr += incd;
        pwg += incw;//x++;
        w = *pwg;

        c1 = Lp0[-1];
        c0 = Lp0[0];
        c4 = minLp0cost + P2;
        Lptemp = Lp0 + 1;
        for (d = 0;d < dlength;d++)
        {
            // Lp1cost = cpd + w*min(Lp0[d], min(Lp0[d-1] + P1, min(Lp0[d+1] + P1, minLpcost + P2)));
            c2 = Lptemp[d];
            c3 = ((c1 < c2) ? c1 : c2) + P1;
            if (c0 > c4)    c0 = c4;
            c5 = ((c3 < c0) ? c3 : c0) - minLp0cost;

            mulc = w*c5;
            mulc = mulc >> IMA_STI_Q_WEIGHT;
            cpd = cost[d];
            Lp1cost = cpd + mulc;

            Lp1[d] = Lp1cost;
            aggr[d] = Lp1cost;
            if (Lp1cost < minLp1cost)
            {
                minLp1cost = Lp1cost;
            }
            c1 = c0;
            c0 = c2;
        }
    }

    return;
}

void StaLineDpaggrInt2(IN int *imagecost, 
                       IN int *weight, 
                       IN int height, 
                       IN int width, 
                       IN int dlength, 
                       IN int begin, 
                       IN int P1,
                       IN int P2,
                       OUT int *aggrcost, 
                       OUT int *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int LpBufWidth;
    int *Lp0, *Lp1, *Lptemp;

    int *cost;
    int *aggr;
    int *pwg;

    int Lp1cost;
    int minLp0cost,minLp1cost;

    int inc, incd, incw;
    int startx, endx, startw;
    int c0, c1, c2;
    int c3, c4, c5;
    int cpd, w;
    int mulc;

    // 调整方向
    LpBufWidth = dlength+IMA_STI_BUFEX;
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        incd = dlength;
        incw = 1;
        startx = 0;
        endx = width;
        startw = startx;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        incd = -dlength;
        incw = -1;
        startx = width-1;
        endx = -1;
        startw = startx;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        incd = width*dlength;
        incw = width;
        startx = 0;
        endx = height;
        startw = startx*width;
    }
    else
    {
        // 从下到上
        inc = -1;
        incd = -width*dlength;
        incw = -width;
        startx = height-1;
        endx = -1;
        startw = startx*width;
    }

    // 第一个点
    pwg = weight + startw;
    cost = imagecost + startx*abs(incd);
    aggr = aggrcost + startx*abs(incd);

    Lp1 = LpBuf + IMA_STI_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;

    minLp1cost = INT_INF;
    for (d = 0;d < dlength;d++)
    {
        Lp1cost = cost[d];
        Lp1[d] = Lp1cost;
        // aggr[d] += Lp1cost;
        if (Lp1cost < minLp1cost)
        {
            minLp1cost = Lp1cost;
        }
    }

    // 递归计算
    for (x = startx + inc; x != endx; x += inc)
    {
        swapAB(Lp0, Lp1, Lptemp);
        minLp0cost = minLp1cost;
        minLp1cost = INT_INF;
        cost += incd;
        aggr += incd;
        pwg += incw;
        w = *pwg;

        c1 = Lp0[-1];
        c0 = Lp0[0];
        c4 = minLp0cost + P2;
        Lptemp = Lp0 + 1;
        for (d = 0;d < dlength;d++)
        {
            // Lp1cost = cpd + w*min(Lp0[d], min(Lp0[d-1] + P1, min(Lp0[d+1] + P1, minLpcost + P2)));
            c2 = Lptemp[d];
            c3 = ((c1 < c2) ? c1 : c2) + P1;
            if (c0 > c4)    c0 = c4;
            c5 = ((c3 < c0) ? c3 : c0) - minLp0cost;

            mulc = w*c5;
            mulc = mulc >> IMA_STI_Q_WEIGHT;
            cpd = cost[d];
            Lp1cost = cpd + mulc;

            Lp1[d] = Lp1cost;
            aggr[d] += (Lp1cost - cpd);
            if (Lp1cost < minLp1cost)
            {
                minLp1cost = Lp1cost;
            }
            c1 = c0;
            c0 = c2;
        }
    }

    return;
}

void StaMoveCenterInt(IN int *imagecost, 
                      IN int height, 
                      IN int width, 
                      IN int dlength, 
                      IN int direct, 
                      OUT int *aggrcost)
{
    int x, d;
    int incd;
    int startx, endx;
    int cpd;
    int *cost;
    int *aggr;

    if (direct == TRUE)
    {
        // 水平方向
        incd = dlength;
        startx = 0;
        endx = width;
    }
    else
    {
        // 垂直方向
        incd = width*dlength;
        startx = 0;
        endx = height;
    }

    cost = imagecost;
    aggr = aggrcost;
    for (x = startx; x < endx; x++)
    {
        for (d = 0;d < dlength;d++)
        {
            cpd = cost[d];
            aggr[d] -= cpd;
        }
        cost += incd;
        aggr += incd;
    }
}

void StaWeightTabelInt(IN floatcost fg,
                       IN floatcost sc,
                       OUT int *table,
                       IN int length)
{
    double usc = -1.0/sc;

    int i;
    double s, w;

    // 计算exp table
    for (i = 0, s = 0;i < length;i++, s += usc)
    {
        w = fg*exp(s);
        table[i] = round(IMA_STI_S_WEIGHT*w);
    }
    
    return;
}

void StaWeightLookupInt(IN IMAGE32_S *G, 
                        OUT int *weight1, 
                        OUT int *weight2, 
                        IN int *table,
                        IN int direct)
{
    int height = G->height;
    int width = G->width;
    int *g = G->data;

    int i, x, y;
    int right, bottom;
    int *w1, *w2;
    int wgt;

    if (direct == TRUE)
    {
        w1 = weight1 + 1;           // 从左到右
        w2 = weight2;               // 从右到左
        right = width-1;
        for (y = 0;y < height;y++, w1 += width, w2 += width, g += width)
        {
            w1[-1] = -1;
            w2[right] = -1;
            for (x = 0;x < right;x++)
            {
                i = g[x];
                wgt = table[i];
                w1[x] = wgt;
                w2[x] = wgt;
            }
        }
    }
    else
    {
        bottom = height-1;          // 从上到下
        w1 = weight1 + width;       // 从下到上
        w2 = weight2;
        for (y = 0;y < bottom;y++, w1 += width, w2 += width, g += width)
        {
            for (x = 0;x < width;x++)
            {
                i = g[x];
                wgt = table[i];
                w1[x] = wgt;
                w2[x] = wgt;
            }
        }

        w1 = weight1;
        w2 = weight2 + bottom*width;
        for (x = 0;x < width;x++)
        {
            w1[x] = -1;
            w2[x] = -1;
        }
    }

    return;
}


////////////////////////////////////////////////////////////////////////////////////////////////////

void ImaScanTreeUshortAggrPara(ImaScanTreeUshortInfo *aggr)
{
    aggr->mode = 0;
    aggr->P1 = 129;
    aggr->P2 = 2156;
}
void ImaScanTreeUshortAggrInit(ImaScanTreeUshortInfo *aggr)
{
    int mode = aggr->mode;
    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;
    int Lplen;

    aggr->smoothcost = mm_MallocType(ushort, height*width*dlength);

    if (mode == 0)
    {
        Lplen = (dlength + IMA_STUS_BUFEX) * 2;
        aggr->LpBuf = ram_MallocType(ushort, Lplen);
    }
    else
    {
        int hw = (height > width) ? height : width;
        Lplen = 2 * (dlength + 2) * hw;
        aggr->LpBuf = MallocType(ushort, Lplen);
        aggr->minLp = MallocType(ushort, hw);
    }

    return;
}
void ImaScanTreeUshortAggrDestroy(ImaScanTreeUshortInfo *aggr)
{
    int mode = aggr->mode;
    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;
    int Lplen;

    mm_FreeType(aggr->smoothcost, ushort, height*width*dlength);

    if (mode == 0)
    {
        Lplen = (dlength + IMA_STUS_BUFEX) * 2;
        ram_FreeType(aggr->LpBuf, ushort, Lplen);
    }
    else
    {
        int hw = (height > width) ? height : width;
        Lplen = 2 * (dlength + 2) * hw;
        FreeType(aggr->LpBuf, ushort, Lplen);
        FreeType(aggr->minLp, ushort, hw);
    }

    return;
}

void ImaScanTreeUshortAggrProc(INOUT ImaScanTreeUshortInfo *aggr)
{
    ushort P1 = aggr->P1;
    ushort P2 = aggr->P2;

    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;
    int costwidth = width*dlength; 

    ushort *imagecost = aggr->imagecost;
    ushort *smoothcost = aggr->smoothcost;
    ushort *LpBuf = aggr->LpBuf;
    int LpBufWidth = dlength + IMA_STUS_BUFEX;

    int i;
    ushort *cost, *socost;
    ushort *Lp;

    // 初始化
    // memset(smoothcost, 0, height*width*dlength*sizeof(ushort));

    // 初始化视差边界
    Lp = LpBuf + IMA_STUS_BUFOFF;
    for (i = 0; i < 2; i++)
    {
        Lp[-1] = Lp[dlength] = INT16_INF;
        Lp += LpBufWidth;
    }

    // 行方向进行aggr
    cost = imagecost;
    socost = smoothcost;
    for (i = 0; i < height; i++)
    {
        StaLineSgmaggrUshort1(cost, height, width, dlength, IMAGE_DIRECT_LEFT, 
                              P1, P2, socost, LpBuf);
        StaLineSgmaggrUshort(cost, height, width, dlength, IMAGE_DIRECT_RIGHT, 
                             P1, P2, socost, LpBuf);
        cost += costwidth;
        socost += costwidth;
    }

    // 列方向进行aggr
    cost = imagecost;
    socost = smoothcost;
    for (i = 0; i < width; i++)
    {
        StaLineSgmaggrUshort(cost, height, width, dlength, IMAGE_DIRECT_TOP, 
                             P1, P2, socost, LpBuf);
        StaLineSgmaggrUshort(cost, height, width, dlength, IMAGE_DIRECT_BOTTOM, 
                             P1, P2, socost, LpBuf);        
        cost += dlength;
        socost += dlength;
    }

    return;
}

#ifdef PLATFORM_CCS
void StaLineSgmaggrUshort(IN ushort *imagecost, 
                          IN int height, 
                          IN int width, 
                          IN int dlength, 
                          IN int begin, 
                          IN ushort P1,
                          IN ushort P2,
                          OUT ushort *aggrcost, 
                          OUT ushort *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int dlength_half;
    int LpBufWidth;

    unit32 *cost;
    unit32 *aggr;
    unit32 *Lp132;
    ushort *Lp0, *Lp1, *Lptemp;

    int inc, incd;
    int startx, endx;
    unit32 P1P1, P2P2;
    unit32 c0, c1, c2;
    unit32 c3, c4, c5;
    unit32 c6, c7;
    unit32 cpd;
    unit32 Lp1cost;
    unit32 minLp0cost, minLp1cost;
    ushort minLp0costone;
    ushort *pminLp0cost, *pminLp1cost;
    ushort *temp;

    temp = (ushort *)(&P1P1);
    temp[0] = temp[1] = P1;
    temp = (ushort *)(&P2P2);
    temp[0] = temp[1] = P2;

    // 调整方向
    LpBufWidth = dlength+IMA_STUS_BUFEX;
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        incd = dlength;
        startx = 0;
        endx = width;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        incd = -dlength;
        startx = width-1;
        endx = -1;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        incd = width*dlength;
        startx = 0;
        endx = height;
    }
    else
    {
        // 从下到上
        inc = -1;
        incd = -width*dlength;
        startx = height-1;
        endx = -1;
    }

    // 第一个点
    dlength_half = dlength >> 1;
    incd = incd >> 1;
    cost = ((unit32 *)imagecost) + startx*abs(incd);
    aggr = ((unit32 *)aggrcost) + startx*abs(incd);

    Lp1 = LpBuf + IMA_STUS_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;
    pminLp0cost = (ushort *)(&minLp0cost);
    pminLp1cost = (ushort *)(&minLp1cost);

    minLp1cost = INT16_INF2;
    Lp132 = (unit32 *)Lp1;
    for (d = 0;d < dlength_half;d++)
    {
        Lp1cost = cost[d];
        Lp132[d] = Lp1cost;
        aggr[d] = _add2(aggr[d], Lp1cost);
        minLp1cost = _min2(minLp1cost, Lp1cost);
    }
    minLp0costone = min(pminLp1cost[0], pminLp1cost[1]);
    pminLp0cost[0] = pminLp0cost[1] = minLp0costone;

    // 递归计算
    for (x = startx + inc; x != endx; x += inc)
    {        
        swapAB(Lp0, Lp1, Lptemp);
        cost += incd;
        aggr += incd;
        minLp1cost = INT16_INF2;
        Lp132 = (unit32 *)Lp1;
        Lptemp = Lp0 + 1;
        c1 = _mem4(Lp0 - 1);
        c4 = _add2(minLp0cost, P2P2);
        for (d = 0;d < dlength_half;d++, Lptemp += 2)
        {            
            c2 = _mem4(Lptemp);
            c0 = _packlh2(c2, c1);

            c3 = _add2(_min2(c1, c2), P1P1);
            c5 = _min2(c0, c4);
            c6 = _min2(c5, c3);
#if 0
            // (c6 - minLp0cost) + cpd
            c7 = _sub2(c6, minLp0cost);
            cpd = cost[d2];
            Lp1cost = _add2(c7, cpd);
#else
            // (cpd - minLp0cost) + c6: 并行更好，但c7会溢出为负数，最终的测试结果发现结果正确!
            cpd = cost[d];
            c7 = _sub2(cpd, minLp0cost);
            Lp1cost = _add2(c7, c6);
#endif
            Lp132[d] = Lp1cost;
            aggr[d] = _add2(aggr[d], Lp1cost);
            minLp1cost = _min2(minLp1cost, Lp1cost);
            c1 = c2;
        }
        minLp0costone = min(pminLp1cost[0], pminLp1cost[1]);
        pminLp0cost[0] = pminLp0cost[1] = minLp0costone;
    }

    return;
}
void StaLineSgmaggrUshort1(IN ushort *imagecost, 
                           IN int height, 
                           IN int width, 
                           IN int dlength, 
                           IN int begin, 
                           IN ushort P1,
                           IN ushort P2,
                           OUT ushort *aggrcost, 
                           OUT ushort *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int dlength_half;
    int LpBufWidth;

    unit32 *cost;
    unit32 *aggr;
    unit32 *Lp132;
    ushort *Lp0, *Lp1, *Lptemp;

    int inc, incd;
    int startx, endx;
    unit32 P1P1, P2P2;
    unit32 c0, c1, c2;
    unit32 c3, c4, c5;
    unit32 c6, c7;
    unit32 cpd;
    unit32 Lp1cost;
    unit32 minLp0cost, minLp1cost;
    ushort minLp0costone;
    ushort *pminLp0cost, *pminLp1cost;
    ushort *temp;

    temp = (ushort *)(&P1P1);
    temp[0] = temp[1] = P1;
    temp = (ushort *)(&P2P2);
    temp[0] = temp[1] = P2;

    // 调整方向
    LpBufWidth = dlength+IMA_STUS_BUFEX;
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        incd = dlength;
        startx = 0;
        endx = width;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        incd = -dlength;
        startx = width-1;
        endx = -1;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        incd = width*dlength;
        startx = 0;
        endx = height;
    }
    else
    {
        // 从下到上
        inc = -1;
        incd = -width*dlength;
        startx = height-1;
        endx = -1;
    }

    // 第一个点
    dlength_half = dlength >> 1;
    incd = incd >> 1;
    cost = ((unit32 *)imagecost) + startx*abs(incd);
    aggr = ((unit32 *)aggrcost) + startx*abs(incd);

    Lp1 = LpBuf + IMA_STUS_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;
    pminLp0cost = (ushort *)(&minLp0cost);
    pminLp1cost = (ushort *)(&minLp1cost);

    minLp1cost = INT16_INF2;
    Lp132 = (unit32 *)Lp1;
    for (d = 0;d < dlength_half;d++)
    {
        Lp1cost = cost[d];
        Lp132[d] = Lp1cost;
        aggr[d] = Lp1cost;
        minLp1cost = _min2(minLp1cost, Lp1cost);
    }
    minLp0costone = min(pminLp1cost[0], pminLp1cost[1]);
    pminLp0cost[0] = pminLp0cost[1] = minLp0costone;

    // 递归计算
    for (x = startx + inc; x != endx; x += inc)
    {        
        swapAB(Lp0, Lp1, Lptemp);
        cost += incd;
        aggr += incd;
        minLp1cost = INT16_INF2;
        Lp132 = (unit32 *)Lp1;
        Lptemp = Lp0 + 1;
        c1 = _mem4(Lp0 - 1);
        c4 = _add2(minLp0cost, P2P2);
        for (d = 0;d < dlength_half;d++, Lptemp += 2)
        {            
            c2 = _mem4(Lptemp);
            c0 = _packlh2(c2, c1);

            c3 = _add2(_min2(c1, c2), P1P1);
            c5 = _min2(c0, c4);
            c6 = _min2(c5, c3);
#if 0
            // (c6 - minLp0cost) + cpd
            c7 = _sub2(c6, minLp0cost);
            cpd = cost[d2];
            Lp1cost = _add2(c7, cpd);
#else
            // (cpd - minLp0cost) + c6: 并行更好，但c7会溢出为负数，最终的测试结果发现结果正确!
            cpd = cost[d];
            c7 = _sub2(cpd, minLp0cost);
            Lp1cost = _add2(c7, c6);
#endif
            Lp132[d] = Lp1cost;
            aggr[d] = Lp1cost;
            minLp1cost = _min2(minLp1cost, Lp1cost);
            c1 = c2;
        }
        minLp0costone = min(pminLp1cost[0], pminLp1cost[1]);
        pminLp0cost[0] = pminLp0cost[1] = minLp0costone;
    }

    return;
}
#elif defined PLATFORM_SSE

#define _mm_min_epi16_all(arr, min16)       \
{                                           \
    min16 = arr[0];                         \
    for (d = 1;d < 8;d++)                   \
    {                                       \
        ushort one = arr[d];                \
        if (min16 > one) min16 = one;       \
    }                                       \
}

void StaLineSgmaggrUshort(IN ushort *imagecost, 
                          IN int height, 
                          IN int width, 
                          IN int dlength, 
                          IN int begin, 
                          IN ushort P1,
                          IN ushort P2,
                          OUT ushort *aggrcost, 
                          OUT ushort *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int inc, startx, endx;
    int LpBufWidth;
    ushort *Lp0, *Lp1, *Lptemp;
    ushort minLpcost;

    __m128i *mm_imagecost, *mm_aggrcost, *mm_LpBuf;
    __m128i *mm_cost, *mm_aggr;
    __m128i *mm_Lp0, *mm_Lp1;
    __m128i mm_P1, mm_Lpcost;
    __m128i mm_minLp0cost, mm_minLp1cost;
    __m128i mm_c0, mm_c1, mm_c2;
    __m128i mm_c3, mm_c4, mm_c5;
    __m128i mm_c6, mm_c7;
    __m128i *mm_Lptemp;
    ushort *pminLp1cost;
    int mm_d, mm_incd;
    int mm_dlength, mm_LpBufWidth;

    mm_dlength = dlength >> 3;
    LpBufWidth = dlength + IMA_STUS_BUFEX;
    mm_LpBufWidth = mm_dlength + 2;

    // 调整方向   
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        mm_incd = mm_dlength;
        startx = 0;
        endx = width;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        mm_incd = -mm_dlength;
        startx = width-1;
        endx = -1;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        mm_incd = width*mm_dlength;
        startx = 0;
        endx = height;
    }
    else
    {
        // 从下到上
        inc = -1;
        mm_incd = -width*mm_dlength;
        startx = height-1;
        endx = -1;
    }

    // 第一个点
    mm_imagecost = (__m128i *)imagecost;
    mm_aggrcost = (__m128i *)aggrcost;
    mm_cost = mm_imagecost + startx*abs(mm_incd);
    mm_aggr = mm_aggrcost + startx*abs(mm_incd);

    mm_LpBuf = (__m128i *)LpBuf;
    Lp1 = LpBuf + IMA_STUS_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;
    mm_Lp1 = mm_LpBuf + 1;
    mm_Lp0 = mm_Lp1 + mm_LpBufWidth;
    pminLp1cost = (ushort *)(&mm_minLp1cost);

    mm_minLp1cost = _mm_set1_epi16(INT16_INF);
    for (mm_d = 0;mm_d < mm_dlength;mm_d++)
    {
        mm_Lpcost = mm_cost[mm_d];
        mm_Lp1[mm_d] = mm_Lpcost;
        mm_aggr[mm_d] = _mm_adds_epi16(mm_aggr[mm_d], mm_Lpcost);
        mm_minLp1cost = _mm_min_epi16(mm_Lpcost, mm_minLp1cost);
    }
    _mm_min_epi16_all(pminLp1cost, minLpcost);

    // 递归计算
    mm_P1 = _mm_set1_epi16(P1);
    for (x = startx + inc; x != endx; x += inc)
    {
        swapAB(Lp0, Lp1, Lptemp);
        swapAB(mm_Lp0, mm_Lp1, mm_Lptemp);        
        mm_cost += mm_incd;
        mm_aggr += mm_incd;
        mm_minLp1cost = _mm_set1_epi16(INT16_INF);
        mm_minLp0cost = _mm_set1_epi16(minLpcost);
        mm_c4 = _mm_set1_epi16(minLpcost + P2);
        Lptemp = Lp0 - 1;
        for (mm_d = 0;mm_d < mm_dlength;mm_d++, Lptemp += 8)
        {
            mm_c0 = mm_Lp0[mm_d];
            mm_c1 = _mm_loadu_si128((__m128i *)Lptemp);
            mm_c2 = _mm_loadu_si128((__m128i *)(Lptemp + 2));
            mm_c3 = _mm_adds_epi16(_mm_min_epi16(mm_c1, mm_c2), mm_P1);
            mm_c5 = _mm_min_epi16(mm_c0, mm_c4);
            mm_c6 = _mm_min_epi16(mm_c5, mm_c3);
            mm_c7 = _mm_subs_epi16(mm_cost[mm_d], mm_minLp0cost);
            mm_Lpcost = _mm_adds_epi16(mm_c7, mm_c6);

            mm_Lp1[mm_d] = mm_Lpcost;
            mm_aggr[mm_d] = _mm_adds_epi16(mm_aggr[mm_d], mm_Lpcost);
            mm_minLp1cost = _mm_min_epi16(mm_minLp1cost, mm_Lpcost);
        }
        _mm_min_epi16_all(pminLp1cost, minLpcost);
    }

    return;
}

void StaLineSgmaggrUshort1(IN ushort *imagecost, 
                           IN int height, 
                           IN int width, 
                           IN int dlength, 
                           IN int begin, 
                           IN ushort P1,
                           IN ushort P2,
                           OUT ushort *aggrcost, 
                           OUT ushort *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int inc, startx, endx;
    int LpBufWidth;
    ushort *Lp0, *Lp1, *Lptemp;
    ushort minLpcost;

    __m128i *mm_imagecost, *mm_aggrcost, *mm_LpBuf;
    __m128i *mm_cost, *mm_aggr;
    __m128i *mm_Lp0, *mm_Lp1;
    __m128i mm_P1, mm_Lpcost;
    __m128i mm_minLp0cost, mm_minLp1cost;
    __m128i mm_c0, mm_c1, mm_c2;
    __m128i mm_c3, mm_c4, mm_c5;
    __m128i mm_c6, mm_c7;
    __m128i *mm_Lptemp;
    ushort *pminLp1cost;
    int mm_d, mm_incd;
    int mm_dlength, mm_LpBufWidth;

    mm_dlength = dlength >> 3;
    LpBufWidth = dlength + IMA_STUS_BUFEX;
    mm_LpBufWidth = mm_dlength + 2;

    // 调整方向   
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        mm_incd = mm_dlength;
        startx = 0;
        endx = width;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        mm_incd = -mm_dlength;
        startx = width-1;
        endx = -1;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        mm_incd = width*mm_dlength;
        startx = 0;
        endx = height;
    }
    else
    {
        // 从下到上
        inc = -1;
        mm_incd = -width*mm_dlength;
        startx = height-1;
        endx = -1;
    }

    // 第一个点
    mm_imagecost = (__m128i *)imagecost;
    mm_aggrcost = (__m128i *)aggrcost;
    mm_cost = mm_imagecost + startx*abs(mm_incd);
    mm_aggr = mm_aggrcost + startx*abs(mm_incd);

    mm_LpBuf = (__m128i *)LpBuf;
    Lp1 = LpBuf + IMA_STUS_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;
    mm_Lp1 = mm_LpBuf + 1;
    mm_Lp0 = mm_Lp1 + mm_LpBufWidth;
    pminLp1cost = (ushort *)(&mm_minLp1cost);

    mm_minLp1cost = _mm_set1_epi16(INT16_INF);
    for (mm_d = 0;mm_d < mm_dlength;mm_d++)
    {
        mm_Lpcost = mm_cost[mm_d];
        mm_Lp1[mm_d] = mm_Lpcost;
        mm_aggr[mm_d] = mm_Lpcost;
        mm_minLp1cost = _mm_min_epi16(mm_Lpcost, mm_minLp1cost);
    }
    _mm_min_epi16_all(pminLp1cost, minLpcost);

    // 递归计算
    mm_P1 = _mm_set1_epi16(P1);
    for (x = startx + inc; x != endx; x += inc)
    {
        swapAB(Lp0, Lp1, Lptemp);
        swapAB(mm_Lp0, mm_Lp1, mm_Lptemp);        
        mm_cost += mm_incd;
        mm_aggr += mm_incd;
        mm_minLp1cost = _mm_set1_epi16(INT16_INF);
        mm_minLp0cost = _mm_set1_epi16(minLpcost);
        mm_c4 = _mm_set1_epi16(minLpcost + P2);
        Lptemp = Lp0 - 1;
        for (mm_d = 0;mm_d < mm_dlength;mm_d++, Lptemp += 8)
        {
            mm_c0 = mm_Lp0[mm_d];
            mm_c1 = _mm_loadu_si128((__m128i *)Lptemp);
            mm_c2 = _mm_loadu_si128((__m128i *)(Lptemp + 2));
            mm_c3 = _mm_adds_epi16(_mm_min_epi16(mm_c1, mm_c2), mm_P1);
            mm_c5 = _mm_min_epi16(mm_c0, mm_c4);
            mm_c6 = _mm_min_epi16(mm_c5, mm_c3);
            mm_c7 = _mm_subs_epi16(mm_cost[mm_d], mm_minLp0cost);
            mm_Lpcost = _mm_adds_epi16(mm_c7, mm_c6);

            mm_Lp1[mm_d] = mm_Lpcost;
            mm_aggr[mm_d] = mm_Lpcost;
            mm_minLp1cost = _mm_min_epi16(mm_minLp1cost, mm_Lpcost);
        }
        _mm_min_epi16_all(pminLp1cost, minLpcost);
    }

    return;
}
#else
void StaLineSgmaggrUshort(IN ushort *imagecost, 
                          IN int height, 
                          IN int width, 
                          IN int dlength, 
                          IN int begin, 
                          IN ushort P1,
                          IN ushort P2,
                          OUT ushort *aggrcost, 
                          OUT ushort *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int LpBufWidth;

    ushort *cost;
    ushort *aggr;
    ushort *Lp0, *Lp1, *Lptemp;

    ushort Lp1cost;
    ushort minLp0cost,minLp1cost;

    int inc, incd;
    int startx, endx;
    ushort c0, c1, c2;
    ushort c3, c4, c5;
    ushort cpd;

    // 调整方向
    LpBufWidth = dlength+IMA_STUS_BUFEX;
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        incd = dlength;
        startx = 0;
        endx = width;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        incd = -dlength;
        startx = width-1;
        endx = -1;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        incd = width*dlength;
        startx = 0;
        endx = height;
    }
    else
    {
        // 从下到上
        inc = -1;
        incd = -width*dlength;
        startx = height-1;
        endx = -1;
    }

    // 第一个点
    cost = imagecost + startx*abs(incd);
    aggr = aggrcost + startx*abs(incd);

    Lp1 = LpBuf + IMA_STUS_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;

    minLp1cost = INT16_INF;
    for (d = 0;d < dlength;d++)
    {
        Lp1cost = cost[d];
        Lp1[d] = Lp1cost;
        aggr[d] += Lp1cost;
        if (Lp1cost < minLp1cost)
        {
            minLp1cost = Lp1cost;
        }
    }

    // 递归计算
    for (x = startx + inc; x != endx; x += inc)
    {
        swapAB(Lp0, Lp1, Lptemp);
        minLp0cost = minLp1cost;
        minLp1cost = INT16_INF;
        cost += incd;
        aggr += incd;
        c1 = Lp0[-1];
        c0 = Lp0[0];
        c4 = minLp0cost + P2;
        for (d = 0;d < dlength;d++)
        {
            // Lp1cost = cpd + min(Lp0[d], min(Lp0[d-1] + P1, min(Lp0[d+1] + P1, minLpcost + P2)));
            c2 = Lp0[d+1];
            c3 = min(c1, c2) + P1;
            if (c0 > c4)    c0 = c4;
            c5 = min(c3, c0) - minLp0cost;
            cpd = cost[d];
            Lp1cost = cpd + c5;
            Lp1[d] = Lp1cost;
            aggr[d] += Lp1cost;
            if (Lp1cost < minLp1cost)
            {
                minLp1cost = Lp1cost;
            }
            c1 = c0;
            c0 = c2;
        }
    }

    return;
}

void StaLineSgmaggrUshort1(IN ushort *imagecost, 
                           IN int height, 
                           IN int width, 
                           IN int dlength, 
                           IN int begin, 
                           IN ushort P1,
                           IN ushort P2,
                           OUT ushort *aggrcost, 
                           OUT ushort *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, d;
    int LpBufWidth;

    ushort *cost;
    ushort *aggr;
    ushort *Lp0, *Lp1, *Lptemp;

    ushort Lp1cost;
    ushort minLp0cost,minLp1cost;

    int inc, incd;
    int startx, endx;
    ushort c0, c1, c2;
    ushort c3, c4, c5;
    ushort cpd;

    // 调整方向
    LpBufWidth = dlength+IMA_STUS_BUFEX;
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        inc = 1;
        incd = dlength;
        startx = 0;
        endx = width;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        inc = -1;
        incd = -dlength;
        startx = width-1;
        endx = -1;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        inc = 1;
        incd = width*dlength;
        startx = 0;
        endx = height;
    }
    else
    {
        // 从下到上
        inc = -1;
        incd = -width*dlength;
        startx = height-1;
        endx = -1;
    }

    // 第一个点
    cost = imagecost + startx*abs(incd);
    aggr = aggrcost + startx*abs(incd);

    Lp1 = LpBuf + IMA_STUS_BUFOFF;
    Lp0 = Lp1 + LpBufWidth;

    minLp1cost = INT16_INF;
    for (d = 0;d < dlength;d++)
    {
        Lp1cost = cost[d];
        Lp1[d] = Lp1cost;
        aggr[d] = Lp1cost;
        if (Lp1cost < minLp1cost)
        {
            minLp1cost = Lp1cost;
        }
    }

    // 递归计算
    for (x = startx + inc; x != endx; x += inc)
    {
        swapAB(Lp0, Lp1, Lptemp);
        minLp0cost = minLp1cost;
        minLp1cost = INT16_INF;
        cost += incd;
        aggr += incd;
        c1 = Lp0[-1];
        c0 = Lp0[0];
        c4 = minLp0cost + P2;
        for (d = 0;d < dlength;d++)
        {
            // Lp1cost = cpd + min(Lp0[d], min(Lp0[d-1] + P1, min(Lp0[d+1] + P1, minLpcost + P2)));
            c2 = Lp0[d+1];
            c3 = min(c1, c2) + P1;
            if (c0 > c4)    c0 = c4;
            c5 = min(c3, c0) - minLp0cost;
            cpd = cost[d];
            Lp1cost = cpd + c5;
            Lp1[d] = Lp1cost;
            aggr[d] = Lp1cost;
            if (Lp1cost < minLp1cost)
            {
                minLp1cost = Lp1cost;
            }
            c1 = c0;
            c0 = c2;
        }
    }

    return;
}
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////

void ImaScanTreeUshortAggrProcXdy(INOUT ImaScanTreeUshortInfo *aggr)
{
    ushort P1 = aggr->P1;
    ushort P2 = aggr->P2;

    int height = aggr->height;
    int width = aggr->width;
    int dlength = aggr->dlength;

    ushort *imagecost = aggr->imagecost;
    ushort *smoothcost = aggr->smoothcost;
    ushort *LpBuf = aggr->LpBuf;
    ushort *minLp = aggr->minLp;

    int i;

    // 初始化
    memset(smoothcost, 0, height*width*dlength*sizeof(ushort));

    // 行方向进行aggr
    for (i = IMAGE_DIRECT_LEFT; i <= IMAGE_DIRECT_BOTTOM; i++)
    {
        StaLineSgmaggrUshortXdy(imagecost, height, width, dlength, i, 
                                P1, P2, smoothcost, minLp, LpBuf);
    }

    return;
}

void StaLineSgmaggrUshortXdy(IN ushort *imagecost, 
                             IN int height, 
                             IN int width, 
                             IN int dlength, 
                             IN int begin, 
                             IN ushort P1,
                             IN ushort P2,
                             OUT ushort *aggrcost, 
                             INOUT ushort *minLp,
                             INOUT ushort *LpBuf)
{
    // 0表示前一个点，1表示当前点
    int x, y, d;

    ushort *cost, *aggr;
    ushort *himage, *haggr;
    ushort *Lp0, *Lp1, *Lp2, *Lpnew;
    ushort *Lpbufcur, *Lpbufnew, *Lptemp;

    ushort Lpcost;
    ushort minLpcost;

    int incd, incxd;
    int pos;
    int xlength, ylength;

    ushort c0, c1, c2;
    ushort cpd;

    // 调整方向
    if (begin == IMAGE_DIRECT_LEFT)
    {
        // 从左到右
        incd = dlength;
        pos = 0;
        incxd = width*dlength;
        xlength = width;
        ylength = height;
    }
    else if (begin == IMAGE_DIRECT_RIGHT)
    {
        // 从右到左
        incd = -dlength;
        pos = (width-1)*dlength;
        incxd = width*dlength;
        xlength = width;
        ylength = height;
    }
    else if (begin == IMAGE_DIRECT_TOP)
    {
        // 从上到下
        incd = width*dlength;
        pos = 0;
        incxd = dlength;
        xlength = height;
        ylength = width;
    }
    else
    {
        // 从下到上
        incd = -width*dlength;
        pos = (height-1)*width*dlength;
        incxd = dlength;
        xlength = height;
        ylength = width;
    }
    himage = imagecost + pos;
    haggr = aggrcost + pos;

    // 乒乓内存设置
    Lpbufcur = LpBuf + ylength;
    Lpbufnew = Lpbufcur + (dlength + 2) * ylength;

    // 第一个点
    cost = himage;
    aggr = haggr;
    for (y = 0; y < ylength; y++, cost += incxd, aggr += incxd)
    {
        // 设置无穷大
        Lp0 = Lpbufnew + y;
        Lp0[-ylength] = INT16_INF;
        Lp0[dlength*ylength] = INT16_INF;
        Lp1 = Lpbufcur + y;
        Lp1[-ylength] = INT16_INF;
        Lp1[dlength*ylength] = INT16_INF;

        // 求最小值
        minLpcost = INT16_INF;
        for (d = 0;d < dlength;d++, Lp0 += ylength)
        {
            Lpcost = cost[d];
            *Lp0 = Lpcost;
            aggr[d] += Lpcost;
            if (Lpcost < minLpcost)
            {
                minLpcost = Lpcost;
            }
        }
        minLp[y] = minLpcost;
    }

    // 递归计算
    for (x = 1; x < xlength; x++)
    {    
        himage += incd;
        haggr += incd;
        swapAB(Lpbufcur, Lpbufnew, Lptemp);
        for (d = 0;d < dlength;d++)
        {
            cost = himage + d;
            aggr = haggr + d;
            Lpnew = Lpbufnew + d * ylength;
            Lp0 = Lpbufcur + d * ylength;
            Lp1 = Lp0 - ylength;
            Lp2 = Lp0 + ylength;
            for (y = 0; y < ylength; y++, cost += incxd, aggr += incxd)
            {
                c0 = Lp0[y];
                c1 = Lp1[y];
                c2 = Lp2[y];
                minLpcost = minLp[y];

                c1 = ((c1 < c2) ? c1 : c2) + P1;
                c2 = minLpcost + P2;
                c1 = (c1 < c2) ? c1 : c2;
                c0 = (c1 < c0) ? c1 : c0;
                c0 -= minLpcost;

                cpd = *cost;
                Lpcost = cpd + c0;

                Lpnew[y] = Lpcost;
                *aggr += Lpcost;
            }
        }

        for (y = 0; y < ylength; y++)
        {
            Lp0 = Lpbufnew + y;
            minLpcost = *Lp0;
            for (d = 1, Lp0 += ylength;d < dlength;d++, Lp0 += ylength)
            {
                Lpcost = *Lp0;
                if (Lpcost < minLpcost)
                {
                    minLpcost = Lpcost;
                }
            }
            minLp[y] = minLpcost;
        }
    }

    return;
}


#ifdef __cplusplus
}
#endif /* end of __cplusplus */
