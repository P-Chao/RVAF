/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchEadp.c 			                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2015/05/05                                                          */
/*                                                                           */
/* Description: Eadp算法                                                     */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "ImageMatchEadp.h"
#include "math.h"

void ImageMatchEadpPara(INOUT ImEadpInfo *eadp)
{
    eadp->imagenum = 2;
    eadp->mode = 0;
    eadp->guildmr = 1;
    eadp->dispmr = 2;
    eadp->sg = -1;
    eadp->sc = 0.1f*255.0f;
    eadp->r1 = 30;
    eadp->r2 = 500;
}

void ImageMatchEadpInit(INOUT ImEadpInfo *eadp)
{
    int imagenum = eadp->imagenum;
    int dlength = eadp->dlength;
    int height = eadp->height;
    int width = eadp->width;
    int channel = eadp->channel;
    int area = height*width;

    IMAGE_S *filter = eadp->filter;
    IMAGE32_S *Gx = eadp->Gx;
    IMAGE32_S *Gy = eadp->Gy;
    ImaScanTreeIntInfo *aggr = &eadp->aggr;
    ImppInvalidInfo *invalid = &eadp->invalid;

    int i;

    eadp->imagecost = mm_MallocType(int, height*width*dlength);

    for (i = 0;i < imagenum;i++)
    {
        filter[i].data = MallocType(PIXEL, area*channel);
        filter[i].height = height;
        filter[i].width = width;
        filter[i].channel = channel;

        Gx[i].data = MallocType(int, area);
        Gx[i].height = height;
        Gx[i].width = width;
        Gx[i].channel = 1;

        Gy[i].data = MallocType(int, area);
        Gy[i].height = height;
        Gy[i].width = width;
        Gy[i].channel = 1;
    }

    ImaScanTreeIntAggrPara(aggr);
    aggr->height = height;
    aggr->width = width;
    aggr->dlength = dlength;
    aggr->mode = eadp->mode;
    aggr->sg = eadp->sg;
    aggr->sc = eadp->sc;
    ImaScanTreeIntAggrInit(aggr);

    ImppInvalidInit(invalid, area);

    eadp->left = MallocType(PIXEL, area);
    eadp->right = MallocType(PIXEL, area);
    eadp->sparse = MallocType(PIXEL, area);
    eadp->fill = MallocType(PIXEL, area);
    eadp->dense = MallocType(PIXEL, area);

    return;
}
void ImageMatchEadpDestroy(INOUT ImEadpInfo *eadp)
{
    int imagenum = eadp->imagenum;
    int dlength = eadp->dlength;
    int height = eadp->height;
    int width = eadp->width;
    int channel = eadp->channel;
    int area = height*width;

    IMAGE_S *filter = eadp->filter;
    IMAGE32_S *Gx = eadp->Gx;
    IMAGE32_S *Gy = eadp->Gy;

    int i;

    mm_FreeType(eadp->imagecost, int, height*width*dlength);

    for (i = 0;i < imagenum;i++)
    {
        FreeType(filter[i].data, PIXEL, area*channel);
        FreeType(Gx[i].data, int, area);
        FreeType(Gy[i].data, int, area);
    }

    ImaScanTreeIntAggrDestroy(&eadp->aggr);
    ImppInvalidDestroy(&eadp->invalid, area);

    FreeType(eadp->left, PIXEL, area);
    FreeType(eadp->right, PIXEL, area);
    FreeType(eadp->sparse, PIXEL, area);
    FreeType(eadp->fill, PIXEL, area);
    FreeType(eadp->dense, PIXEL, area);

    return;
}

void ImageMatchEadpProc(INOUT ImEadpInfo *eadp)
{
    ImaScanTreeIntInfo *aggr = &eadp->aggr;
    ImppInvalidInfo *invalid = &eadp->invalid;
    
    IMAGE_S **src = eadp->src;
    IMAGE32_S *Gx = eadp->Gx;
    IMAGE32_S *Gy = eadp->Gy;
    int *imagecost = eadp->imagecost;
    int dlength = eadp->dlength;

    int height = eadp->height;
    int width = eadp->width;
    int area = height*width;

    int maxcost;

    // guild image
    EadpGradient(eadp);	// 创建指导图像

    // cost
    ImcPaperCensusInt(src[0], src[1], dlength, 2, 2, &maxcost, imagecost);
    aggr->P1 = round(eadp->r1*maxcost/dlength);
    aggr->P2 = round(eadp->r2*maxcost/dlength);

    // cost aggr
    aggr->imagecost = imagecost;
    aggr->src = src[0];
    aggr->Gx = &Gx[0];
    aggr->Gy = &Gy[0];
    ImaScanTreeIntAggrProc(aggr);

    // wta
    ImppOptimizeWtaPixel(aggr->smoothcost, dlength, area, eadp->left);

    // invcost
    ImcDsiReverseInt(imagecost, aggr->SoBuf, height, width, dlength, maxcost, 2);
    eadp->imagecost = aggr->SoBuf;
    aggr->SoBuf = imagecost;
    imagecost = eadp->imagecost;

    // cost aggr
    aggr->imagecost = imagecost;
    aggr->src = src[1];
    aggr->Gx = &Gx[1];
    aggr->Gy = &Gy[1];
    ImaScanTreeIntAggrProc(aggr);

    // wta
    ImppOptimizeWtaPixel(aggr->smoothcost, dlength, area, eadp->right);

    // refine && fill
    ImppLrCheckPixel(eadp->left, eadp->right, height, width, 1, eadp->sparse);
    ImppBackgroundFillPixel(eadp->sparse, height, width, eadp->fill, invalid);
    ImppMedianFilterPixel(eadp->fill, eadp->dense, height, width, eadp->dispmr);

    return;
}

void EadpGradient(INOUT ImEadpInfo *eadp)
{
    int imagenum = eadp->imagenum;
    int guildmr = eadp->guildmr;
    IMAGE_S **src = eadp->src;
    IMAGE_S *filter = eadp->filter;
    IMAGE32_S *Gx = eadp->Gx;
    IMAGE32_S *Gy = eadp->Gy;

    int i;
    IMAGE_S *guild;

    for (i = 0;i < imagenum;i++)
    {
        guild = src[i];
        if (guildmr == 1)
        {
            guild = filter + i;
            ImageColorMedian3x3(src[i], guild);
        }
        else if (guildmr > 0)
        {
            int c;
            int height = eadp->height;
            int width = eadp->width;
            int channel = eadp->channel;

            guild = filter + i;
            for (c = 0;c < channel;c++)
            {
                ctmfopt(src[i]->data+c, guild->data+c, height, width, channel, guildmr);
            }
        }
        ImageGradient1(guild, Gx + i, Gy + i, VECTOR_DIST_INF);
    }

    return;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
