/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchSgm.c 			                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2015/05/05                                                          */
/*                                                                           */
/* Description: Sgm算法                                                      */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "ImageMatchSgm.h"

void ImageMatchSgmPara(INOUT ImSgmInfo *Sgm)
{
    Sgm->imagenum = 2;
    Sgm->mode = 0;
    Sgm->dispmr = 2;
    Sgm->r1 = 30;
    Sgm->r2 = 500;
}

void ImageMatchSgmInit(INOUT ImSgmInfo *Sgm)
{
    int dlength = Sgm->dlength;
    int height = Sgm->height;
    int width = Sgm->width;
    int area = height*width;

    ImaScanTreeUshortInfo *aggr = &Sgm->aggr;
    ImppInvalidInfo *invalid = &Sgm->invalid;

    Sgm->imagecost = mm_MallocType(ushort, height*width*dlength);

    ImaScanTreeUshortAggrPara(aggr);
    aggr->height = height;
    aggr->width = width;
    aggr->dlength = dlength;
    aggr->mode = Sgm->mode;
    ImaScanTreeUshortAggrInit(aggr);

    ImppInvalidInit(invalid, area);
    
    Sgm->left = MallocType(PIXEL, area);
    Sgm->right = MallocType(PIXEL, area);
    Sgm->sparse = MallocType(PIXEL, area);
    Sgm->fill = MallocType(PIXEL, area);
    Sgm->dense = MallocType(PIXEL, area);

    return;
}
void ImageMatchSgmDestroy(INOUT ImSgmInfo *Sgm)
{
    int dlength = Sgm->dlength;
    int height = Sgm->height;
    int width = Sgm->width;
    int area = height*width;

    mm_FreeType(Sgm->imagecost, ushort, height*width*dlength);

    ImaScanTreeUshortAggrDestroy(&Sgm->aggr);
    ImppInvalidDestroy(&Sgm->invalid, area);

    FreeType(Sgm->left, PIXEL, area);
    FreeType(Sgm->right, PIXEL, area);
    FreeType(Sgm->sparse, PIXEL, area);
    FreeType(Sgm->fill, PIXEL, area);
    FreeType(Sgm->dense, PIXEL, area);

    return;
}

void ImageMatchSgmProc(INOUT ImSgmInfo *Sgm)
{
    int mode = Sgm->mode;
    ImaScanTreeUshortInfo *aggr = &Sgm->aggr;
    ImppInvalidInfo *invalid = &Sgm->invalid;

    IMAGE_S **src = Sgm->src;
    ushort *imagecost = Sgm->imagecost;
    int dlength = Sgm->dlength;

    int height = Sgm->height;
    int width = Sgm->width;
    int area = height*width;

    ushort maxcost;

    // cost
    ImcPaperCensusUshort(src[0], src[1], dlength, 2, 2, &maxcost, imagecost);
    aggr->P1 = (ushort)round(Sgm->r1*maxcost/dlength);
    aggr->P2 = (ushort)round(Sgm->r2*maxcost/dlength);

    // cost aggr
    aggr->imagecost = imagecost;
    if (mode == 0)  ImaScanTreeUshortAggrProc(aggr);
    else            ImaScanTreeUshortAggrProcXdy(aggr);

    // wta
    ImppOptimizeWta16Pixel(aggr->smoothcost, dlength, area, Sgm->left);

    // invcost
    ImcDsiReverseUshort(imagecost, aggr->smoothcost, height, width, dlength, maxcost, 2);  
    Sgm->imagecost = aggr->smoothcost;
    aggr->smoothcost = imagecost;
    imagecost = Sgm->imagecost;

    // cost aggr
    aggr->imagecost = imagecost;
    if (mode == 0)  ImaScanTreeUshortAggrProc(aggr);
    else            ImaScanTreeUshortAggrProcXdy(aggr);

    // wta
    ImppOptimizeWta16Pixel(aggr->smoothcost, dlength, area, Sgm->right);

    // refine && fill
    ImppLrCheckPixel(Sgm->left, Sgm->right, height, width, 1, Sgm->sparse);
    ImppBackgroundFillPixel(Sgm->sparse, height, width, Sgm->fill, invalid);
    ImppMedianFilterPixel(Sgm->fill, Sgm->dense, height, width, Sgm->dispmr);

    return;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
