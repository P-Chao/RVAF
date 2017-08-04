/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                              */
/*                                                                           */
/* FileName: ImageRoadDetection.c 			                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: 路面检测                                                     */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "ImageRoadDetection.h"

void ImageRdPara(OUT ImageRdInfo *rd)
{
    IerInfo *ier = &rd->ier;
    ImEadpInfo *eadp = &rd->eadp;
    ImSgmInfo *sgm = &rd->sgm;
    IrfFeaInfo *feainfo = &rd->feainfo;

    rd->dispcal = 1;
    rd->dispshow = TRUE;
    rd->segshow = TRUE;

    // IER 参数配置
    ImageIerPara(ier);
    ier->K = 100;
    ier->maxK = 2*100;
    ier->normC = 10;
    ier->optint = 1;

    // EADP 参数配置
    ImageMatchEadpPara(eadp);
    eadp->guildmr = 1;
    eadp->dispmr = 1;
    eadp->sg = -1;
    eadp->sc = 0.1*255;
    eadp->r1 = 10;
    eadp->r2 = 500;

    // SGM 参数配置
    ImageMatchSgmPara(sgm);
    sgm->dispmr = 1;
    sgm->r1 = 10;
    sgm->r2 = 500;

    // Feature 参数配置
    IrfFeaturePara(feainfo);

    return;
}

void ImageRdInfoset(OUT ImageRdInfo *rd, 
                    IN IMAGE_S *imageL, 
                    IN IMAGE_S *imageR,
                    IN int dlength)
{
    IerInfo *ier = &rd->ier;
    IrfFeaInfo *feainfo = &rd->feainfo;
    int dispcal = rd->dispcal;
    
    int height = imageL->height;
    int width = imageL->width;
    int channel = imageL->channel;

    rd->src[0] = *imageL;
    rd->src[1] = *imageR;
    rd->dlength = dlength;

    ier->height = height;
    ier->width = width;
    ier->channel = channel;
    ier->imagecolori = imageL->data;

    if (dispcal == 1)
    {
        ImEadpInfo *eadp = &rd->eadp;
        eadp->height = height;
        eadp->width = width;
        eadp->channel = channel;
        eadp->dlength = dlength;
        eadp->src[0] = imageL;
        eadp->src[1] = imageR;
    }
    if (dispcal == 2)
    {
        ImSgmInfo *sgm = &rd->sgm;
        sgm->height = height;
        sgm->width = width;
        sgm->channel = channel;
        sgm->dlength = dlength;
        sgm->src[0] = imageL;
        sgm->src[1] = imageR;
    }

    feainfo->maxHeight = height;
    feainfo->maxWidth = width;
    feainfo->maxChannel = channel;

    return;
}

void ImageRdInit(OUT ImageRdInfo *rd)
{
    IMAGE_S *src = rd->src;
    int height = src->height;
    int width = src->width;
    int channel = src->channel;
    int dispcal = rd->dispcal;
    
    IrfFeaInfo *feainfo = &rd->feainfo;
    int feanum = IrfFeatureNum(feainfo, channel);
    int regionnum = feainfo->maxRegionnum;

    if (dispcal == 1)
    {
        ImageMatchEadpInit(&rd->eadp);
    }
    if (dispcal == 2)
    {
        ImageMatchSgmInit(&rd->sgm);
    }

    ImageIerInit(&rd->ier);

    IrfFeatureInit(feainfo);
    rd->feature = MallocType(int, regionnum*feanum);
    rd->lblimg = MallocType(PIXEL, height*width);
    rd->label = MallocType(int, regionnum);

    rd->dispimg = MallocType(PIXEL, height*width);
    rd->segimg = MallocType(PIXEL, height*width);

    return;
}

void ImageRdDestroy(OUT ImageRdInfo *rd)
{
    IMAGE_S *src = rd->src;
    int height = src->height;
    int width = src->width;
    int channel = src->channel;
    int dispcal = rd->dispcal;
    
    IrfFeaInfo *feainfo = &rd->feainfo;
    int feanum = IrfFeatureNum(feainfo, channel);
    int regionnum = feainfo->maxRegionnum;

    if (dispcal == 1)
    {
        ImageMatchEadpDestroy(&rd->eadp);
    }
    if (dispcal == 2)
    {
        ImageMatchSgmDestroy(&rd->sgm);
    }

    ImageIerDestroy(&rd->ier);

    IrfFeatureInit(&rd->feainfo);

    FreeType(rd->feature, int, regionnum*feanum);
    FreeType(rd->lblimg, PIXEL, height*width);
    FreeType(rd->label, int, regionnum);

    FreeType(rd->dispimg, PIXEL, height*width);
    FreeType(rd->segimg, PIXEL, height*width);

	return;
}

void ImageRdProc(OUT ImageRdInfo *rd)
{
    int dispcal = rd->dispcal;
    IerInfo *ier = &rd->ier;
    IrfFeaInfo *feainfo = &rd->feainfo;

    IMAGE_S *src = rd->src;
    int height = src->height;
    int width = src->width;
    int channel = src->channel;
    int dlength = rd->dlength;

    int feanum;
    int regionnum;
    PIXEL *left;
    PIXEL *right;
    PIXEL *sparse;
    PIXEL *dense;
    IMAGE32_S markImage;
    IMAGE_S lblImage;

    // EADP or SGM
    if (dispcal == 1)
    {
        ImEadpInfo *eadp = &rd->eadp;
        ImageMatchEadpProc(eadp);
        left = eadp->left;
        right = eadp->right;
        sparse = eadp->sparse;
        dense = eadp->dense;
    }
    else if (dispcal == 2)
    {
        ImSgmInfo *sgm = &rd->sgm;
        ImageMatchSgmProc(sgm);
        left = sgm->left;
        right = sgm->right;
        sparse = sgm->sparse;
        dense = sgm->dense;
    }
    if (rd->dispshow)
    {
        ImppDisp2Image8(rd->dispimg, sparse, height*width, IMAGE_WHITE/dlength);
    }
    // SaveMatrixChar("E:/left.txt", left, height, width);
    // SaveMatrixChar("E:/right.txt", right, height, width);
    // SaveMatrixChar("E:/sparse.txt", sparse, height, width);
    // SaveMatrixChar("E:/dense.txt", dense, height, width);

    // IER
    IerPixelInit(ier);
    IerPixelRefine(ier);
    if (rd->dispcal)
    {
        IMAGE_S showImage;
        showImage.data = rd->segimg;
        ImageSegShow(src, ier->label, NULL, &showImage);
    }
    markImage.height = height;
    markImage.width = width;
    markImage.channel = 1;
    markImage.data = ier->label;
    regionnum = ier->num;
    // SaveMatrixInt("E:/ier.txt", ier->label, height, width);

    // Feature
    feanum = IrfFeatureNum(feainfo, channel);
    if (regionnum > feainfo->maxRegionnum)
    {
        FreeType(rd->feature, int, feainfo->maxRegionnum*feanum);
        FreeType(rd->label, int, feainfo->maxRegionnum);
        rd->feature = MallocType(int, regionnum*feanum);
        rd->label = MallocType(int, regionnum);
        IrfFeatureMemoryAdapt(feainfo, height, width, regionnum);
    }
    IrfFeatureProcInt(feainfo, &markImage, src, regionnum, rd->feature, feanum);

    // Classify
    lblImage.data = rd->lblimg;
    IrcClassifyImage(&markImage, &lblImage, rd->label, rd->feature, regionnum);
    // SaveMatrixChar("E:/lblimg.txt", rd->lblimg, height, width);

    return;
}

void TestMyData()
{
    int height = 202;
    int width = 270;
    int channel = 3;

    int dlength = 24;
    IMAGE_S imageL;
    IMAGE_S imageR;
    ImageRdInfo rd;

    imageL.height = height;
    imageL.width = width;
    imageL.channel = channel;
    imageL.data = MallocType(PIXEL, height*width*channel);

    imageR.height = height;
    imageR.width = width;
    imageR.channel = channel;
    imageR.data = MallocType(PIXEL, height*width*channel);

///////////////////////////////////////////////////////////////////////
//              载入图像数据:   imageL.data, imageR.data
///////////////////////////////////////////////////////////////////////

    printf("load image L\n");
    printf("load image R\n");

    ImageRdPara(&rd);
    ImageRdInfoset(&rd, &imageL, &imageR, dlength);
    ImageRdInit(&rd);

    while (1)
    {
        ImageRdProc(&rd);
    }

    // ImageRdDestroy(&rd);
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
