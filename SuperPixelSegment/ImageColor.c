/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageColor.c 			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/02/06                                                          */
/*                                                                           */
/* Description: 彩色图像变换                                                 */
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
#include "ImageColor.h"

/*******************************************************************************
    Func Name: ImageRgb2Gray
 Date Created: 2011-02-06
       Author: zhusong
     Function: RGB转换为灰度图像
        Input: IN IMAGE_S *srcImage, 源图像
               IN RECT_S *rect, 处理区域
       Output: OUT IMAGE_S *dstImage, 灰度图像
       Return: 
      Caution: 
      Description: YUV变换获得Y通量
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageRgb2Gray0(IN IMAGE_S *srcImage, OUT IMAGE_S *dstImage, IN RECT_S *rect)
{
    int i, j;
    int srcx, srcy;    
    double yuv[IMAGE_COLOR_YUV_MAX];

    PIXEL *dst = dstImage->data;
    PIXEL *src = srcImage->data;
    int width = srcImage->width;
    int channel = srcImage->channel;
    int dstwidth,dstheight;
    int left,top;

    if (channel != 3)   return;

    if (rect == NULL)
    {
        left = 0;
        top = 0;
        dstheight = srcImage->height;
        dstwidth = srcImage->width;
    }
    else
    {
        left = rect->left;
        top = rect->top;
        dstheight = rect->bottom - rect->top + 1;
        dstwidth = rect->right - rect->left + 1;
    }

    for(i = 0,srcy = top;i < dstheight;i++,srcy++)
	{  
		for(j = 0,srcx = left;j < dstwidth;j++,srcx++)
		{
            Rgb2Yuv(src+(srcy*width+srcx)*channel, yuv);
            dst[i*dstwidth+j] = round(yuv[IMAGE_COLOR_YUV_Y]);
		}
	}

    dstImage->height = dstheight;
    dstImage->width = dstwidth;
    dstImage->channel = 1;
    
    return;
}

void ImageRgb2Gray1(IN IMAGE_S *srcImage, OUT IMAGE_S *dstImage, IN RECT_S *rect)
{
    int i, j, x;

    PIXEL *srcblue = srcImage[IMAGE_COLOR_BLUE].data;
    PIXEL *srcgreen = srcImage[IMAGE_COLOR_GREEN].data;
    PIXEL *srcred = srcImage[IMAGE_COLOR_RED].data;
    int width = srcImage->width;
    int height = srcImage->height;
    PIXEL *dst = dstImage->data;

    int dstwidth,dstheight;
    int left,right,top,bottom;
    float gray;

    RectEnsure(rect);
    dstheight = bottom - top + 1;
    dstwidth = right - left + 1;

    srcblue += top*width;
    srcgreen += top*width;
    srcred += top*width;
    for(i = 0;i < dstheight;i++, srcblue += width, srcgreen += width, srcred += width, dst += dstwidth)
	{
		for(j = 0, x = left;j < dstwidth;j++, x++)
		{
            gray = 0.299f*srcblue[x] + 0.587f*srcgreen[x] + 0.114f*srcred[x];
            dst[j] = round(gray);
		}
	}

    dstImage->height = dstheight;
    dstImage->width = dstwidth;
    dstImage->channel = 1;
    
    return;
}

#if 0
void ImageRgb2Gray(IN IMAGE_S *srcImage, OUT IMAGE_S *dstImage, IN RECT_S *rect)
{
    if (srcImage->channel == 3)
    {
        ImageRgb2Gray0(srcImage, dstImage, NULL);
    }
    else
    {
        ImageRgb2Gray1(srcImage, dstImage, NULL);
    }
}
#else

/*******************************************************************************
    Func Name: ImageRgb2Gray
 Date Created: 2011-02-06
       Author: zhusong
     Function: RGB转换为灰度图像
        Input: IN IMAGE_S *srcImage, 源图像
       Output: OUT IMAGE_S *dstImage, 灰度图像
       Return: 
      Caution: 
      Description: YUV变换获得Y通量
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageRgb2Gray(IN IMAGE_S *srcImage, OUT IMAGE_S *dstImage, IN RECT_S *rect)
{
    int height = srcImage->height;
    int width = srcImage->width;
    int imagesize = height*width;
    int channel = srcImage->channel;
    PIXEL *dst = dstImage->data;
    PIXEL *src = srcImage->data;

    int i;
    int r, g, b;
    int rgb;

    for(i = 0;i < imagesize;i++, src += channel)
	{
	    r = src[IMAGE_COLOR_RED];
	    g = src[IMAGE_COLOR_GREEN];
	    b = src[IMAGE_COLOR_BLUE];
        rgb = 19595*r + 38470*g + 7471*b + 32768;
        dst[i] = (PIXEL)(rgb >> 16);
	}
    dstImage->height = height;
    dstImage->width = width;
    dstImage->channel = 1;
    
    return;
}
#endif

/*******************************************************************************
    Func Name: Rgb2Yuv
 Date Created: 2011-02-06
       Author: zhusong
     Function: 彩色图像变换RGB空间到YUV空间
        Input: IN PIXEL *rgb, RGB空间源像素
       Output: OUT double *yuv YUV空间像素
       Return: 
      Caution: 
      Description: Y: 亮度 [0 255]      灰度图只有Y分量
                   U: 色度 [-125 125]
                   V: 色度 [-224 224]
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void Rgb2Yuv(IN PIXEL *rgb, OUT double *yuv)
{
    yuv[IMAGE_COLOR_YUV_Y] = 0.299*rgb[IMAGE_COLOR_RED] + 0.587*rgb[IMAGE_COLOR_GREEN] + 0.114*rgb[IMAGE_COLOR_BLUE];
    yuv[IMAGE_COLOR_YUV_U] = (rgb[IMAGE_COLOR_BLUE] - yuv[IMAGE_COLOR_YUV_Y])*0.4921;
    yuv[IMAGE_COLOR_YUV_V] = (rgb[IMAGE_COLOR_RED] - yuv[IMAGE_COLOR_YUV_Y])*0.8773;

    return;
}

void MatrixSeqInit(IN Matrix_S *matseq, IN int height, IN int width, IN int channel)
{
    int i;
    int size;
    double *buf;
    Matrix_S *mat;

    size = height*width;
    buf = MallocType(double, size*channel);
    mat = matseq;
    for (i = 0; i < channel; i++, mat++)
    {
        mat->height = height;
    	mat->width = width;
    	mat->data = buf;
    	buf += size;
    }
}
void MatrixSeqDestroy(IN Matrix_S *matseq)
{
    FreeType(matseq->data, double, matseq->height*matseq->width*matseq->channel);
}
void MatrixRgb2Seq(IN Matrix_S *srcImage, INOUT Matrix_S *dstImage)
{
    int width = srcImage->width;
    int height = srcImage->height;
    double *src = srcImage->data;

    int i, j;
    double *r, *g, *b;

    r = dstImage[0].data;
    g = dstImage[1].data;
    b = dstImage[2].data;
	dstImage[0].height = height;
	dstImage[0].width = width;
	dstImage[0].channel = -2;
	for(i = 0;i < height;i++, r += width, g += width, b += width)
	{
		for(j = 0;j < width;j++)
		{
            r[j] = src[0];
            g[j] = src[1];
            b[j] = src[2];
            src += 3;
		}
    }
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
