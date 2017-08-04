/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageGradient.c 	                                             */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2012/11/16                                                          */
/*                                                                           */
/* Description: 图像梯度                                                     */
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
#include "ImageGradient.h"


/*******************************************************************************
    Func Name: ImageGradient1
 Date Created: 2011-01-26
       Author: zhusong
     Function: 图像2方向梯度-1阶微分
        Input: IN IMAGE_S *srcImage, 源图像
               IN int type, 
       Output: OUT IMAGE32_S *Gx, x方向的梯度
               OUT IMAGE32_S *Gy, y方向的梯度
       Return: 无
      Caution: 
      Description: gx = I(x+1)-I(x), gy = I(y+1)-I(y)
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageGradient1(IN IMAGE_S *srcImage, 
                    OUT IMAGE32_S *Gx, 
                    OUT IMAGE32_S *Gy,
                    IN int type)
{
    PIXEL *src = srcImage->data;
    int width = srcImage->width;
    int height = srcImage->height;
    int channel = srcImage->channel;
    int widthc = width*channel;
    int *gradx = Gx->data;
    int *grady = Gy->data;

    int x, y;
    PIXEL *p;
    int *gx, *gy;
    RECT_S *rect = NULL;
    int left, right, top, bottom;

    RectEnsure(rect);

    Gx->width = width;
    Gx->height = height;
    Gx->channel = 1;
    Gy->width = width;
    Gy->height = height;
    Gy->channel = 1;

    // 求图像梯度
    src += top*widthc + left*channel;
    gx = gradx + top*width;
    gy = grady + top*width;
    for(y = top; y < bottom; y++, src += widthc, gx += width, gy += width)
    {
        p = src;
        for(x = left; x < right; x++, p += channel)
        {
            gx[x] = VectorDistPixelInt(p, p + channel, channel, type);
            gy[x] = VectorDistPixelInt(p, p + widthc, channel, type);
        }

        // 最后一列 y 方向梯度
        gy[x] = VectorDistPixelInt(p, p + widthc, channel, type);
    }

    // 最后一行 x 方向梯度
    p = src;
    for(x = left; x < right; x++, p += channel)
    {
        gx[x] = VectorDistPixelInt(p, p + channel, channel, type);
    }

    // 边界插值 最后一列 x 方向梯度和最后一行 y 方向梯度
    gx = gradx + top*width + right;
    for(y = top; y <= bottom; y++, gx += width)
    {
        gx[0] = gx[-1];
    }
    gy = grady + bottom*width;
    for(x = left; x <= right; x++)
    {
        gy[x] = gy[x - width];
    }

    return;
}

/*******************************************************************************
    Func Name: ImageGradientX2
 Date Created: 2011-01-26
       Author: zhusong
     Function: 图像2方向梯度-2阶微分
        Input: IN IMAGE_S *srcImage, 源图像
               IN int type, 梯度距离函数
       Output: OUT IMAGE32_S *Gx, x方向的梯度
       Return: 无
      Caution: 
      Description: 实际值是中值差分的2倍 !

                   内部像素采用中值差分:
                   dx(i,j) = [I(i+1,j) - I(i-1,j)]/2;
                   dy(i,j) = [I(i,j+1) - I(i,j-1)]/2;
                   边界像素采用差分:
                   dx(i,j) = I(i+1,j) - I(i,j);
                   dy(i,j) = I(i,j+1) - I(i,j);
                   等价于 matlab函数[GradX GradY] = gradient(srcImage)
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageGradientX2(IN IMAGE_S *srcImage,
                     OUT IMAGE32_S *Gx,
                     IN int type)
{
    PIXEL *image = srcImage->data;
    int width = srcImage->width;
    int height = srcImage->height;
    int channel = srcImage->channel;
    int channel2 = 2*channel;
    int *gx = Gx->data;

    int x, y;
    PIXEL *px;

    RECT_S *rect = NULL;
    int left, right, top, bottom;

    RectEnsure(rect);

    for(y = top; y <= bottom; y++, gx += width)
    {
        x = left;
        px = image + (y*width+x)*channel;

        // 左端点
        gx[x] = 2*VectorDistPixelInt(px + channel, px, channel, type);

        // 中间点
        for(x++; x < right; x++)
        {
            gx[x] = VectorDistPixelInt(px + channel2, px, channel, type);
            px += channel;
        }

        // 右端点
        gx[x] = 2*VectorDistPixelInt(px + channel, px, channel, type);
    }

    Gx->width = width;
    Gx->height = height;
    Gx->channel = 1;

    return;
}

/*******************************************************************************
    Func Name: ImageGradientX2
 Date Created: 2011-01-26
       Author: zhusong
     Function: 
        Input: IN IMAGE_S *srcImage, 源图像
               IN int trunc, 截断值
       Output: OUT IMAGE32_S *Gx, x方向的梯度
       Return: 无
      Caution: 
      Description: -1  0   1
                   -2  0   2
                   -1  0   1
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageSobelHor(IN IMAGE_S *srcImage,
                   OUT IMAGE32_S *Gx,
                   IN int trunc,
                   IN int c)
{
    int maxv = 2*trunc;
    PIXEL *image = srcImage->data + c;
    int width = srcImage->width;
    int height = srcImage->height;
    int channel = srcImage->channel;
    int *grad = Gx->data;

    int x, y;
    PIXEL *px1,*px2,*px3;
    int sobel;
    int *gx;

    RECT_S *rect = NULL;
    int left, right, top, bottom;

    RectEnsure(rect);

    // 中间点
    gx = grad + (top+1)*width;
    for(y = top+1; y < bottom; y++, gx += width)
    {
        x = left+1;
        px1 = image + ((y-1)*width+x)*channel;
        px2 = image + (y*width+x)*channel;
        px3 = image + ((y+1)*width+x)*channel;
        for(; x < right; x++)
        {
            sobel = (px1[channel]-px1[-channel]) + 
                    (px2[channel]-px2[-channel])*2 + 
                    (px3[channel]-px3[-channel]);
            sobel += trunc;
            sobel = (0 > sobel) ? 0 : sobel;
            sobel = (maxv < sobel) ? maxv : sobel;
            gx[x] = sobel;
            px1 += channel;
            px2 += channel;
            px3 += channel;
        }
    }

    // 边缘
    gx = grad + top*width;
    for(x = left+1; x < right; x++)             gx[x] = gx[x+width];
    gx = grad + bottom*width;
    for(x = left+1; x < right; x++)             gx[x] = gx[x-width];
    gx = grad;
    for(y = top; y <= bottom; y++, gx += width) gx[0] = gx[1];
    gx = grad + right;
    for(y = top; y <= bottom; y++, gx += width) gx[0] = gx[-1];

    Gx->width = width;
    Gx->height = height;
    Gx->channel = 1;

    return;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
