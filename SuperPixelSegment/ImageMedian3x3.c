/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMedian3x3.c 			                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: 空文件，模板                                                 */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseTypeDef.h"
//#include "ImageMedian3x3.h"
#include "ctmfopt.h"

#ifdef PLATFORM_CCS
void ImageGrayMedian3x3(IN IMAGE_S *src, OUT IMAGE_S *filter)
{
    int height = src->height;
    int width = src->width;
    PIXEL *srcdata = src->data;
    PIXEL *dstdata = filter->data;

    int i, j;
    PIXEL *srctemp;
    PIXEL *dsttemp;
    PIXEL *p, *q, *t;

    int enheight = height + 2;
    int enwidth1 = width + 2;
    int enwidth2;

    // 确定宽度
    i = (enwidth1 >> 2) << 2;
    j = enwidth1 - i;
    enwidth2 = i + ((j > 0) << 2);

    srctemp = MallocType(PIXEL, enheight*enwidth2);
    dsttemp = MallocType(PIXEL, enwidth2);

    // 扩展数据
    q = srctemp + enwidth2 + 1;
    p = srcdata;
    for (i = 0;i < height;i++, p += width, q += enwidth2)
    {
        // 复制有效数据
        memcpy(q, p, width*sizeof(PIXEL));

        // 复制前后两列
        q[-1] = q[0];
        q[width] = q[width-1];
    }

    // 复制前后两行
    p = srctemp + enwidth2;
    q = srctemp;
    memcpy(q, p, enwidth2*sizeof(PIXEL));
    p = srctemp + height * enwidth2;
    q = p + enwidth2;
    memcpy(q, p, enwidth2*sizeof(PIXEL));

    // 中值滤波
    p = srctemp;
    q = dstdata;
    for (i = 0;i < height;i++, p += enwidth2, q += width)
    {
        IMG_median_3x3_8(p, enwidth2, dsttemp);
        t = dsttemp + 2;
        memcpy(q, t, width*sizeof(PIXEL));
    }

    FreeType(srctemp, PIXEL, enheight*enwidth2);
    FreeType(dsttemp, PIXEL, enwidth2);

    return;
}

void ImageColorMedian3x3(IN IMAGE_S *src, OUT IMAGE_S *filter)
{
    int height = src->height;
    int width = src->width;
    int channel = src->channel;
    PIXEL *srcdata = src->data;
    PIXEL *dstdata = filter->data;

    int i, j, c;
    PIXEL *srctemp;
    PIXEL *dsttemp;
    PIXEL *p, *q, *t;

    int enheight = height + 2;
    int enwidth1 = width + 2;
    int enwidth2;

    // 确定宽度
    i = (enwidth1 >> 2) << 2;
    j = enwidth1 - i;
    enwidth2 = i + ((j > 0) << 2);

    srctemp = MallocType(PIXEL, enheight*enwidth2);
    dsttemp = MallocType(PIXEL, enwidth2);
    for (c = 0;c < channel;c++)
    {
        // 扩展数据
        q = srctemp + enwidth2 + 1;
        p = srcdata + c;
        for (i = 0;i < height;i++, q += enwidth2)
        {
            // 复制有效数据
            for (j = 0;j < width;j++, p += channel)
            {
                q[j] = *p;
            }

            // 复制前后两列
            q[-1] = q[0];
            q[width] = q[width-1];
        }

        // 复制前后两行
        p = srctemp + enwidth2;
        q = srctemp;
        memcpy(q, p, enwidth2*sizeof(PIXEL));
        p = srctemp + height * enwidth2;
        q = p + enwidth2;
        memcpy(q, p, enwidth2*sizeof(PIXEL));

        // 中值滤波
        p = srctemp;
        for (i = 0;i < height;i++, p += enwidth2)
        {
            IMG_median_3x3_8(p, enwidth2, dsttemp);
            t = dsttemp + 2;
            q = dstdata + i*width*channel + c;
            for (j = 0;j < width;j++,q += channel)
            {
                *q = t[j];
            }
        }
    }

    FreeType(srctemp, PIXEL, enheight*enwidth2);
    FreeType(dsttemp, PIXEL, enwidth2);

    return;
}
#else
void ImageGrayMedian3x3(IN IMAGE_S *src, OUT IMAGE_S *filter)
{
    int height = src->height;
    int width = src->width;
    ctmfopt(src->data, filter->data, height, width, 1, 1);
}
void ImageColorMedian3x3(IN IMAGE_S *src, OUT IMAGE_S *filter)
{
    int c;
    int height = src->height;
    int width = src->width;
    int channel = src->channel;

    for (c = 0;c < channel;c++)
    {
        ctmfopt(src->data+c, filter->data+c, height, width, channel, 1);
    }
}
#endif

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
