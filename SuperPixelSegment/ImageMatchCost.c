/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageMatchCost.c 	                                             */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/06/16                                                          */
/*                                                                           */
/* Description: 图像匹配值计算                                               */
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
#include "string.h"
#include "ImageMatchCost.h"

void ImcPaperCensusInt(IN IMAGE_S *imageL,
                       IN IMAGE_S *imageR,
                       IN int dlength,
                       IN int cw,
                       IN int ch,
                       OUT int *mcost,
                       OUT int *pixeldsi)
{
    int height = imageL->height;
    int width = imageL->width;
    int channel = imageL->channel;

    int maxcost;
    IMAGE_S imageLg, imageRg;
    IMAGE_S *srcL, *srcR;

    maxcost = (cw*2+1)*(ch*2+1)-1;
    *mcost = maxcost;

    // RGB2GRAY
    if (channel != 1)
    {
        imageLg.data = MallocType(PIXEL, height*width);
        imageRg.data = MallocType(PIXEL, height*width);
        srcL = &imageLg;
        srcR = &imageRg;
        ImageRgb2Gray(imageL, srcL, NULL);
        ImageRgb2Gray(imageR, srcR, NULL);
    }
    else
    {
        srcL = imageL;
        srcR = imageR;
    }

	// 匹配值: Census
    ImcImageCensusInt(srcL, srcR, cw, ch, dlength, pixeldsi);

    if (channel != 1)
    {
        FreeType(imageLg.data, PIXEL, height*width);
        FreeType(imageRg.data, PIXEL, height*width);
    }

    return;
}
#if 0
void ImcImageCensusInt(IN IMAGE_S *imageL, 
                       IN IMAGE_S *imageR, 
                       IN int cwx,
                       IN int cwy,
                       IN int dlength,
                       OUT int *imagedsi)
{
    int width = imageL->width;
    int height = imageL->height;
	int widthd = width*dlength;

    int y, x, d;
    int *pdsi, *pdsi0;
    PIXEL *dist;
    int borderval;

    int bit, flag64;
    IMAGE_S imageDist;
    IMAGE64_S censusL64, censusR64;
    IMAGE32_S censusL32, censusR32;
    RECT_S rectL, rectR;

    bit = (2*cwx+1)*(2*cwy+1);
    if (bit >= 32)  flag64 = 1;
    else            flag64 = 0;

    // 申请内存
    imageDist.height = height;
    imageDist.width = width;
    imageDist.channel = 1;
    imageDist.data = MallocType(PIXEL, height*width);

    // Census变换
    if (flag64)
    {
        censusL64.data = MallocType(int64, height*width);
        censusR64.data = MallocType(int64, height*width);
        ImageCensus(imageL, &censusL64, cwx, cwy);
        ImageCensus(imageR, &censusR64, cwx, cwy);
    }
    else
    {
        censusL32.data = MallocType(int, height*width);
        censusR32.data = MallocType(int, height*width);
        ImageCensus32(imageL, &censusL32, cwx, cwy);
        ImageCensus32(imageR, &censusR32, cwx, cwy);
    }

    // 匹配值
    rectL.top = rectR.top = 0;
    rectL.bottom = rectR.bottom = height-1;
    for (d = 0;d < dlength;d++)
    {
        rectL.left = d;
        rectL.right = width-1;
        rectR.left = 0;
        rectR.right = width - 1 - d;

        if (flag64)
        {
            ImageHamming(&censusL64, &censusR64, &rectL, &rectR, &rectL, &imageDist);
        }
        else
        {
            ImageHamming32(&censusL32, &censusR32, &rectL, &rectR, &rectL, &imageDist);
        }

        dist = imageDist.data;
		pdsi0 = imagedsi + d;
        for (y = 0;y < height;y++, dist += width, pdsi0 += widthd)
        {
            // 非边界
            pdsi = pdsi0 + d*dlength;
            for (x = d;x < width;x++, pdsi += dlength)
            {
                *pdsi = dist[x];
            }

			// 边界
			pdsi = pdsi0;
            for (x = 0;x < d;x++, pdsi += dlength)
            {
				borderval = pdsi[-1];
                *pdsi = borderval;
            }
        }
    }

    FreeType(imageDist.data, PIXEL, height*width);
    if (flag64)
    {
        FreeType(censusL64.data, int64, height*width);
        FreeType(censusR64.data, int64, height*width);
    }
    else
    {
        FreeType(censusL32.data, int64, height*width);
        FreeType(censusR32.data, int64, height*width);
    }

    return;
}
#else
void CensusCostInt(IN IMAGE32_S *censusL32, 
                   IN IMAGE32_S *censusR32,
                   IN int dlength,
                   OUT int *imagedsi)
{
    int width = censusL32->width;
    int height = censusL32->height;
    int *CL = censusL32->data;
    int *CR = censusR32->data;
    int maxdisp = dlength - 1;

    int y, x, d;
    int maxd;
	int *pdsi;
    int borderval;
    int cl, cr, temp;

#ifdef PLATFORM_CCS
    mix32 bitcount;
    unit8 *ui8 = bitcount.ui8; 
#endif

	pdsi = imagedsi;
    for (y = 0;y < height;y++, CL += width, CR += width)
    {
        for (x = 0;x < width;x++, pdsi += dlength)
        {
            maxd = (x < maxdisp) ? x : maxdisp;

            // 非边界
            cl = CL[x];
            for (d = 0;d <= maxd;d++)
            {
                cr = CR[x - d];
                temp = cl ^ cr;

#ifdef PLATFORM_SSE
                pdsi[d] = _mm_popcnt_u32(temp);
#elif defined PLATFORM_CCS
                bitcount.ui32 = _bitc4(temp);
                pdsi[d] = ui8[0] + ui8[1] + ui8[2] + ui8[3];
#else
                pdsi[d] = BitCountU32(temp);
#endif
            }

            // 边界
            borderval = pdsi[maxd];
            for (d = maxd+1;d < dlength;d++)
            {
                pdsi[d] = borderval;
            }
        }
    }
    return;
}

void ImcImageCensusInt(IN IMAGE_S *imageL, 
                       IN IMAGE_S *imageR, 
                       IN int cwx,
                       IN int cwy,
                       IN int dlength,
                       OUT int *imagedsi)
{
    int width = imageL->width;
    int height = imageL->height;

    int bit, flag64;
    IMAGE64_S censusL64, censusR64;
    IMAGE32_S censusL32, censusR32;

    bit = (2*cwx+1)*(2*cwy+1);
    if (bit >= 32)  flag64 = 1;
    else            flag64 = 0;

    // Census变换
    if (flag64)
    {
        censusL64.data = MallocType(int64, height*width);
        censusR64.data = MallocType(int64, height*width);
        ImageCensus(imageL, &censusL64, cwx, cwy);
        ImageCensus(imageR, &censusR64, cwx, cwy);
    }
    else
    {
        censusL32.data = MallocType(int, height*width);
        censusR32.data = MallocType(int, height*width);
        ImageCensus32(imageL, &censusL32, cwx, cwy);
        ImageCensus32(imageR, &censusR32, cwx, cwy);
    }

    CensusCostInt(&censusL32, &censusR32, dlength, imagedsi);

    if (flag64)
    {
        FreeType(censusL64.data, int64, height*width);
        FreeType(censusR64.data, int64, height*width);
    }
    else
    {
        FreeType(censusL32.data, int64, height*width);
        FreeType(censusR32.data, int64, height*width);
    }

    return;
}
#endif

// 将left图像上的cost变换到right图像上
void ImcDsiReverseInt(IN int *dsi, 
                      OUT int *invdsi, 
                      IN int height, 
                      IN int width, 
                      IN int dlength,
                      IN int bordercost,
                      IN int intp)
{
    int d, y;
    int xL, xR;
    int border;
    int maxdisp;
    int rightedge;
    int wdlength;
    int *xinvdsi, *yinvdsi;
    int *dsiL, *dsiR;
    int cost;

    maxdisp = dlength-1;
    rightedge = width-maxdisp;
    wdlength = width*dlength;
    dsiL = dsi;
    yinvdsi = invdsi;
    for (y = 0;y < height;y++, yinvdsi += wdlength)
    {
        // 左边界
        xinvdsi = yinvdsi;
        for (xL = 0;xL < dlength;xL++, dsiL += dlength, xinvdsi += dlength)
        {
            dsiR = xinvdsi;
            for (d = 0;d <= xL;d++)
            {
                // invdsi(y, xR, d) = dsi(y, xL, d)
                cost = dsiL[d];
                dsiR[d] = cost;

                // xR = xL-d (向左移动)
                dsiR -= dlength;
            }
        }

        // 中间
        for (xL = dlength;xL < width;xL++, dsiL += dlength, xinvdsi += dlength)
        {
            dsiR = xinvdsi;
            for (d = 0;d < dlength;d++)
            {
                // invdsi(y, xR, d) = dsi(y, xL, d)
                cost = dsiL[d];
                dsiR[d] = cost;

                // xR = xL-d (向左移动)
                dsiR -= dlength;
            }
        }

        // 右边界(填充边界值)
		if (intp == 1)
		{
			// c(x, d) = c(x-1, d) : d通道下cost右边界复制
			for (d = 1;d < dlength;d++)
			{
				rightedge = width-d;
				xinvdsi = yinvdsi + rightedge*dlength;
				bordercost = xinvdsi[d - dlength];	// c(rightedge-1, d)
				for (xR = rightedge;xR < width;xR++, xinvdsi += dlength)
				{
					xinvdsi[d] = bordercost;
				}
			}
		}
		else
		{
			xinvdsi = yinvdsi + rightedge*dlength;
	        for (xR = rightedge, border = maxdisp;xR < width;xR++, border--, xinvdsi += dlength)
	        {
	        	if (intp == 2)
	        	{
	        		// c(x, d) = c(x, d-1) : 左图像的右边界复制
					bordercost = xinvdsi[border-1];
				}
	            for (d = maxdisp;d >= border;d--)
	            {
	            	// c(x, d) = B : 固定值
	                xinvdsi[d] = bordercost;
	            }
	        }
		}
    }

    return;
}

void ImcPaperCensusUshort(IN IMAGE_S *imageL,
                          IN IMAGE_S *imageR,
                          IN int dlength,
                          IN int cw,
                          IN int ch,
                          OUT ushort *mcost,
                          OUT ushort *pixeldsi)
{
    int height = imageL->height;
    int width = imageL->width;
    int channel = imageL->channel;

    ushort maxcost;
    IMAGE_S imageLg, imageRg;
    IMAGE_S *srcL, *srcR;

    maxcost = (cw*2+1)*(ch*2+1)-1;
    *mcost = maxcost;

    // RGB2GRAY
    if (channel != 1)
    {
        imageLg.data = MallocType(PIXEL, height*width);
        imageRg.data = MallocType(PIXEL, height*width);
        srcL = &imageLg;
        srcR = &imageRg;
        ImageRgb2Gray(imageL, srcL, NULL);
        ImageRgb2Gray(imageR, srcR, NULL);
    }
    else
    {
        srcL = imageL;
        srcR = imageR;
    }

	// 匹配值: Census
    ImcImageCensusUshort(srcL, srcR, cw, ch, dlength, pixeldsi);

    if (channel != 1)
    {
        FreeType(imageLg.data, PIXEL, height*width);
        FreeType(imageRg.data, PIXEL, height*width);
    }

    return;
}

#if 0
void ImcImageCensusUshort(IN IMAGE_S *imageL, 
                          IN IMAGE_S *imageR, 
                          IN int cwx,
                          IN int cwy,
                          IN int dlength,
                          OUT ushort *imagedsi)
{
    int width = imageL->width;
    int height = imageL->height;
	int widthd = width*dlength;

    int y, x, d;
    ushort *pdsi, *pdsi0;
    PIXEL *dist;
    ushort borderval;

    int bit, flag64;
    IMAGE_S imageDist;
    IMAGE64_S censusL64, censusR64;
    IMAGE32_S censusL32, censusR32;
    RECT_S rectL, rectR;

    bit = (2*cwx+1)*(2*cwy+1);
    if (bit >= 32)  flag64 = 1;
    else            flag64 = 0;

    // 申请内存
    imageDist.height = height;
    imageDist.width = width;
    imageDist.channel = 1;
    imageDist.data = MallocType(PIXEL, height*width);

    // Census变换
    if (flag64)
    {
        censusL64.data = MallocType(int64, height*width);
        censusR64.data = MallocType(int64, height*width);
        ImageCensus(imageL, &censusL64, cwx, cwy);
        ImageCensus(imageR, &censusR64, cwx, cwy);
    }
    else
    {
        censusL32.data = MallocType(int, height*width);
        censusR32.data = MallocType(int, height*width);
        ImageCensus32(imageL, &censusL32, cwx, cwy);
        ImageCensus32(imageR, &censusR32, cwx, cwy);
    }

    // 匹配值
    rectL.top = rectR.top = 0;
    rectL.bottom = rectR.bottom = height-1;
    for (d = 0;d < dlength;d++)
    {
        rectL.left = d;
        rectL.right = width-1;
        rectR.left = 0;
        rectR.right = width - 1 - d;

        if (flag64)
        {
            ImageHamming(&censusL64, &censusR64, &rectL, &rectR, &rectL, &imageDist);
        }
        else
        {
            ImageHamming32(&censusL32, &censusR32, &rectL, &rectR, &rectL, &imageDist);
        }

        dist = imageDist.data;
		pdsi0 = imagedsi + d;
        for (y = 0;y < height;y++, dist += width, pdsi0 += widthd)
        {
            // 非边界
            pdsi = pdsi0 + d*dlength;
            for (x = d;x < width;x++, pdsi += dlength)
            {
                *pdsi = dist[x];
            }

			// 边界
			pdsi = pdsi0;
            for (x = 0;x < d;x++, pdsi += dlength)
            {
				borderval = pdsi[-1];
                *pdsi = borderval;
            }
        }
    }

    FreeType(imageDist.data, PIXEL, height*width);
    if (flag64)
    {
        FreeType(censusL64.data, int64, height*width);
        FreeType(censusR64.data, int64, height*width);
    }
    else
    {
        FreeType(censusL32.data, int64, height*width);
        FreeType(censusR32.data, int64, height*width);
    }

    return;
}
#else
void CensusCostUshort(IN IMAGE32_S *censusL32, 
                      IN IMAGE32_S *censusR32,
                      IN int dlength,
                      OUT ushort *imagedsi)
{
    int width = censusL32->width;
    int height = censusL32->height;
    int *CL = censusL32->data;
    int *CR = censusR32->data;
    int maxdisp = dlength - 1;

    int y, x, d;
    int maxd;
	ushort *pdsi;
    ushort borderval;
    int cl, cr, temp;

#ifdef PLATFORM_CCS
    mix32 bitcount;
    unit8 *ui8 = bitcount.ui8; 
#endif

	pdsi = imagedsi;
    for (y = 0;y < height;y++, CL += width, CR += width)
    {
        for (x = 0;x < width;x++, pdsi += dlength)
        {
            maxd = (x < maxdisp) ? x : maxdisp;

            // 非边界
            cl = CL[x];
            for (d = 0;d <= maxd;d++)
            {
                cr = CR[x - d];
                temp = cl ^ cr;

#ifdef PLATFORM_SSE
                pdsi[d] = (ushort)_mm_popcnt_u32(temp);
#elif defined PLATFORM_CCS
                bitcount.ui32 = _bitc4(temp);
                pdsi[d] = ui8[0] + ui8[1] + ui8[2] + ui8[3];
#else
                pdsi[d] = (ushort)BitCountU32(temp);
#endif
            }

            // 边界
            borderval = pdsi[maxd];
            for (d = maxd+1;d < dlength;d++)
            {
                pdsi[d] = borderval;
            }
        }
    }

    return;
}

void ImcImageCensusUshort(IN IMAGE_S *imageL, 
                          IN IMAGE_S *imageR, 
                          IN int cwx,
                          IN int cwy,
                          IN int dlength,
                          OUT ushort *imagedsi)
{
    int width = imageL->width;
    int height = imageL->height;

    int bit, flag64;
    IMAGE64_S censusL64, censusR64;
    IMAGE32_S censusL32, censusR32;

    bit = (2*cwx+1)*(2*cwy+1);
    if (bit >= 32)  flag64 = 1;
    else            flag64 = 0;

    // Census变换
    if (flag64)
    {
        censusL64.data = MallocType(int64, height*width);
        censusR64.data = MallocType(int64, height*width);
        ImageCensus(imageL, &censusL64, cwx, cwy);
        ImageCensus(imageR, &censusR64, cwx, cwy);
    }
    else
    {
        censusL32.data = MallocType(int, height*width);
        censusR32.data = MallocType(int, height*width);
        ImageCensus32(imageL, &censusL32, cwx, cwy);
        ImageCensus32(imageR, &censusR32, cwx, cwy);
    }

    CensusCostUshort(&censusL32, &censusR32, dlength, imagedsi);

    if (flag64)
    {
        FreeType(censusL64.data, int64, height*width);
        FreeType(censusR64.data, int64, height*width);
    }
    else
    {
        FreeType(censusL32.data, int64, height*width);
        FreeType(censusR32.data, int64, height*width);
    }

    return;
}
#endif

void ImcDsiReverseUshort(IN ushort *dsi, 
                         OUT ushort *invdsi, 
                         IN int height, 
                         IN int width, 
                         IN int dlength,
                         IN ushort bordercost,
                         IN int intp)
{
    int d, y;
    int xL, xR;
    int border;
    int maxdisp;
    int rightedge;
    int wdlength;
    ushort *xinvdsi, *yinvdsi;
    ushort *dsiL, *dsiR;
    ushort cost;

    maxdisp = dlength-1;
    rightedge = width-maxdisp;
    wdlength = width*dlength;
    dsiL = dsi;
    yinvdsi = invdsi;
    for (y = 0;y < height;y++, yinvdsi += wdlength)
    {
        // 左边界
        xinvdsi = yinvdsi;
        for (xL = 0;xL < dlength;xL++, dsiL += dlength, xinvdsi += dlength)
        {
            dsiR = xinvdsi;
            for (d = 0;d <= xL;d++)
            {
                // invdsi(y, xR, d) = dsi(y, xL, d)
                cost = dsiL[d];
                dsiR[d] = cost;

                // xR = xL-d (向左移动)
                dsiR -= dlength;
            }
        }

        // 中间
        for (xL = dlength;xL < width;xL++, dsiL += dlength, xinvdsi += dlength)
        {
            dsiR = xinvdsi;
            for (d = 0;d < dlength;d++)
            {
                // invdsi(y, xR, d) = dsi(y, xL, d)
                cost = dsiL[d];
                dsiR[d] = cost;

                // xR = xL-d (向左移动)
                dsiR -= dlength;
            }
        }

        // 右边界(填充边界值)
		if (intp == 1)
		{
			// c(x, d) = c(x-1, d) : d通道下cost右边界复制
			for (d = 1;d < dlength;d++)
			{
				rightedge = width-d;
				xinvdsi = yinvdsi + rightedge*dlength;
				bordercost = xinvdsi[d - dlength];	// c(rightedge-1, d)
				for (xR = rightedge;xR < width;xR++, xinvdsi += dlength)
				{
					xinvdsi[d] = bordercost;
				}
			}
		}
		else
		{
			xinvdsi = yinvdsi + rightedge*dlength;
	        for (xR = rightedge, border = maxdisp;xR < width;xR++, border--, xinvdsi += dlength)
	        {
	        	if (intp == 2)
	        	{
	        		// c(x, d) = c(x, d-1) : 左图像的右边界复制
					bordercost = xinvdsi[border-1];
				}
	            for (d = maxdisp;d >= border;d--)
	            {
	            	// c(x, d) = B : 固定值
	                xinvdsi[d] = bordercost;
	            }
	        }
		}
    }

    return;
}




#ifdef __cplusplus
}
#endif /* end of __cplusplus */
