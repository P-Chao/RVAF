/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageIO.cpp    			                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/25                                                          */
/*                                                                           */
/* Description: 图像读取保存功能，OpenCv 1.0 不支持16位图像                  */
/*                                                                           */
/* Others: 这一部分主要用于调试，分析，与实际应用无关。采用OpenCv图像处理库  */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#include "ImageIO.h"

/*******************************************************************************
    Func Name: ImageOpenCv2Hustsong
 Date Created: 2011-01-25
       Author: zhusong
     Function: OpenCv存储方式转换为矩阵存储方式
        Input: IN IplImage *pIamge, OpenCv存储方式的图像
       Output: OUT IMAGE_S *img, 矩阵存储方式的图像
       Return: 无
      Caution: 需要OpenCv支持
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageOpenCv2Hustsong(IN IplImage *pIamge, OUT IMAGE_S *img)
{
    int i,j;
    int color;
    PIXEL c;
    int channel = img->channel;
    int flag = channel;

	img->height = pIamge->height;
	img->width = pIamge->width;
	if (channel < 0)
    {
        channel = img->channel = pIamge->nChannels;
    }

    // OpenCv里面的图像数据组织并不是连续的
    // memcpy(img->data, pIamge->imageData,size*sizeof(PIXEL) );

    if (flag != IMAGE_SEQ_PARALLEL || channel == 1)
    {
        for(i = 0;i < pIamge->height;i++)
    	{  
    		for(j = 0;j < pIamge->width;j++)
    		{
    		    if (pIamge->nChannels == 3)
    		    {
        		    for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
        		    {
                        c = CV_IMAGE_ELEM( pIamge, PIXEL, i, j*pIamge->nChannels+color );
            			img->data[(i*pIamge->width+j)*channel+color] = c;
        		    }
    		    }
				else if (pIamge->nChannels == 4){
					for (color = IMAGE_COLOR_BLUE; color < IMAGE_COLOR_MAX; color++)
					{
						c = CV_IMAGE_ELEM(pIamge, PIXEL, i, j*pIamge->nChannels + color);
						img->data[(i*pIamge->width + j)*3 + color] = c;
						img->channel = 3;
					}
				}
    		    else
    		    {
                    c = CV_IMAGE_ELEM( pIamge, PIXEL, i, j);
            		img->data[i*pIamge->width+j] = c;
    		    }
    		}
    	}
    }
    else{     // channel == 3                
        PIXEL *dst;
		
			for (color = IMAGE_COLOR_BLUE; color < IMAGE_COLOR_MAX; color++)
			{
				dst = img[color].data;
				img[color].height = pIamge->height;
				img[color].width = pIamge->width;
				img[color].channel = IMAGE_SEQ_PARALLEL;

				for (i = 0; i < pIamge->height; i++)
				{
					for (j = 0; j < pIamge->width; j++)
					{
						c = CV_IMAGE_ELEM(pIamge, PIXEL, i, j*pIamge->nChannels + color);
						dst[i*pIamge->width + j] = c;
					}
				}
			}
		
    }
	

    return;
}

/*******************************************************************************
    Func Name: ImageHustsong2OpenCv
 Date Created: 2011-01-25
       Author: zhusong
     Function: 矩阵存储方式转换为OpenCv存储方式
        Input: IN IMAGE_S *img, 矩阵存储方式的图像
               IN RECT_S *rect, 转换的图像区域
       Output: OUT IplImage* pIamge, OpenCv存储方式的图像
       Return: 无
      Caution: 需要OpenCv支持
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageHustsong2OpenCv(IN IMAGE_S *img, OUT IplImage *pIamge, IN RECT_S *rect)
{
    int i,j;
    int color;
    int Imagewidth = img->width;
    int flag = img->channel;
    int channel;
    int width, height;
    PIXEL *dst;
    int left,top;

    if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
    else if (flag > 3)              channel = 3;
    else                            channel = flag;

    if (rect == NULL)
    {
        left = 0;
        top = 0;
        height = img->height;
        width = img->width;
    }
    else
    {
        left = rect->left;
        top = rect->top;
        height = rect->bottom - rect->top + 1;
        width = rect->right - rect->left + 1;
    }

    if (flag != IMAGE_SEQ_PARALLEL)
    {
        for(i = 0;i < height;i++)
    	{  
    		for(j = 0;j < width;j++)
    		{
    		    if (channel == 3)
    		    {
        		    for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
        		    {
        		        dst = &CV_IMAGE_ELEM( pIamge, PIXEL, i, j*channel+color );
        		        *dst = img->data[((i+top)*Imagewidth+(j+left))*img->channel+color];
        		    }
    		    }
    		    else
    		    {
                    dst = &CV_IMAGE_ELEM( pIamge, PIXEL, i, j );
                    *dst = img->data[(i+top)*Imagewidth+(j+left)];
    		    }
    		}
    	}
    }
    else
    {
        PIXEL *imagedata;
        for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
        {
            imagedata = img[color].data;
        	for(i = 0;i < height;i++)
        	{  
        		for(j = 0;j < width;j++)
        		{
        		    dst = &CV_IMAGE_ELEM( pIamge, PIXEL, i, j*channel+color );
                    *dst = imagedata[(i+top)*Imagewidth+(j+left)];
        		}
        	}
        }
    }

    return;
}

IplImage *ImageHustsong2OpenCv(IN IMAGE_S *img, IN RECT_S *rect)
{
    IplImage* pNewImg;
    CvSize size;
    int channel;
    int flag = img->channel;

    if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
    else if (flag > 3)              channel = 3;
    else                            channel = flag;

    if (rect == NULL)
    {
        size.height = img->height;
        size.width = img->width;
    }
    else
    {
        size.height = rect->bottom - rect->top + 1;
        size.width = rect->right - rect->left + 1;
    }

	pNewImg = cvCreateImage(size,IPL_DEPTH_8U,channel);
    ImageHustsong2OpenCv(img, pNewImg, rect);

    return pNewImg;
}

void Image16OpenCv2Hustsong(IN IplImage *pIamge, OUT IMAGE32_S *img)
{
    int i,j;
    int color;
    int c;
    int channel = img->channel;
    int flag = channel;

	img->height = pIamge->height;
	img->width = pIamge->width;
	if (channel < 0)
    {
        channel = img->channel = pIamge->nChannels;
    }

    if (flag != IMAGE_SEQ_PARALLEL || channel == 1)
    {
        for(i = 0;i < pIamge->height;i++)
    	{
    		for(j = 0;j < pIamge->width;j++)
    		{
    		    if (pIamge->nChannels == 3)
    		    {
        		    for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
        		    {
                        c = CV_IMAGE_ELEM( pIamge, unit16, i, j*pIamge->nChannels+color );
            			img->data[(i*pIamge->width+j)*channel+color] = c;
        		    }
    		    }
    		    else
    		    {
                    c = CV_IMAGE_ELEM( pIamge, unit16, i, j);
            		img->data[i*pIamge->width+j] = c;
    		    }
    		}
    	}
    }
    else    // channel == 3
    {
        int *dst;

        for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
        {
            dst = img[color].data;
            img[color].height = pIamge->height;
        	img[color].width = pIamge->width;
            img[color].channel = IMAGE_SEQ_PARALLEL;

        	for(i = 0;i < pIamge->height;i++)
        	{  
        		for(j = 0;j < pIamge->width;j++)
        		{
        		    c = CV_IMAGE_ELEM( pIamge, unit16, i, j*pIamge->nChannels+color );
            	    dst[i*pIamge->width+j] = c;
        		}
        	}
        }
    }

    return;
}

void Image16Hustsong2OpenCv(IN IMAGE32_S *img, OUT IplImage *pIamge, IN RECT_S *rect)
{
    int i,j;
    int color;
    int Imagewidth = img->width;
    int flag = img->channel;
    int channel;
    int width, height;
    unit16 *dst;
    int left,top;

    if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
    else if (flag > 3)              channel = 3;
    else                            channel = flag;

    if (rect == NULL)
    {
        left = 0;
        top = 0;
        height = img->height;
        width = img->width;
    }
    else
    {
        left = rect->left;
        top = rect->top;
        height = rect->bottom - rect->top + 1;
        width = rect->right - rect->left + 1;
    }

    if (flag != IMAGE_SEQ_PARALLEL)
    {
        for(i = 0;i < height;i++)
    	{  
    		for(j = 0;j < width;j++)
    		{
    		    if (channel == 3)
    		    {
        		    for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
        		    {
        		        dst = &CV_IMAGE_ELEM( pIamge, unit16, i, j*channel+color );
        		        *dst = (unit16)img->data[((i+top)*Imagewidth+(j+left))*img->channel+color];
        		    }
    		    }
    		    else
    		    {
                    dst = &CV_IMAGE_ELEM( pIamge, unit16, i, j );
                    *dst = (unit16)img->data[(i+top)*Imagewidth+(j+left)];
    		    }
    		}
    	}
    }
    else
    {
        int *imagedata;
        for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
        {
            imagedata = img[color].data;
        	for(i = 0;i < height;i++)
        	{  
        		for(j = 0;j < width;j++)
        		{
        		    dst = &CV_IMAGE_ELEM( pIamge, unit16, i, j*channel+color );
                    *dst = (unit16)imagedata[(i+top)*Imagewidth+(j+left)];
        		}
        	}
        }
    }

    return;
}

void Image16Read(IN char* filename, OUT IMAGE32_S *img)
{
    IplImage* pIamge = cvLoadImage( filename, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    img->channel = IMAGE_SEQ_NORMAL;
    if (NULL == pIamge)
    {
        printf("ImageRead16 error !\n");
    }
    else if (pIamge->depth != 16)
    {
        printf("depth != 16\n");
        cvReleaseImage( &pIamge );
    }
    else
    {
        int size = pIamge->height*pIamge->width*pIamge->nChannels;
    	img->data = MallocType(int, size);

        Image16OpenCv2Hustsong(pIamge, img);

    	cvReleaseImage( &pIamge );
    }

	return;
}

void Image16Read(IN char* filename, OUT IMAGE32_S *img, IN int buffer)
{
    IplImage* pIamge = cvLoadImage( filename, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

    img->channel = IMAGE_SEQ_NORMAL;
    if (NULL == pIamge)
    {
        printf("ImageRead16 error !\n");
    }
    else if (pIamge->depth != 16)
    {
        printf("depth != 16\n");
        cvReleaseImage( &pIamge );
    }
    else
    {
        int size = pIamge->height*pIamge->width*pIamge->nChannels;
        if (buffer < size)
        {
            if (img->data && buffer != 0)       free(img->data);
            img->data = MallocType(int, size);
        }

        Image16OpenCv2Hustsong(pIamge, img);

    	cvReleaseImage( &pIamge );
    }

	return;
}

cv::Mat Image16Mat(IN IMAGE32_S *img, IN RECT_S *rect)
{
	IplImage* pNewImg;
	CvSize size;
	int channel;
	int flag = img->channel;

	if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
	else if (flag > 3)              channel = 3;
	else                            channel = flag;

	if (rect == NULL)
	{
		size.height = img->height;
		size.width = img->width;
	}
	else
	{
		size.height = rect->bottom - rect->top + 1;
		size.width = rect->right - rect->left + 1;
	}

	pNewImg = cvCreateImage(size, IPL_DEPTH_16U, channel);
	Image16Hustsong2OpenCv(img, pNewImg, rect);

	cv::Mat mat(pNewImg, true);
	//cvSaveImage(filename, pNewImg);
	cvReleaseImage(&pNewImg);
	return mat;
}

void Image16Save(IN char* filename, IN IMAGE32_S *img, IN RECT_S *rect)
{
    IplImage* pNewImg;
    CvSize size;
    int channel;
    int flag = img->channel;

    if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
    else if (flag > 3)              channel = 3;
    else                            channel = flag;

    if (rect == NULL)
    {
        size.height = img->height;
        size.width = img->width;
    }
    else
    {
        size.height = rect->bottom - rect->top + 1;
        size.width = rect->right - rect->left + 1;
    }

	pNewImg = cvCreateImage(size,IPL_DEPTH_16U,channel);
    Image16Hustsong2OpenCv(img, pNewImg, rect);

    cvSaveImage(filename, pNewImg);
	cvReleaseImage( &pNewImg );
}

void ImageHustsong2Parallel(IN IMAGE_S *img, OUT IMAGE_S *img2)
{
    int i, j;
    int color;
    int height = img->height;
    int width = img->width;
    int channel = img->channel;
    PIXEL *p, *q;

    for (color = IMAGE_COLOR_BLUE;color < IMAGE_COLOR_MAX;color++)
    {
        img2[color].height = height;
    	img2[color].width = width;
        img2[color].channel = IMAGE_SEQ_PARALLEL;
        p = img2[color].data;
        q = img->data + color;
    	for(i = 0;i < height;i++, p += width)
        {
    		for(j = 0;j < width;j++)
    		{
        	    p[j] = *q;
                q += channel;
    		}
    	}
    }

    return;
}

/*******************************************************************************
    Func Name: ImageRead
 Date Created: 2011-01-25
       Author: zhusong
     Function: 读取图像
        Input: IN char *fileName, 文件名
               IN int buffer, img中的内存大小
       Output: OUT IMAGE_S *img, 读取的图像
       Return: 无
      Caution: 需要OpenCv支持
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
bool MatRead(cv::Mat& mat, OUT IMAGE_S *img)
{
	IplImage pImage = mat;
	IplImage* pIamge = cvCloneImage(&pImage);

	img->width = img->height = 0;
	img->channel = IMAGE_SEQ_NORMAL;
	if (NULL == pIamge)
	{
		return false;
	}
	else
	{
		int size = pIamge->height*pIamge->width*pIamge->nChannels;
		img->data = MallocType(PIXEL, size);

		ImageOpenCv2Hustsong(pIamge, img);

		cvReleaseImage(&pIamge);
	}

	return true;
}

void ImageRead(IN char* filename, OUT IMAGE_S *img)
{
    IplImage* pIamge = cvLoadImage( filename, CV_LOAD_IMAGE_UNCHANGED);

    img->width = img->height = 0;
    img->channel = IMAGE_SEQ_NORMAL;
    if (NULL == pIamge)
    {
        ImageReadFree(filename, img, 0);
    }
    else
    {
        int size = pIamge->height*pIamge->width*pIamge->nChannels;
    	img->data = MallocType(PIXEL, size);

        ImageOpenCv2Hustsong(pIamge, img);

    	cvReleaseImage( &pIamge );
    }

	return;
}

void ImageReadParallel(IN char* filename, OUT IMAGE_S *img)
{
    IplImage* pIamge = cvLoadImage( filename, CV_LOAD_IMAGE_UNCHANGED);

    if (NULL == pIamge)
    {
        printf("ImageReadParallel error !\n");
    }
    else
    {
        int color;
        int channel = pIamge->nChannels;
        int size = pIamge->height*pIamge->width;

        if (channel != 1)   img->channel = IMAGE_SEQ_PARALLEL;
        else                img->channel = channel;
        for (color = 0;color < channel;color++)
        {
#ifdef PLATFORM_SSE
            img[color].data = (__declspec(align(16))PIXEL *)_mm_malloc(size, 16);
#else
            img[color].data = MallocType(PIXEL, size);
#endif
        }
        ImageOpenCv2Hustsong(pIamge, img);

    	cvReleaseImage( &pIamge );
    }

	return;
}

// 自定义通道数
void ImageReadChannel(IN char* filename, OUT IMAGE_S *img, IN int channel)
{
    IplImage* pIamge = cvLoadImage( filename, CV_LOAD_IMAGE_UNCHANGED);

    if (NULL == pIamge)
    {
        printf("ImageReadChannel error !\n");
    }
    else
    {
        int size = pIamge->height*pIamge->width*channel;
    	img->data = MallocType(PIXEL, size);
        img->channel = channel;

        ImageOpenCv2Hustsong(pIamge, img);

    	cvReleaseImage( &pIamge );
    }

	return;
}

// 与ImageRead一样，仅内存申请上有区别，用于批处理
void ImageRead(IN char* filename, INOUT IMAGE_S *img, IN int buffer)
{
    IplImage* pIamge = cvLoadImage( filename, CV_LOAD_IMAGE_UNCHANGED);

    img->channel = IMAGE_SEQ_NORMAL;
    if (NULL == pIamge)
    {
        ImageReadFree(filename, img, buffer);
    }
    else
    {
        int size = pIamge->height*pIamge->width*pIamge->nChannels;
        if (buffer < size)
        {
            if (img->data && buffer != 0)       free(img->data);
            img->data = MallocType(PIXEL, size);
        }

        ImageOpenCv2Hustsong(pIamge, img);

    	cvReleaseImage( &pIamge );
    }

	return;
}

/*******************************************************************************
    Func Name: ImageReadFree
 Date Created: 2011-01-25
       Author: zhusong
     Function: FreeImage库进行读取
        Input: IN char *fileName, 文件名
               IN int buffer, img中的内存大小
       Output: OUT IMAGE_S *img, 读取的图像
       Return: 无
      Caution: 
      Description: 支持OpenCv不支持的一些图像格式，如tif
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageReadFree(IN char* filename, IN IMAGE_S *img, IN int buffer)
{
	return;
}

/*******************************************************************************
    Func Name: ImageSave
 Date Created: 2011-01-25
       Author: zhusong
     Function: 保存图像
        Input: IN char *fileName, 文件名
               IN IMAGE_S *img, 待保存的图像
               IN RECT_S *rect, 待保存的图像区域
       Output: 
       Return: 无
      Caution: 需要OpenCv支持
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageSave(IN char* filename, IN IMAGE_S *img, IN RECT_S *rect)
{
    IplImage* pNewImg;
    CvSize size;
    int channel;
    int flag = img->channel;

    if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
    else if (flag > 3)              channel = 3;
    else                            channel = flag;

    if (rect == NULL)
    {
        size.height = img->height;
        size.width = img->width;
    }
    else
    {
        size.height = rect->bottom - rect->top + 1;
        size.width = rect->right - rect->left + 1;
    }

	pNewImg = cvCreateImage(size,IPL_DEPTH_8U,channel);
    ImageHustsong2OpenCv(img, pNewImg, rect);

    cvSaveImage(filename, pNewImg);
	cvReleaseImage( &pNewImg );
}

cv::Mat Image2Mat(IN IMAGE_S *img, IN RECT_S *rect)
{
	IplImage* pNewImg;
	CvSize size;
	int channel;
	int flag = img->channel;

	if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
	else if (flag > 3)              channel = 3;
	else                            channel = flag;

	if (rect == NULL)
	{
		size.height = img->height;
		size.width = img->width;
	}
	else
	{
		size.height = rect->bottom - rect->top + 1;
		size.width = rect->right - rect->left + 1;
	}

	pNewImg = cvCreateImage(size, IPL_DEPTH_8U, channel);
	ImageHustsong2OpenCv(img, pNewImg, rect);

	cv::Mat mat(pNewImg, true);
	//cvShowImage(filename, pNewImg);
	//cvWaitKey(1);
	cvReleaseImage(&pNewImg);
	return mat;
}

void ImageShow(IN char* filename, IN IMAGE_S *img, IN RECT_S *rect)
{
    IplImage* pNewImg;
    CvSize size;
    int channel;
    int flag = img->channel;

    if (flag == IMAGE_SEQ_PARALLEL) channel = 3;
    else if (flag > 3)              channel = 3;
    else                            channel = flag;

    if (rect == NULL)
    {
        size.height = img->height;
        size.width = img->width;
    }
    else
    {
        size.height = rect->bottom - rect->top + 1;
        size.width = rect->right - rect->left + 1;
    }

	pNewImg = cvCreateImage(size,IPL_DEPTH_8U,channel);
    ImageHustsong2OpenCv(img, pNewImg, rect);

    cvShowImage(filename, pNewImg);
	cvWaitKey(1);
	cvReleaseImage( &pNewImg );
}



/*******************************************************************************
    Func Name: ImageReadCcs
 Date Created: 2011-01-25
       Author: zhusong
     Function: 读取图像为dsp的ccs数据格式
        Input: IN char *fileName, 文件名
       Output: OUT IMAGE_S *img, 读取的图像
       Return: 无
      Caution: 只支持八位灰度图
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageReadCcs(IN char* filename, OUT IMAGE_S *img)
{
    int x,y,l;
    int width = img->width;
    int height = img->height;
    PIXEL *image = img->data;

    CcsData *data;
	int length;

    if (width % 4 != 0)
    {
        printf("image width is not 4*n !\n");
        return;
    }

	data = ReadCcsData(filename,&length);
	if (length != width*height)
	{
        printf("length error !\n");
        return;
    }

    l = 0;
    for (y = 0;y < height;y++)
    {
		for (x = 0;x < width;x+=4,l++)
        {
            image[y*width+(x+0)] = data[l].pixel[0];
            image[y*width+(x+1)] = data[l].pixel[1];
            image[y*width+(x+2)] = data[l].pixel[2];
            image[y*width+(x+3)] = data[l].pixel[3];
        }
    }

    free(data);

	return;

}

/*******************************************************************************
    Func Name: ImageSaveCcs
 Date Created: 2011-01-25
       Author: zhusong
     Function: 保存图像为dsp的ccs数据格式
        Input: IN char *fileName, 文件名
               IN IMAGE_S *img, 待保存的图像
       Output: 
       Return: 无
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageSaveCcs(IN char* filename, IN IMAGE_S *img)
{
    int x,y;
    int width = img->width;
    int height = img->height;
    PIXEL *image = img->data;
	CcsData data;

    FILE *pf = NULL;
	char szBuffer[4096];
    char *p = szBuffer;

    if (width % 4 != 0)
    {
        printf("image width is not 4*n\n");
        return;
    }

    pf = fopen(filename, "w");
    if (NULL == pf)     return;

    // 第一个行信息头
    // MagicNumber  Format  StartingAddress PageNum Length 
    
    // MagicNumber      固定为1651
    // Format           指示文件数据的格式。1:十六进制 2:整型 3:长整型 4:浮点型。  
    // StartingAddress  存储块的起始地址。  
    // PageNum          存储块的页地址。  
    // Length           数据长度
    p += sprintf(p, "1651 1 80000000 0 %x\n",(width*height)/4);
    fputs(szBuffer,pf);

    for (y = 0;y < height;y++)
    {
        p = szBuffer;
		for (x = 0;x < width;x+=4)
        {
            data.pixel[0] = image[y*width+(x+0)];
            data.pixel[1] = image[y*width+(x+1)];
            data.pixel[2] = image[y*width+(x+2)];
            data.pixel[3] = image[y*width+(x+3)];

			p += sprintf(p, "%s", "0x");
            for (int i = 0;i < 4;i++)
            {
                if (data.pixel[3-i] == 0x00)
                {
                    p += sprintf(p, "%s", "00");
                }
                else if (data.pixel[3-i] <= 0x0F)
                {
                    p += sprintf(p, "%s", "0");
                    break;
                }
                else
                {
                    break;
                }
            }
            if (data.item != 0x00)
                p += sprintf(p, "%X", data.item);

            p += sprintf(p, "\n");
        }
        fputs(szBuffer,pf);
    }

    fclose(pf);

	return;

}

/*******************************************************************************
    Func Name: ImageSaveQuartus
 Date Created: 2011-01-25
       Author: zhusong
     Function: 保存图像为dsp的ccs数据格式
        Input: IN char *fileName, 文件名
               IN IMAGE_S img, 待保存的图像
       Output: 
       Return: 无
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageSaveQuartus(IN char* filename, IN IMAGE_S *img)
{
    int x,y;
    int width = img->width;
    int height = img->height;
    int channel = img->channel;
    PIXEL *image = img->data;
    ushort r,g,b,data16;

    FILE *pf = NULL;
	char szBuffer[4096];
    char *p = szBuffer;

    if (channel != 3)   return;

    pf = fopen(filename, "wb");
    if (NULL == pf)     return;

    // 第一个行信息头
	p += sprintf(p, "Depth = %i;\r\n",width*height);
    p += sprintf(p, "Width = 16;\r\n");
	p += sprintf(p, "Address_radix=DEC;\r\n");
	p += sprintf(p, "Data_radix=HEX;\r\n");
	p += sprintf(p, "Content\r\n");
	p += sprintf(p, "BEGIN\r\n");
	p += sprintf(p, "\t[0..%i] : 0000;\r\n", width*height - 1);
    fputs(szBuffer,pf);

    for (y = 0;y < height;y++)
    {
        p = szBuffer;
		for (x = 0;x < width;x++)
        {
    		r = (ushort)image[(y*width+x)*channel+IMAGE_COLOR_RED];
    		g = (ushort)image[(y*width+x)*channel+IMAGE_COLOR_GREEN];
    		b = (ushort)image[(y*width+x)*channel+IMAGE_COLOR_BLUE];

            r = r & 0xF8;   // 5
            g = g & 0xFC;   // 6
            b = b & 0xF8;   // 5

            r = r << 8;     r = r & 0xF800;
            g = g << 3;     g = g & 0x07E0;
            b = b >> 3;     b = b & 0x001F;

            data16 = 0;
            data16 = data16 | r;
            data16 = data16 | g;
            data16 = data16 | b;

            p += sprintf(p, "\t%i :", y*width+x);
            p += sprintf(p, " %x;\r\n", data16);
        }
        fputs(szBuffer,pf);
    }

    p = szBuffer;
    sprintf(p, "End;\r\n");
    fputs(szBuffer,pf);

    fclose(pf);

	return;

}

/*******************************************************************************
    Func Name: ImageInt2Pixel
 Date Created: 2011-02-06
       Author: zhusong
     Function: 从图像中截取矩形区域
        Input: IN IMAGE32_S *srcImage, 源图像
               IN RECT_S *rect, 处理区域
       Output: OUT IMAGE_S *dstImage, 截取的图像
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageInt2Pixel(IN IMAGE32_S *srcImage,
                    OUT IMAGE_S *dstImage,
                    IN RECT_S *rect)
{
    int i, j, color;
    int srcx, srcy;
    int *src = srcImage->data;
    PIXEL *dst = dstImage->data;
    int width = srcImage->width;
    int height = srcImage->height;
    int channel = srcImage->channel;
    int dstwidth;
    int dstheight;
    int left,top;
    int temp;

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

    dstImage->width = dstwidth;
    dstImage->height = dstheight;
    dstImage->channel = channel;

    for(i = 0,srcy = top;i < dstheight;i++,srcy++)
	{  
		for(j = 0,srcx = left;j < dstwidth;j++,srcx++)
		{
		    if (channel == 3)
		    {
    		    for (color = 0;color < channel;color++)
    		    {
    		        temp = min(IMAGE_WHITE, src[(srcy*width+srcx)*channel+color]);
        			dst[(i*dstwidth+j)*channel+color] = (PIXEL)temp;
    		    }
		    }
		    else
		    {
		        temp = min(IMAGE_WHITE, src[(srcy*width+srcx)*channel+color]);
        	    dst[i*dstwidth+j] = (PIXEL)temp;
		    }
		}
	}

    return;
}

/*******************************************************************************
    Func Name: ImageFree
 Date Created: 2011-01-25
       Author: zhusong
     Function: 释放图像
        Input: INOUT IMAGE_S img, 待释放的图像               
       Output: 
       Return: 无
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageFree(INOUT IMAGE_S *img)
{
    int flag = img->channel;
    if (flag == IMAGE_SEQ_PARALLEL)
    {
        int i;
        for (i = 0;i < 3;i++)
        {
#ifdef PLATFORM_SSE
            _mm_free(img[i].data);
#else
            free(img[i].data);
#endif
        }
    }
    else if (img->data != NULL)
    {
        free(img->data);
    }

    img->data = NULL;
    img->width = 0;
    img->height = 0;
    img->channel = 0;
}

/*******************************************************************************
    Func Name: ImageRaw16ReadCcs
 Date Created: 2011-01-25
       Author: zhusong
     Function: 读取RAW格式图像
        Input: IN char *fileName, 文件名
               IN int bits, 像素bit数(支持12位彩色CCD)
       Output: OUT IMAGE32_S *img, 读取的彩色图像
       Return: 无
      Caution: 
      Description: dsp的ccs数据格式
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void ImageRawReadCcs(IN char* filename, OUT IMAGE_S *img)
{
    int x,y,l;
    int width = img->width;
    int height = img->height;
    int channel = img->channel;
    PIXEL *image = img->data;

    CcsData *data;
	int length;

    if (channel != 3)
    {
        printf("image channel is not 3 !\n");
        return;
    }
    
    if (width % 4 != 0)
    {
        printf("image width is not 4*n !\n");
        return;
    }

	data = ReadCcsData(filename,&length);
	if (length != width*height)
	{
        printf("length error !\n");
        return;
    }

    l = 0;
    for (y = 0;y < height;)
    {
		for (x = 0;x < width;x += 4,l++)
        {
            // R G R G R ...
            image[(y*width+x)*channel+IMAGE_COLOR_RED] = data[l].pixel[0];
            image[(y*width+x+1)*channel+IMAGE_COLOR_GREEN] = data[l].pixel[1];
            image[(y*width+x+2)*channel+IMAGE_COLOR_RED] = data[l].pixel[2];
            image[(y*width+x+3)*channel+IMAGE_COLOR_GREEN] = data[l].pixel[3];
        }
        y++;

        for (x = 0;x < width;x += 4,l++)
        {
            // G B G B G ...
            image[(y*width+x)*channel+IMAGE_COLOR_GREEN] = data[l].pixel[0];
            image[(y*width+x+1)*channel+IMAGE_COLOR_BLUE] = data[l].pixel[1];
            image[(y*width+x+2)*channel+IMAGE_COLOR_GREEN] = data[l].pixel[2];
            image[(y*width+x+3)*channel+IMAGE_COLOR_BLUE] = data[l].pixel[3];
        }
        y++;
    }

    free(data);

	return;
}

void ImageRaw16ReadCcs(IN char* filename, OUT IMAGE_S *img, IN int bits)
{
    int x,y,l;
    int width = img->width;
    int height = img->height;
    int channel = img->channel;
    PIXEL *image = img->data;
    CcsData *data;
	int length;
	int temp;
    int mask = (1 << (bits+1))-1;
    double r,g,b;
    int nr,ng,nb;
    double grey = 128;

    if (channel != 3)
    {
        printf("image channel is not 3 !\n");
        return;
    }

    if (width % 4 != 0)
    {
        printf("image width is not 4*n !\n");
        return;
    }

	data = ReadCcsData(filename,&length);
	if (length != 2*width*height)
	{
        printf("length error !\n");
        return;
    }

    l = 0; nr = 0;
    for (y = 0;y < height;y++)
    {
		for (x = 0;x < width;x += 2,l++)
		{
            if ((data[l].pixel16[0] & 0xf000) != 0)   nr++;
            if ((data[l].pixel16[1] & 0xf000) != 0)   nr++;
		}
    }
    printf("%d\n",nr);  // 153705  640*480 = 307200


#if 1
    // 白平衡

    // 求rgb均值
    l = 0;
    r = g = b = 0;
    nr= ng = nb = 0;
    for (y = 0;y < height;)
    {
		for (x = 0;x < width;x += 2,l++)
        {
            // R G R G R ...
            temp = data[l].pixel16[0] & mask;
            r += temp;  nr++;

            temp = data[l].pixel16[1] & mask;
            g += temp;  ng++;
        }
        y++;

        for (x = 0;x < width;x += 2,l++)
        {
            // G B G B G ...
            temp = data[l].pixel16[0] & mask;
            g += temp;  ng++;

            temp = data[l].pixel16[1] & mask;
            b += temp;  nb++;
        }
        y++;
    }

    r /= nr;    g /= ng;    b /= nb;
    r = grey/r; g = grey/g; b = grey/b; 

    l = 0;
    for (y = 0;y < height;)
    {
		for (x = 0;x < width;x += 2,l++)
        {
            // R G R G R ...
            temp = data[l].pixel16[0];
            image[(y*width+x)*channel+IMAGE_COLOR_RED] = (PIXEL)round(temp*r);

            temp = data[l].pixel16[1];
            image[(y*width+x+1)*channel+IMAGE_COLOR_GREEN] = (PIXEL)round(temp*g);
        }
        y++;

        for (x = 0;x < width;x += 2,l++)
        {
            // G B G B G ...
            temp = data[l].pixel16[0];
            image[(y*width+x)*channel+IMAGE_COLOR_GREEN] = (PIXEL)round(temp*g);

            temp = data[l].pixel16[1];
            image[(y*width+x+1)*channel+IMAGE_COLOR_BLUE] = (PIXEL)round(temp*b);
        }
        y++;
    }

#else

    l = 0;
    for (y = 0;y < height;)
    {
		for (x = 0;x < width;x += 2,l++)
        {
            // R G R G R ...
            temp = data[l].pixel16[0] & mask;
            image[(y*width+x)*channel+IMAGE_COLOR_RED] = (PIXEL)(temp >> 4);

            temp = data[l].pixel16[1] & mask;
            image[(y*width+x+1)*channel+IMAGE_COLOR_GREEN] = (PIXEL)(temp >> 4);
        }
        y++;

        for (x = 0;x < width;x += 2,l++)
        {
            // G B G B G ...
            temp = data[l].pixel16[0] & mask;
            image[(y*width+x)*channel+IMAGE_COLOR_GREEN] = (PIXEL)(temp >> 4);

            temp = data[l].pixel16[1] & mask;
            image[(y*width+x+1)*channel+IMAGE_COLOR_BLUE] = (PIXEL)(temp >> 4);
        }
        y++;
    }
#endif


    free(data);

	return;
}

