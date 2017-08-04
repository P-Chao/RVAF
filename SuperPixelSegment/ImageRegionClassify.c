/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageRegionClassify.c 			                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: ImageRegionFeature 中对应的分类函数                          */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "ImageRegionClassify.h"
//#include "glog\logging.h"
// 模型
#define IRC_DIM     27
int IrcB = -296889;
int IrcW[IRC_DIM] = {

-38748,
67898,
-41409,
-10040,
37301,
69036,
-34655,
-9044,
-39330,
70528,
-46919,
494,
53,
22781,
97247,
1843,
829,
-193,
1508,
13693,
8138,
-25895,
12567,
26613,
-244884,
53011,
16423
};

int IrcClassifyOne(IN int *feature)
{
    int i;
    int wf;
    int sum;

    sum = -IrcB;
    for(i = 0;i < IRC_DIM;i++)
    {
       //wf = _IQ16mpy(IrcW[i],feature[i]);
		
		wf = IrcW[i] * feature[i];
		// 上面写法是正确的，下面写法是错误的，仅仅为了编译能够通过
	//	CHECK_EQ(sum,i) << "See Note, This code need to be modify";
        sum += wf;
	}

    return (sum < 0);
}

void IrcClassifyImage(IN IMAGE32_S *segImage, 
                      IN IMAGE_S *lblImage, 
                      OUT int *map,
                      IN int *feature, 
                      IN int regionnum)
{
    int height = segImage->height;
    int width = segImage->width;
    int *mark = segImage->data;
    PIXEL *lbl = lblImage->data;
    int area = height * width;

    int i, c, k;
    int *fea;

    fea = feature;
    for (i = 0;i < regionnum;i++, fea += IRC_DIM)
    {
        map[i] = IrcClassifyOne(fea);
    }

    lblImage->height = height;
    lblImage->width = width;
    lblImage->channel = 1;
    memset(lbl, 0, area*sizeof(PIXEL));
    for (i = 0;i < area;i++)
    {
        k = mark[i];
        c = map[k];
        // if (c != 0) lbl[i] = IMAGE_WHITE;
        if (c != 0) lbl[i] = 1;
    }

    return;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
