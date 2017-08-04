/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: Stat.c      			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 数据统计功能的实现                                           */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "Stat.h"

// 定义常用类型的数组 int,double
#define STAT_SUFFIX     Int
#define STAT_TYPE       int
#include "StatFuncDef.h"

#define STAT_SUFFIX     Float
#define STAT_TYPE       float
#include "StatFuncDef.h"

#define STAT_SUFFIX     Double
#define STAT_TYPE       double
#include "StatFuncDef.h"


int StatSumPixel(IN PIXEL a[], IN int n, IN int s)
{
    int i;
    int sum = (int)a[0];

    for (i = 1;i < n;i++)
    {
        sum += a[i*s];
    }

    return sum;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
