/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: TimeStat.c    			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 算法时间统计功能                                             */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "TimeStat.h"

clock_t overheadClock;
int64 overheadTime;

/*******************************************************************************
    Func Name: TsClock
 Date Created: 2011-01-23
       Author: zhusong
     Function: clock方法统计代码运行时间框架
        Input: 
       Output: 
       Return: double, 运行时间(单位s，精度ms)
      Caution: 
      Description: 标准C语言函数
                   clock函数返回的是CPU耗费在本程序上的时间
                   如果途中sleep，由于CPU资源被释放，sleep时间将不被统计。
                   返回值实际是耗费在本程序上的CPU时间片的数量
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
double TsClock()
{
    clock_t t = 0;

    TsClockInit();
    
    TsClockStart(t);

    /*
        code...
    */

    TsClockEnd(t);

    return (double)(t)/CLOCKS_PER_SEC;
}

/*******************************************************************************
    Func Name: TsTime
 Date Created: 2011-01-23
       Author: zhusong
     Function: gettimeofday方法统计代码运行时间框架
        Input: 
       Output: 
       Return: double, 运行时间(单位s，精度us)
      Caution: 
      Description: Windows: QueryPerformanceFrequency && QueryPerformanceCounter
                   Linux: gettimeofday
                   直接提取硬件时钟进行运算，精度较高(us)
                   只能计算程序开始时间和结束时间的差值，会受到后台程序的影响。
                   如果后台繁忙，不能完全真实反映被测量函数的运行时间
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
double TsTime()
{
    double t = 0;

    TsTimeInit();
    TsTimeStart(t);

    /*
        code...
    */

    TsTimeEnd(t);
    TsGetTime(t);

    return t;
}



#ifdef __cplusplus
}
#endif /* end of __cplusplus */
