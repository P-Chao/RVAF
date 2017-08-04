/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: StatFuncDef.h    			                                     */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 数据统计功能template实现，兼容各种数据类型                   */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"

#ifndef STAT_SUFFIX
#error "STAT_SUFFIX must be defined"
#endif

#ifndef STAT_TYPE
#error "STAT_TYPE must be defined"
#endif

/*******************************************************************************
    Func Name: StatMin(StatMax)
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求最小(大)值
        Input: IN STAT_TYPE a[], 待统计的数组
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: 
       Return: STAT_TYPE, 最小(大)值 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatMin

#define _StatMin    DefStrcat2(StatMin,STAT_SUFFIX)
STAT_TYPE _StatMin(IN STAT_TYPE a[], IN int n, IN int s)
{
    int i;
    STAT_TYPE mina = a[0];
    
    for (i = 1;i < n;i++)
    {
        if (a[i*s] < mina)
        {
            mina = a[i*s];
        }
    }

    return mina;
}

#endif


#ifndef _StatMax

#define _StatMax    DefStrcat2(StatMax,STAT_SUFFIX)
STAT_TYPE _StatMax(IN STAT_TYPE a[], IN int n, IN int s)
{
    int i;
    STAT_TYPE maxa = a[0];
    
    for (i = 1;i < n;i++)
    {
        if (a[i*s] > maxa)
        {
            maxa = a[i*s];
        }
    }

    return maxa;
}

#endif


#ifndef _StatMinMax

#define _StatMinMax DefStrcat2(StatMinMax,STAT_SUFFIX)
void _StatMinMax(IN STAT_TYPE a[], IN int n, IN int s, OUT STAT_TYPE *mina, OUT STAT_TYPE *maxa)
{
    int i;
    int start;

    if (n%2 == 1)
    {
        *maxa = *mina = a[0];
        start = 1;
    }
    else
    {
        if (a[0] > a[s])
        {
            *maxa = a[0];
            *mina = a[s];
        }
        else
        {
            *maxa = a[s];
            *mina = a[0];
        }
        start = 2;
    }

    for (i = start;i < n;i+=2)
    {
        if (a[i*s] > a[(i+1)*s])
        {
            if (a[i*s] > *maxa)         *maxa = a[i*s];
            if (a[(i+1)*s] < *mina)     *mina = a[(i+1)*s];
        }
        else
        {
            if (a[(i+1)*s] > *maxa)     *maxa = a[(i+1)*s];
            if (a[i*s] < *mina)         *mina = a[i*s];
        }
    }
}

#endif

/*******************************************************************************
    Func Name: _StatMin12
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求最小(大)值及次小(大)值
        Input: IN STAT_TYPE a[], 待统计的数组
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: OUT STAT_TYPE *mina1, 最小(大)值
               OUT STAT_TYPE *mina2, 次小(大)值
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatMin12

#define _StatMin12 DefStrcat2(StatMin12,STAT_SUFFIX)
void _StatMin12(IN STAT_TYPE a[], IN int n, IN int s, OUT STAT_TYPE *mina1, OUT STAT_TYPE *mina2)
{
    int i;
    STAT_TYPE a1, a2, temp;

    a1 = a[0];
    a2 = a[s];
    if (a1 > a2)
    {
        swapAB(a1, a2, temp);
    }

    for (i = 2;i < n;i++)
    {
        temp = a[i*s];
        if (temp < a1)
        {
            a2 = a1;
            a1 = temp;
        }
        else if(temp < a2)
        {
            a2 = temp;
        }
    }

    *mina1 = a1;
    *mina2 = a2;
}

#endif

#ifndef _StatMax12

#define _StatMax12 DefStrcat2(StatMax12,STAT_SUFFIX)
void _StatMax12(IN STAT_TYPE a[], IN int n, IN int s, OUT STAT_TYPE *maxa1, OUT STAT_TYPE *maxa2)
{
    int i;
    STAT_TYPE a1, a2, temp;

    a1 = a[0];
    a2 = a[s];
    if (a1 < a2)
    {
        swapAB(a1, a2, temp);
    }

    for (i = 2;i < n;i++)
    {
        temp = a[i*s];
        if (temp > a1)
        {
            a2 = a1;
            a1 = temp;
        }
        else if(temp > a2)
        {
            a2 = temp;
        }
    }

    *maxa1 = a1;
    *maxa2 = a2;
}

#endif

/*******************************************************************************
    Func Name: StatMinIndex(StatMaxIndex)
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求最小(大)值及其索引
        Input: IN STAT_TYPE a[], 待统计的数组
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: OUT STAT_TYPE *val, 最小(大)值
               OUT int *index, 索引值
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatMinIndex

#define _StatMinIndex       DefStrcat2(StatMinIndex,STAT_SUFFIX)
void _StatMinIndex(IN STAT_TYPE a[], IN int n, IN int s, OUT STAT_TYPE *val, OUT int *index)
{
    int i;
    int ind = 0;
    STAT_TYPE mina = a[0];
    
    for (i = 1;i < n;i++)
    {
        if (a[i*s] < mina)
        {
            mina = a[i*s];
            ind = i;
        }
    }

    *val = mina;
    *index = ind;

    return;
}

#endif


#ifndef _StatMaxIndex

#define _StatMaxIndex       DefStrcat2(StatMaxIndex,STAT_SUFFIX)
void _StatMaxIndex(IN STAT_TYPE a[], IN int n, IN int s, OUT STAT_TYPE *val, OUT int *index)
{
    int i;
    int ind = 0;
    STAT_TYPE maxa = a[0];
    
    for (i = 1;i < n;i++)
    {
        if (a[i*s] > maxa)
        {
            maxa = a[i*s];
            ind = i;
        }
    }

    *val = maxa;
    *index = ind;

    return;
}

#endif

/*******************************************************************************
    Func Name: _StatMinIndex12(StatMaxIndex12)
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求最小(大)值及次小(大)值和他们的索引
        Input: IN STAT_TYPE a[], 待统计的数组
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: OUT STAT_TYPE *val, 最小(大)值
               OUT int *index, 索引值
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatMinIndex12

#define _StatMinIndex12 DefStrcat2(StatMinIndex12,STAT_SUFFIX)
void _StatMinIndex12(IN STAT_TYPE a[], 
                     IN int n, 
                     IN int s, 
                     OUT STAT_TYPE *mina1, 
                     OUT STAT_TYPE *mina2,
                     OUT int *index1,
                     OUT int *index2)
{
    int i;
    STAT_TYPE a1, a2, temp;
    int i1, i2;

    a1 = a[0];
    a2 = a[s];
    if (a1 > a2)
    {
        i1 = 1;
        i2 = 0;
        swapAB(a1, a2, temp);
    }
    else
    {
        i1 = 0;
        i2 = 1;
    }

    for (i = 2;i < n;i++)
    {
        temp = a[i*s];
        if (temp < a1)
        {
            a2 = a1;
            i2 = i1;
            a1 = temp;
            i1 = i;
        }
        else if(temp < a2)
        {
            a2 = temp;
            i2 = i;
        }
    }

    *mina1 = a1;
    *mina2 = a2;
    *index1 = i1;
    *index2 = i2;
}

#endif

#ifndef _StatMaxIndex12

#define _StatMaxIndex12 DefStrcat2(StatMaxIndex12,STAT_SUFFIX)
void _StatMaxIndex12(IN STAT_TYPE a[], 
                     IN int n, 
                     IN int s, 
                     OUT STAT_TYPE *maxa1, 
                     OUT STAT_TYPE *maxa2,
                     OUT int *index1,
                     OUT int *index2)
{
    int i;
    STAT_TYPE a1, a2, temp;
    int i1, i2;

    a1 = a[0];
    a2 = a[s];
    if (a1 < a2)
    {
        i1 = 1;
        i2 = 0;
        swapAB(a1, a2, temp);
    }
    else
    {
        i1 = 0;
        i2 = 1;
    }

    for (i = 2;i < n;i++)
    {
        temp = a[i*s];
        if (temp > a1)
        {
            a2 = a1;
            i2 = i1;
            a1 = temp;
            i1 = i;
        }
        else if(temp > a2)
        {
            a2 = temp;
            i2 = i;
        }
    }

    *maxa1 = a1;
    *maxa2 = a2;
    *index1 = i1;
    *index2 = i2;
}

#endif

/*******************************************************************************
    Func Name: StatMinPos(StatMaxPos)
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求索引内的最小(大)值
        Input: IN STAT_TYPE a[], 待统计的数组
               IN int *pos, 索引序列
               IN int n, 数据个数
       Output: OUT STAT_TYPE *val, 最小(大)值
               OUT int *index, 索引序列的索引值
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatMinPos

#define _StatMinPos         DefStrcat2(StatMinPos,STAT_SUFFIX)
void _StatMinPos(IN STAT_TYPE a[], IN int *pos, IN int n, OUT STAT_TYPE *val, OUT int *index)
{
    int i;
    int ind = 0;
    int p = pos[0];
    STAT_TYPE mina = a[p];
    STAT_TYPE temp;

    for (i = 1;i < n;i++)
    {
        p = pos[i];
        temp = a[p];
        if (temp < mina)
        {
            mina = temp;
            ind = i;
        }
    }

    *val = mina;
    *index = ind;

    return;
}

#endif

#ifndef _StatMaxPos

#define _StatMaxPos         DefStrcat2(StatMaxPos,STAT_SUFFIX)
void _StatMaxPos(IN STAT_TYPE a[], IN int *pos, IN int n, OUT STAT_TYPE *val, OUT int *index)
{
    int i;
    int ind = 0;
    int p = pos[0];
    STAT_TYPE maxa = a[p];
    STAT_TYPE temp;

    for (i = 1;i < n;i++)
    {
        p = pos[i];
        temp = a[p];
        if (temp > maxa)
        {
            maxa = temp;
            ind = i;
        }
    }

    *val = maxa;
    *index = ind;

    return;
}

#endif

/*******************************************************************************
    Func Name: StatLocalMin(StatLocalMax)
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求极小(大)值
        Input: IN STAT_TYPE a[], 待统计的数组
               IN int n, 数据个数
               IN int s, 采样间隔
               OUT int *pos, 极小(大)值位置
               OUT int *num, 极小(大)值数量
       Output: 
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatLocalMin

#define _StatLocalMin       DefStrcat2(StatLocalMin,STAT_SUFFIX)
void _StatLocalMin(IN STAT_TYPE a[], IN int n, IN int s, OUT int *pos, OUT int *num)
{
    int i,j;
    STAT_TYPE cur;
    STAT_TYPE pre;
    STAT_TYPE *p = a;
    int end = n-1;
 
    // 寻找信号的突变点
    cur = *p;
    for (i = 0, j = 0;i < end;i++)
    {
        pre = cur;
        p += s;
        cur = *p;
        // if (cur > pre)           // 相等的一段，记录后面的，同时跳过鞍点
        if (cur >= pre)             // 相等的一段，记录前面的，同时记录鞍点
        {
            // pre为波谷
            pos[j++] = i;

            // 将pre移动到波峰
            while (i < end)
            {
                pre = cur;
                p += s;
                i++;
                cur = *p;
                if (cur < pre)  break;
            }
        }
    }

    // 最后一个数据点
    if (a[end*s] < a[(end-1)*s])
    {
        pos[j++] = end;
    }
    *num = j;

    return;
}

#endif

#ifndef _StatLocalMax

#define _StatLocalMax       DefStrcat2(StatLocalMax,STAT_SUFFIX)
void _StatLocalMax(IN STAT_TYPE a[], IN int n, IN int s, OUT int *pos, OUT int *num)
{
    int i,j;
    STAT_TYPE cur;
    STAT_TYPE pre;
    STAT_TYPE *p = a;
    int end = n-1;
 
    // 寻找信号的突变点
    cur = *p;
    for (i = 0, j = 0;i < end;i++)
    {
        pre = cur;
        p += s;
        cur = *p;
        if (cur < pre)
        {
            // pre为波峰
            pos[j++] = i;

            // 将pre移动到波谷
            while (i < end)
            {
                pre = cur;
                p += s;
                i++;
                cur = *p;
                if (cur > pre)  break;
            }
        }
    }

    // 最后一个数据点
    if (a[end*s] > a[(end-1)*s])
    {
        pos[j++] = end;
    }
    *num = j;

    return;
}

#endif

#ifndef _StatLocalMaxTh

#define _StatLocalMaxTh         DefStrcat2(StatLocalMaxTh,STAT_SUFFIX)
void _StatLocalMaxTh(IN STAT_TYPE a[], IN int n, IN int s, IN STAT_TYPE thre, OUT int *pos, OUT int *num)
{
    int i,j;
    STAT_TYPE cur;
    STAT_TYPE pre;
    STAT_TYPE *p = a;
    int end = n-1;
 
    // 寻找信号的突变点
    cur = *p;
    for (i = 0, j = 0;i < end;i++)
    {
        pre = cur;
        p += s;
        cur = *p;
        if (cur < pre && pre >= thre)
        {
            // pre为波峰
            pos[j++] = i;

            // 将pre移动到波谷
            while (i < end)
            {
                pre = cur;
                p += s;
                i++;
                cur = *p;
                if (cur > pre)  break;
            }
        }
    }

    // 最后一个数据点
    cur = a[end*s];
    pre =  a[(end-1)*s];
    if (cur > pre && cur >= thre)
    {
        pos[j++] = end;
    }
    *num = j;

    return;
}

#endif

#ifndef _StatMaxDisc

#define _StatMaxDisc        DefStrcat2(StatMaxDisc,STAT_SUFFIX)
void _StatMaxDisc(IN STAT_TYPE a[], IN int n, IN int s, OUT int *pos, OUT int *num)
{
    int i,j;
    STAT_TYPE cur;
    STAT_TYPE pre;
    STAT_TYPE *p = a+s;
    int end = n-1;

    // 第一个数据点
    pos[0] = 0;
 
    // 寻找信号的突变点
    cur = *p;
    for (i = 1, j = 1;i < end;i++)
    {
        pre = cur;
        p += s;
        cur = *p;
        if (cur < pre)
        {
            // pre为波峰
            pos[j++] = i;

            // 将pre移动到波谷
            while (i < end)
            {
                pre = cur;
                p += s;
                i++;
                cur = *p;
                if (cur > pre)  break;
            }
        }
    }

    // 最后一个数据点
    pos[j++] = end;
    *num = j;

    return;
}

#endif
/*******************************************************************************
    Func Name: StatSum
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求和
        Input: IN int a[], 待统计的数组
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: 
       Return: int, 和
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatSum

#define _StatSum    DefStrcat2(StatSum,STAT_SUFFIX)
STAT_TYPE _StatSum(IN STAT_TYPE a[], IN int n, IN int s)
{
    int i;
    STAT_TYPE sum = a[0];

    for (i = 1;i < n;i++)
    {
        sum += a[i*s];
    }

    return sum;
}

#endif

/*******************************************************************************
    Func Name: StatAvg
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求平均值
        Input: IN double a[], 待统计的double型数组
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: 
       Return: double, 平均值
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatAvg

#define _StatAvg    DefStrcat2(StatAvg,STAT_SUFFIX)
double _StatAvg(IN STAT_TYPE a[], IN int n, IN int s)
{
    return (double)_StatSum(a, n, s)/(double)n;
}

#endif

/*******************************************************************************
    Func Name: StatMoment
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求中心矩
        Input: IN double a[], 待统计的double型数组
               IN int n, 数据个数
               IN int s, 采样间隔
               IN int m, 矩的阶
       Output: 
       Return: double, 中心矩
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatMoment

#define _StatMoment     DefStrcat2(StatMoment,STAT_SUFFIX)
double _StatMoment(IN STAT_TYPE a[], IN int n, IN int s, IN int m)
{
    int i,j;
    double temp1,temp2;
    double avg = _StatAvg(a, n, s);
    double moment = 0;
    
    for(i = 0;i < n;i++)
    {
        temp1 = a[i*s]-avg;
        temp2 = temp1;
        for(j = 1;j < m;j++)
        {
            temp2 *= temp1;
        }
        moment += temp2;
    }

    return moment;
}

#endif

/*******************************************************************************
    Func Name: StatVar
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求方差
        Input: IN double a[], 待统计的double型数组
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: 
       Return: double, 方差
      Caution: 
      Description: 给出除以n-1和n两种方式
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatVar

#define _StatVar        DefStrcat2(StatVar,STAT_SUFFIX)
#define _StatVarn       DefStrcat2(StatVarn,STAT_SUFFIX)
double _StatVar(IN STAT_TYPE a[], IN int n, IN int s)
{
    return _StatMoment(a, n, s, 2)/(n-1);
}
double _StatVarn(IN STAT_TYPE a[], IN int n, IN int s)
{
    return _StatMoment(a, n, s, 2)/n;
}

#endif

/*******************************************************************************
    Func Name: StatCovMoment
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求协中心矩
        Input: IN double a[], 待统计的数组a
               IN double b[], 待统计的数组b
               IN int n, 数据个数
               IN int s, 采样间隔
               IN int ma, a变量的阶
               IN int mb, b变量的阶
       Output: 
       Return: double, 协中心矩
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatCovMoment

#define _StatCovMoment      DefStrcat2(StatCovMoment,STAT_SUFFIX)
double _StatCovMoment(IN STAT_TYPE a[], 
                      IN STAT_TYPE b[], 
                      IN int n,
                      IN int s,
                      IN int ma,
                      IN int mb)
{
    int i,j;
    double temp1,temp2;
    double avga = _StatAvg(a, n, s);
    double avgb = _StatAvg(b, n, s);
    double moment = 0;

    for(i = 0;i < n;i++)
    {
        temp1 = a[i*s]-avga;
        temp2 = temp1;
        for(j = 1;j < ma;j++)
        {
            temp2 *= temp1;
        }
        temp1 = b[i*s]-avgb;
        for(j = 0;j < mb;j++)
        {
            temp2 *= temp1;
        }
        
        moment += temp2;
    }

    return moment;
}
#endif

/*******************************************************************************
    Func Name: StatCov
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求协方差
        Input: IN double a[], 待统计的double型数组a
               IN double b[], 待统计的double型数组b
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: 
       Return: double, 协方差
      Caution: 
      Description: 给出除以n-1和n两种方式
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatCov

#define _StatCov        DefStrcat2(StatCov,STAT_SUFFIX)
#define _StatCovn       DefStrcat2(StatCovn,STAT_SUFFIX)
double _StatCov(IN STAT_TYPE a[], IN STAT_TYPE b[], IN int n, IN int s)
{
    return _StatCovMoment(a, b, n, s, 1, 1)/(n-1);
}
double _StatCovn(IN STAT_TYPE a[], IN STAT_TYPE b[], IN int n, IN int s)
{
    return _StatCovMoment(a, b, n, s, 1, 1)/n;
}

#endif

/*******************************************************************************
    Func Name: StatCorrcoef
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求相关系数
        Input: IN double a[], 待统计的double型数组a
               IN double b[], 待统计的double型数组b
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: 
       Return: double, 相关系数
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatCorrcoef

#define _StatCorrcoef        DefStrcat2(StatCorrcoef,STAT_SUFFIX)
double _StatCorrcoef(IN STAT_TYPE a[], IN STAT_TYPE b[], IN int n, IN int s)
{
    return _StatCovMoment(a, b, n, s, 1, 1)/sqrt(_StatMoment(a, n, 1, 2)*_StatMoment(b, n, 1, 2));
}

#endif

/*******************************************************************************
    Func Name: StatCorrcoef
 Date Created: 2011-01-23
       Author: zhusong
     Function: 求相关系数
        Input: IN double a[], 待统计的double型数组a
               IN double b[], 待统计的double型数组b
               IN int n, 数据个数
               IN int s, 采样间隔
       Output: 
       Return: double, 相关系数
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatEntropy

#define _StatEntropy        DefStrcat2(StatEntropy,STAT_SUFFIX)
double _StatEntropy(IN STAT_TYPE a[], IN int n, IN int s)
{
    int i;
    STAT_TYPE *p = a;
    double val;
    double entropy = 0;

    for (i = 0;i < n;i++)
    {
        val = (double)*p;
        entropy += val*log(val);
        p += s;
    }
    return entropy;
}

#endif

/*******************************************************************************
    Func Name: StatHistMedian
 Date Created: 2011-01-23
       Author: zhusong
     Function: 根据hist求中值
        Input: IN STAT_TYPE h[], 直方图
               IN STAT_TYPE acc[], 累积直方图(临时内存)
               IN int n, 数据个数
       Output: 
       Return: int, 中值
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
#ifndef _StatHistMedian

#define _StatHistMedian        DefStrcat2(StatHistMedian, STAT_SUFFIX)
int _StatHistMedian(IN STAT_TYPE h[], IN STAT_TYPE acc[], OUT STAT_TYPE *med, IN int n)
{
    int i;
    STAT_TYPE sum;
    STAT_TYPE median;

    // 求累积直方图
    sum = acc[0] = h[0];
	for (i = 1;i < n;i++)
	{
	    sum += h[i];
		acc[i] = sum;
	}
    median = sum/2;
    if (med != NULL)    *med = median;

    // 求中值
    for (i = 0;i < n;i++)
    {    
        if (acc[i] > median)
        {
            return i;
        }
    }

    return -1;
}

#endif



#undef STAT_SUFFIX
#undef STAT_TYPE

#undef _StatMin
#undef _StatMax
#undef _StatMin12
#undef _StatMax12
#undef _StatMinMax
#undef _StatMinIndex
#undef _StatMaxIndex
#undef _StatMinIndex12
#undef _StatMaxIndex12
#undef _StatLocalMin
#undef _StatLocalMax
#undef _StatLocalMaxTh
#undef _StatSum
#undef _StatAvg
#undef _StatMoment
#undef _StatVar
#undef _StatVarn
#undef _StatCovMoment
#undef _StatCov
#undef _StatCovn
#undef _StatCorrcoef
#undef _StatEntropy
#undef _StatHistMedian
