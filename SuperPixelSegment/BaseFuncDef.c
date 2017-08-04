/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: BaseFuncDef.c                                                   */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/02/07                                                          */
/*                                                                           */
/* Description: 基本运算函数                                                 */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseFuncDef.h"
#include "math.h"

#ifndef PLATFORM_CCS
#include <malloc.h>
#endif

/*******************************************************************************
    Func Name: squareSum
 Date Created: 2011-02-07
       Author: zhusong
     Function: 连续整数平方和
        Input: IN int n, 整数
       Output: 
       Return: int, 整数平方和
      Caution: 必须满足 n > 0
      Description: 1^2 + 2^2 + ... n^2 = n*(n+1)*(2n+1)/6
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
int squareSum(IN int n)
{
    int n1 = n+1;
    int n2 = n+n1;
    return n*n1*n2/6;
}

/*******************************************************************************
    Func Name: digitInt
 Date Created: 2011-02-07
       Author: zhusong
     Function: 求x的位数
        Input: IN int x, 整数
               IN int n, 进制
       Output: 
       Return: int, 整数次幂
      Caution: 必须满足 x > 0
      Description: 见logCeil
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
int digitInt(IN int x, IN int n)
{
	int p = 0;
	int y = 1;

    if (x == 0) return 1;

	while(y <= x)
	{
		y *= n;
		p++;
	}

	return p;
}

/*******************************************************************************
    Func Name: BitCountU64
 Date Created: 2011-02-07
       Author: zhusong
     Function: 无符整数中1Bit的个数
        Input: IN int x, 正整数
       Output: 
       Return: int, 1Bit的个数
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
int BitCountU64(IN int64 x)
{
    int count = 0;
    int64 one = 1;

    while(x)
    {
        count += (int)(x & one);
        x >>= 1;
    }

    return count;
}
int BitCountU32(IN int x)
{
    int count = 0;
    int one = 1;

    while(x)
    {
        count += (x & one);
        x >>= 1;
    }

    return count;
}

double gibbs(double x, double sigma)
{
    return exp(-x/sigma);
}
double gaussian(double x, double sigma)
{
    return exp(-(x*x)/(2*sigma*sigma));
}

void memsetint(int *a, int n, int x)
{
    int i;

    for (i = 0;i < n;i++)
    {
        a[i] = x;
    }
}

void Tabel1ExpFloat(OUT float *table, IN int length, IN float sigma)
{
    int i;

    // 计算exp table
    if (sigma <= 0)
    {
        for (i = 0;i < length;i++)
        {
            table[i] = 1.0f;
        }
    }
    else
    {
        double s;
        double usc;
        usc = -1.0/sigma;
        for (i = 0, s = 0;i < length;i++, s += usc)
        {
            table[i] = (float)(exp(s));
        }
    }

    return;
}

void Tabel2ExpFloat(OUT float *table, IN int len1, IN int len2, IN float sigma)                         
{
    int i, j;

    // 计算exp table
    if (sigma <= 0)
    {
        int length = len1*len2;
        for (i = 0;i < length;i++)
        {
            table[i] = 1.0f;
        }
    }
    else
    {
        double usc;

        usc = -1.0/(sigma*sigma);
        for (i = 0;i < len1;i++)
        {
            for (j = 0;j < len2;j++)
            {
                table[i*len2+j] = (float)exp((i*i+j*j)*usc);
            }
        }
    }

    return;
}


#ifdef __cplusplus
}
#endif /* end of __cplusplus */
