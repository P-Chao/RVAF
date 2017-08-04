/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: DataIO.h    			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/25                                                          */
/*                                                                           */
/* Description: 数据读取保存功能                                             */
/*                                                                           */
/* Others: 这一部分主要用于调试，分析，比如matlab画图分析，与实际应用无关。采*/
/*         用C++实现                                                         */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _DATA_IO_H_
#define _DATA_IO_H_

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <stdio.h>
#include <string.h>
#include <malloc.h>

// #include <iostream.h>             // for template
#include <iostream>
using namespace std;

#include "DataIOc.h"

extern void ReadSvmData(IN char *filename,
                        OUT int *m_Rows,
                        OUT int *m_Cols,
                        OUT double **sample,
                        OUT double **label);

extern void ReadSvmData(IN char *filename,
                 OUT SparseMatrix_S *sample,
                 OUT double **label);

extern void SaveSvmData(IN char *fileName,
                        IN char *saveMode,
                        IN double *sample,
                        IN double *label,
                        IN int d,
                        IN int n);

extern FileList *FileListSearch(IN char *dir, IN char *postfix);
extern void FileListDestroy(OUT FileList **l);
extern int FileListNum(IN char *dir, IN char *postfix);
extern int FileListNum(IN FileList *L);
extern FileList *DirListSearch(IN char *path, IN int flag);

/*******************************************************************************
    Func Name: SaveArrayData
 Date Created: 2011-01-25
       Author: zhusong
     Function: 保存数组数据
        Input: IN char *fileName, 文件名
               IN int direction, 数据保存方向
               IN char *saveMode, 保存模式('a': 添加，'w': 覆盖)
               IN char *dataType, 数据类型("%d","%f","%g","%c" ……)
               IN dataTypeT *data, 待保存的数据数组
               IN int n, 数据个数
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
template < class dataTypeT >
void SaveArrayData(IN char *fileName,
                   IN int direction,
                   IN char *saveMode,
                   IN char *dataType,
                   IN dataTypeT *data,
                   IN int n)
{
	FILE *pf = NULL;
	char szBuffer[4096];
	char *p = szBuffer;
	int i;

	pf = fopen(fileName, saveMode);

    if (NULL == pf)
    {
        return;
    }

	for (i = 0; i < n; i++)
	{	
		p += sprintf(p, dataType, data[i]);

        if (direction == SAVE_ARRAY_DATA_VERTICAL)
		{
            p += sprintf(p, "%c", '\n');
        }
        else if (direction == SAVE_ARRAY_DATA_HORIZONTAL)
        {
            p += sprintf(p, "%c", ' ');
        }

        if (p - szBuffer > 4080)
        {
            fputs(szBuffer,pf);
            p = szBuffer;
        }
	}
	if (direction == SAVE_ARRAY_DATA_HORIZONTAL)
	{
		p += sprintf(p, "%c", '\n');
	}
    fputs(szBuffer,pf);

    fclose(pf);
	
	return;
}

/*******************************************************************************
    Func Name: PrintMatrixData
 Date Created: 2011-01-25
       Author: zhusong
     Function: 打印矩阵数据
        Input: IN dataTypeT *data, 矩阵数据, 行*列
               IN char *dataType, 数据类型("%d","%f","%g","%c" ……)
               IN int m_NumRows, 矩阵的行数
               IN int m_NumCols, 矩阵的列数
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
template < class dataTypeT >
void PrintMatrixData(IN dataTypeT *data,
                     IN char *dataType,
                     IN int m_NumRows, 
                     IN int m_NumCols)
{
	int i,j;

    for (i = 0; i < m_NumRows; i++)
    {
        for (j = 0; j < m_NumCols; j++)
        {
            printf(dataType, data[j+i*m_NumCols]);
            printf(" ");            
        }
        printf("\n"); 
    }
	
	return;
}

/*******************************************************************************
    Func Name: DoubleData
 Date Created: 2011-01-25
       Author: zhusong
     Function: 将数据转换为double类型
        Input: IN dataTypeT *in_data, 输入未知类型的数据
               IN int n, 数据个数
       Output: OUT double *out_data, 输出double类型的数据
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
template < class dataTypeT >
void DoubleData(IN dataTypeT *in_data, OUT double *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = (double)in_data[i];
    }
}

#endif

