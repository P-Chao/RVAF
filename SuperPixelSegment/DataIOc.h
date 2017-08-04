/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: DataIOc.h    			                                         */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/25                                                          */
/*                                                                           */
/* Description: 数据读取保存功能                                             */
/*                                                                           */
/* Others: 这一部分主要用于调试，分析，比如matlab画图分析，与实际应用无关。  */
/*         采用纯C实现                                                       */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _DATA_IOC_H_
#define _DATA_IOC_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <stdio.h>
#include <string.h>
#include <malloc.h>

// 数组数据保存方向
#define     SAVE_ARRAY_DATA_HORIZONTAL          1   // 数据水平
#define     SAVE_ARRAY_DATA_VERTICAL            2   // 数据垂直
#define     FILE_NAME_LENGTH                    256 // 文件名长度

// 文件名列表
typedef struct tagFileList
{ 
    char filename[FILE_NAME_LENGTH];
    char name[FILE_NAME_LENGTH];
    struct tagFileList *next;
}FileList;

extern double *ReadMatrixData0(IN char *filename, OUT int *m_Rows, OUT int *m_Cols);
extern void ReadMatrixData1(IN char *filename, OUT double *m_pData, IN int m_NumRows, IN int m_NumCols);
extern double *ReadMatrixData2(IN char *filename, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixData(IN char *filename, IN double *m_pData, IN int m_NumRows, IN int m_NumCols);
extern void ReadMatrixFloat1(IN char *filename, OUT float *m_pData, IN int m_NumRows, IN int m_NumCols);
extern float *ReadMatrixFloat2(IN char *filename, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixFloat(IN char *filename, IN float *m_pData, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixFloat3D(IN char *filename, IN float *m_pData, IN int m_NumRows, IN int m_NumCols, IN int s, IN int d);
extern void SaveMatrixInt64(IN char *filename, IN int64 *m_pData, IN int m_NumRows, IN int m_NumCols);
extern void ReadMatrixInt1(IN char *filename, OUT int *m_pData, IN int m_NumRows, IN int m_NumCols);
extern int *ReadMatrixInt2(IN char *filename, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixInt(IN char *filename, IN int *m_pData, IN int m_NumRows, IN int m_NumCols);
extern unit32 *ReadMatrixUint(IN char *filename, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixUint(IN char *filename, IN unit32 *m_pData, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixUshort(IN char *filename, IN ushort *m_pData, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixChar(IN char *filename, IN char *m_pData, IN int m_NumRows, IN int m_NumCols);
extern void SaveMatrixHex(IN char *filename, IN int *m_pData, IN int m_NumRows, IN int m_NumCols, IN int bits);
extern void SaveMatrixPixel(IN char *filename, IN PIXEL *m_pData, IN int m_NumRows, IN int m_NumCols);
extern BOOL GetMatrixDataSize(IN char *filename, OUT int *m_NumRows, OUT int *m_NumCols);
extern int FileLine(IN char *filename);

extern void ReadBinaryInt(IN char *fileName, IN int *data, IN int size);
extern void SaveBinaryInt(IN char *fileName, IN int *data, IN int size);

extern CcsData *ReadCcsData(IN char *filename, OUT int *length);
extern void SaveCcsData(IN char* filename, IN CcsData *srcdata, IN int size);
extern void Ccs2Pixel(IN CcsData *data, IN int length, OUT PIXEL *image);
extern void Pixel2Ccs(IN PIXEL *image, IN int length, OUT CcsData *data);

extern void ReadBinaryPatternData(IN char *fileName,
                                  OUT Matrix_S *sample,
                                  IN double **label);
extern void SaveBinaryPatternData(IN char *fileName,
                           IN Matrix_S *sample,
                           IN double *label);

extern void ReadMcuResult(IN char *filename,
                          OUT FileList **file,
                          OUT int *nfile,
                          OUT int **number,
                          OUT Matrix_S *point);

extern void FileGetNameFromIndex(IN char *dirName,
                                 OUT char fileName[],
                                 IN char suffix[],
                                 IN int bit,
                                 IN int index);
extern void FileGetNameFromPath(IN char *path, 
                                OUT char *filename, 
                                OUT char *suffix,
                                IN BOOL flag);

extern void FileCopy(IN char* file1, IN char* file2);
extern void FileMerge(IN char* file1, IN char* file2);
extern void FileClear(IN char *filename);
extern BOOL FileExist(IN char *filename);

extern void DirCreate(IN char* dir);
extern BOOL DirCreateRecursion(IN char* dir);

extern void MsgOn();
extern void MsgOff();
extern int MsgShow;
#define MsgPrintf(format, ...)              \
{                                           \
    if (MsgShow)                            \
    {                                       \
        printf(format, ##__VA_ARGS__);      \
    }                                       \
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif

