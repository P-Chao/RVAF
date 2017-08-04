/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: VectorBasic.c      			                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 向量操作                                                     */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "VectorBasic.h"
#include "math.h"

// 定义常用类型的数组 int, float, double
#define VECTOR_SUFFIX       Int
#define VECTOR_TYPE         int
#include "VectorFuncDef.h"

#define VECTOR_SUFFIX       Float
#define VECTOR_TYPE         float
#include "VectorFuncDef.h"

#define VECTOR_SUFFIX       Double
#define VECTOR_TYPE         double
#include "VectorFuncDef.h"

/*******************************************************************************
    Func Name: SparseVectorDot
 Date Created: 2011-02-10
       Author: zhusong
     Function: 向量点积
        Input: IN SparseVector_S *A, 输入向量A
               IN SparseVector_S *B 输入向量B
       Output: 无
       Return: double, 点积
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
double SparseVectorDot(IN SparseVector_S *A, IN SparseVector_S *B)
{
    int i = 0;
	int j = 0;
    int idxA,idxB;
    int nA = A->length;
    int nB = B->length;
    double sum = 0;

    while (i < nA && j < nB)
    {
        idxA = A->index[i];
        idxB = B->index[j];

        if (idxA == idxB)
        {
            sum += A->val[i]*B->val[j];
            i++;
            j++;
        }
        else if (idxA > idxB)
        {
            j++;
        }
        else
        {
            i++;
        }
    }

    return sum;
}

double MixVectorDot(IN Vector_S *A, IN SparseVector_S *B)
{
    int i,idx;
    int n = B->length;
    double sum = 0;

    for (i = 0;i < n;i++)
    {
        idx = B->index[i];
        sum += A[idx]*B->val[i];
    }

    return sum;
}

/*******************************************************************************
    Func Name: VectorSkew
 Date Created: 2011-02-10
       Author: zhusong
     Function: 3维向量的反对称矩阵
        Input: IN Vector_S *A, 输入向量A(3*1)
       Output: OUT Matrix_S *B, 叉积向量(3*1)
       Return: 
      Caution: 
      Description: A X V = B*V
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void VectorSkew(IN Vector_S *A, IN Matrix_S *B)
{
    double *b = B->data;

    b[0] = 0;       b[1] = -A[2];   b[2] = A[1];
    b[3] = A[2];    b[4] = 0;       b[5] = -A[0];
    b[6] = -A[1];   b[7] = A[0];    b[8] =0;

    B->width = 3;
    B->height = 3;

    return;
}


// 多通道像素值距离
int VectorDistPixelInt(IN PIXEL *a, IN PIXEL *b, IN int n, IN int type)
{
    int i;
    int dist;
    int sum = 0;

    switch(type)
    {
        case VECTOR_DIST_MANHATTAN:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                sum += gabs(dist);
            }
            break;
        }
        case VECTOR_DIST_MANHATTAN2:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                sum += dist;
            }
            break;
        }
        case VECTOR_DIST_EUCLIDEAN:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                sum += dist*dist;
            }
            break;
        }
        case VECTOR_DIST_INF:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                dist = gabs(dist);
                if (dist > sum) sum = dist;
            }
            break;
        }
        default:    break;
    }

    return sum;
}

double VectorDistPixel(IN PIXEL *a, IN PIXEL *b, IN int n, IN int type)
{
    int i;
    double dist;
    double sum = 0;

    switch(type)
    {
        case VECTOR_DIST_MANHATTAN:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                sum += gabs(dist);
            }
            break;
        }
        case VECTOR_DIST_EUCLIDEAN:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                sum += dist*dist;
            }
            break;
        }
        case VECTOR_DIST_INF:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                dist = gabs(dist);
                if (dist > sum) sum = dist;
            }
            break;
        }
        default:    break;
    }

    return sum;
}

double VectorDistPixdou(IN PIXEL *a, IN double *b, IN int n, IN int type)
{
    int i;
    double dist;
    double sum = 0;

    switch(type)
    {
        case VECTOR_DIST_MANHATTAN:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                sum += gabs(dist);
            }
            break;
        }
        case VECTOR_DIST_EUCLIDEAN:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                sum += dist*dist;
            }
            break;
        }
        case VECTOR_DIST_INF:
        {
            for (i = 0;i < n;i++)
            {
                dist = a[i]-b[i];
                dist = gabs(dist);
                if (dist > sum) sum = dist;
            }
            break;
        }
        default:    break;
    }

    return sum;
}

/*******************************************************************************
    Func Name: VectorGibbs
 Date Created: 2011-02-10
       Author: zhusong
     Function: Gibbs函数
        Input: IN Vector_S *A, 输入向量A
               IN int n, 向量长度
               IN double sigma, 函数参数
       Output: OUT Vector_S *B, 输出向量
       Return: 
      Caution: 
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void VectorGibbs(IN Vector_S *A, OUT Vector_S *B, IN int n, IN double sigma)
{
    int i;
    double tao = -1.0/sigma;

    for (i = 0;i < n;i++)
    {
        B[i] = exp(tao*A[i]);
    }

    return;
}

/*******************************************************************************
    Func Name: VectorMap
 Date Created: 2011-02-10
       Author: zhusong
     Function: 向量映射
        Input: IN VECTOR_TYPE *A, 输入向量A
               IN VECTOR_TYPE *map, 映射表
               IN int n, 向量数量
               IN int c, 映射维数
       Output: OUT VECTOR_TYPE *B, 映射值
       Return: 
      Caution: A,B不能共享内存
      Description: 
--------------------------------------------------------------------------------
  Modification History                                                        
  DATE        NAME             DESCRIPTION                                    
--------------------------------------------------------------------------------  
  YYYY-MM-DD                                                           
                                                                              
*******************************************************************************/
void VectorMapPixel(IN PIXEL *A, 
                    OUT PIXEL *B, 
                    IN PIXEL *map, 
                    IN int n, 
                    IN int c)
{
    int i, k;
    PIXEL *p;

    for (i = 0, p = B;i < n;i++, p += c)
    {
        k = A[i];
        memcpy(p, map+k*c, c*sizeof(PIXEL));
    }

    return;
}

void VectorMapInt(IN int *A, 
                  OUT int *B, 
                  IN int *map, 
                  IN int n, 
                  IN int c)
{
    int i, k;
    int *p;

    if (c == 1)
    {
        for (i = 0;i < n;i++)
        {
            k = A[i];
            B[i] = map[k];
        }
    }
    else
    {
        for (i = 0, p = B;i < n;i++, p += c)
        {
            k = A[i];
            memcpy(p, map+k*c, c*sizeof(int));
        }
    }
    
    return;
}

void VectorUnit2Float(IN float nx, 
                      IN float ny,
                      OUT float *onx, 
                      OUT float *ony)
{
    float v;

    if (fabs(ny) < FLOAT_EPS)   ny = 1;
    v = (float)sqrt(nx*nx + ny*ny);
    *onx = nx / v;
    *ony = ny / v;
}

void VectorUnit3Float(IN float nx, 
                      IN float ny,
                      IN float nz,
                      OUT float *onx, 
                      OUT float *ony,
                      OUT float *onz)
{
    float v;

    if (fabs(nz) < FLOAT_EPS)   nz = 1;
    v = (float)sqrt(nx*nx + ny*ny + nz*nz);
    *onx = nx / v;
    *ony = ny / v;
    *onz = nz / v;
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
