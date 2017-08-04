/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: DataTypeConvert.c                                               */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2011/01/23                                                          */
/*                                                                           */
/* Description: 数据类型转换                                                 */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "DataTypeConvert.h"

void DataPixel2Double(IN PIXEL *in_data, OUT double *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = (double)in_data[i];
    }
}


void DataInt2Pixel(IN int *in_data, OUT PIXEL *out_data, IN int n)
{
    int i;
    int c;

    for (i = 0;i < n;i++)
    {
        c = in_data[i];
        if (c > IMAGE_WHITE)    out_data[i] = IMAGE_WHITE;
        else if (c < 0)         out_data[i] = IMAGE_BLACK;
        else                    out_data[i] = (PIXEL)c;
    }
    
    return;
}

void DataInt2Float(IN int *in_data, OUT float *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = (float)in_data[i];
    }
}

void DataInt2Double(IN int *in_data, OUT double *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = (double)in_data[i];
    }
}

void DataPixel2Int(IN PIXEL *in_data, OUT int *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = (int)in_data[i];
    }
}

void DataDouble2Int(IN double *in_data, OUT int *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = round(in_data[i]);
    }
}

void DataDouble2Pixel(IN double *in_data, OUT PIXEL *out_data, IN int n)
{
    int i;
    int c;

    for (i = 0;i < n;i++)
    {
        c = round(in_data[i]);
        if (c > IMAGE_WHITE)    out_data[i] = IMAGE_WHITE;
        else if (c < 0)         out_data[i] = IMAGE_WHITE;
        else                    out_data[i] = (PIXEL)c;
    }

    return;
}

void DataFloat2Int(IN float *in_data, OUT int *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = round(in_data[i]);
    }
}

void DataFloat2Double(IN float *in_data, OUT double *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = in_data[i];
    }
}
void DataDouble2Float(IN double *in_data, OUT float *out_data, IN int n)
{
    int i;

    for (i = 0;i < n;i++)
    {
        out_data[i] = (float)in_data[i];
    }
}

void Point2Matrix(IN POINT_S *in_data, OUT Matrix_S *out_data, IN int n)
{
    int i;
    double *p = out_data->data;

    out_data->width = 2;
    out_data->height = n;
    for (i = 0;i < n;i++)
    {
        p[0] = (double)in_data[i].x;
        p[1] = (double)in_data[i].y;
        p += 2;
    }
}

void Point3D642Matrix(IN POINT3D64_S *in_data, OUT Matrix_S *out_data, IN int n)
{
    int i;
    double *p = out_data->data;

    out_data->width = 3;
    out_data->height = n;
    for (i = 0;i < n;i++)
    {
        p[0] = in_data[i].x;
        p[1] = in_data[i].y;
        p[2] = in_data[i].s;
        p += 3;
    }
}

void Matrix2Point3D64(IN Matrix_S *in_data, OUT POINT3D64_S *out_data)
{
    int i;
    int n = in_data->height;
    double *q = in_data->data;
    POINT3D64_S *p = out_data;

    for (i = 0;i < n;i++)
    {
        p->x = q[0];
        p->y = q[1];
        p->s = q[2];
        p++;
        q += 3;
    }
}

int Image2Matrix(IN IMAGE_S *in_data, OUT Matrix_S *out_data, IN int buffer)
{
    int ret = 0;
    int size;

    out_data->width = in_data->width;
    out_data->height = in_data->height;
    out_data->channel = in_data->channel;
    size = in_data->height*in_data->width*in_data->channel;

    if (buffer == 0)
    {
        out_data->data = MallocType(double, size);
        ret = 0;
    }
    else if (buffer < size)
    {
        FreeType(out_data->data, double, buffer);
        out_data->data = MallocType(double, size);
        ret = size;
    }

    DataPixel2Double(in_data->data, out_data->data, size);

    return ret;
}

void Point2D2Point3D(IN POINT64_S *in_data, OUT POINT3D64_S *out_data, IN int n, IN int s)
{
    int i;

    for (i= 0;i < n;i++)
    {
        out_data[i].x = in_data[i].x;
        out_data[i].y = in_data[i].y;
        out_data[i].s = s;
    }
}

#ifdef __cplusplus
}
#endif /* end of __cplusplus */
