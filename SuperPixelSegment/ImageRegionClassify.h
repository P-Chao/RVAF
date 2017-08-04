/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: ImageRegionClassify.h 			                                 */
/*                                                                           */
/* Author: zhusong                                                           */
/*                                                                           */
/* Version: 1.01                                                             */
/*                                                                           */
/* Date: 2014/12/29                                                          */
/*                                                                           */
/* Description: ImageRegionFeature中对应的分类函数                           */
/*                                                                           */
/* Others:                                                                   */
/*                                                                           */
/* History:                                                                  */
/*                                                                           */
/*****************************************************************************/
#ifndef _IMAGE_REGION_CLASSIFY_H_
#define _IMAGE_REGION_CLASSIFY_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>


extern int IrcClassifyOne(IN int *feature);
extern void IrcClassifyImage(IN IMAGE32_S *segImage, 
                             IN IMAGE_S *lblImage, 
                             OUT int *map,
                             IN int *feature, 
                             IN int regionnum);

#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif
