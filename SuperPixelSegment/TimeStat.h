/*****************************************************************************/
/*                                                                           */
/*                            朱松的图像处理平台                             */
/*                                                                           */
/* FileName: TimeStat.h    			                                         */
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

#ifndef _TIME_STAT_H_
#define _TIME_STAT_H_

#ifdef __cplusplus
extern "C"{
#endif /* end of __cplusplus */

#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <time.h>

/* clock时间统计中因调用clock函数多余的开销 */
extern clock_t overheadClock;
/* TsTime时间统计中因调用时间函数多余的开销 */
extern int64 overheadTime;

#ifndef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 1000
#endif

/* clock时间统计定义 */
#define TsClockInit()               \
{                                   \
    clock_t startM,stopM;           \
    startM = clock();               \
    stopM = clock();                \
    overheadClock = stopM - startM; \
}
#define TsClockStart(timeVarM)      \
{                                   \
    timeVarM -= clock();            \
}
#define TsClockEnd(timeVarM)        \
{                                   \
    timeVarM += clock();            \
    timeVarM -= overheadClock;      \
}


#ifdef PLATFORM_VC6

#include <windows.h>

/* QueryPerformanceCounter时间统计定义 */
#define TsTimeInit()                                        \
{                                                           \
    LARGE_INTEGER startM,endM;                              \
    QueryPerformanceCounter(&startM);                       \
    QueryPerformanceCounter(&endM);                         \
    overheadTime = endM.QuadPart - startM.QuadPart;         \
}
#define TsTimeStart(timeVarM)                               \
{                                                           \
    LARGE_INTEGER startM;                                   \
    QueryPerformanceCounter(&startM);                       \
    timeVarM -= startM.QuadPart;                            \
}
#define TsTimeEnd(timeVarM)                                 \
{                                                           \
    LARGE_INTEGER endM;                                     \
    QueryPerformanceCounter(&endM);                         \
    timeVarM += (endM.QuadPart-overheadTime);               \
}
#define TsGetTime(timeVarM)                                 \
{                                                           \
    LARGE_INTEGER frequencyM;                               \
    QueryPerformanceFrequency(&frequencyM);                 \
    timeVarM /= frequencyM.QuadPart;                        \
}

#elif defined(PLATFORM_GCC)

#include <sys/time.h>

/* gettimeofday时间统计定义 */
#define TsTimeInit(timeVarM)                            \
{                                                       \
    timeval startM,endM;                                \
    gettimeofday(&startM, NULL);                        \
    gettimeofday(&endM, NULL);                          \
    overheadTime = 1000000*(endM.tv_sec-startM.tv_sec); \
    overheadTime += (endM.tv_usec - startM.tv_usec);    \
}
#define TsTimeStart(timeVarM)                           \
{                                                       \
    timeval startM;                                     \
    gettimeofday(&startM, NULL);                        \
    timeVarM -= 1000000*startM.tv_sec + startM.tv_usec; \
}
#define TsTimeEnd(timeVarM)                             \
{                                                       \
    timeval endM;                                       \
    gettimeofday(&endM, NULL);                          \
    timeVarM += 1000000*endM.tv_sec + endM.tv_usec;     \
    timeVarM -= overheadTime;                           \
}
#define TsGetTime(timeVarM)                             \
{                                                       \
    timeVarM /= 1000000;                                \
}

#elif defined(PLATFORM_CCS)

/* clock时间统计定义 */
#define TsTimeInit()                \
{                                   \
    clock_t startM,stopM;           \
    startM = clock();               \
    stopM = clock();                \
    overheadTime = stopM - startM;  \
}
#define TsTimeStart(timeVarM)       \
{                                   \
    timeVarM -= clock();            \
}
#define TsTimeEnd(timeVarM)         \
{                                   \
    timeVarM += clock();            \
    timeVarM -= overheadTime;       \
}
#define TsGetTime(timeVarM)         \
{                                   \
    timeVarM /= CLOCKS_PER_SEC;     \
}

#endif




#ifdef __cplusplus
}
#endif /* end of __cplusplus */

#endif

