#include "BaseConstDef.h"
#include "BaseTypeDef.h"
#include "BaseFuncDef.h"
#include <string.h>

// 当 (2*r+1)*(2*r+1) < 256 可采用 unit8
typedef unit8 ctmfstat_t;
// typedef unit16 ctmfstat_t;

#define inline
#define CTMF_OPT_SHIFT      4
#define CTMF_OPT_SCALE      16
#define CTMF_HIGH(x)        ((x) >> CTMF_OPT_SHIFT)
#define CTMF_LOW(x)         ((x) & 0xF)
#define CTMF_MIN(a,b)       ((a) > (b) ? (b) : (a))
#define CTMF_MAX(a,b)       ((a) < (b) ? (b) : (a))

#define CTMF_HIST(p,op)                     \
    v = *p;                                 \
    high = CTMF_HIGH(v);                    \
    low = CTMF_LOW(v);                      \
    h_coarse[ind + high] op;                \
    h_fine[ind + widths*high + low] op;

#ifdef PLATFORM_CCS

// ctmfstat_t 类型 必须为 unit8
#define CTMF_OPTCCS_SCALE       4
static inline void histogram_add(const ctmfstat_t *x, ctmfstat_t *y)
{
    int i;
    int *px = (int *)x;
    int *py = (int *)y;

    for (i = 0; i < CTMF_OPTCCS_SCALE;i++)
    {
        py[i] = _add4(py[i], px[i]);
    }
}
static inline void histogram_sub(const ctmfstat_t *x, ctmfstat_t *y)
{
    int i;
    int *px = (int *)x;
    int *py = (int *)y;

    for (i = 0; i < CTMF_OPTCCS_SCALE;i++)
    {
        py[i] = _sub4(py[i], px[i]);
    }
}
static inline void histogram_muladd(const ctmfstat_t a, const ctmfstat_t *x, ctmfstat_t *y)
{
    int i;
    unit32 *px = (unit32 *)x;
    unit32 *py = (unit32 *)y;
	ctmfstat_t b[4];
    unit32 ma, max;
    mix64 temp;

    b[0] = b[1] = b[2] = b[3] = a;
    ma = *((unit32 *)(b));
    for (i = 0; i < CTMF_OPTCCS_SCALE;i++)
    {
        temp.i64 = _mpyu4ll(ma, px[i]);
        max = _packl4(temp.du32.ui32high, temp.du32.ui32low);
        py[i] = _add4(py[i], max);
    }
}
#else
static inline void histogram_add(const ctmfstat_t *x, ctmfstat_t *y)
{
    int i;
    for (i = 0; i < CTMF_OPT_SCALE;i++)
    {
        y[i] += x[i];
    }
}
static inline void histogram_sub(const ctmfstat_t *x, ctmfstat_t *y)
{
    int i;
    for (i = 0; i < CTMF_OPT_SCALE;i++)
    {
        y[i] -= x[i];
    }
}
static inline void histogram_muladd(const ctmfstat_t a, const ctmfstat_t *x, ctmfstat_t *y)
{
    int i;
    for (i = 0; i < CTMF_OPT_SCALE;i++)
    {
        y[i] += a * x[i];
    }
}
#endif

void ctmfopt(const unsigned char* src,
             unsigned char* dst,
             const int height,
             const int width,
             const int channel,
             const int r)
{
    const int t = 2*r*r + 2*r;     // 中值
    int widths = width*CTMF_OPT_SCALE;

    int i, j, k;
    int ind;
    int high, low;
    unsigned char v;
    const unsigned char *p;
    ctmfstat_t *h_coarse, *h_fine;

    int luc[CTMF_OPT_SCALE];

#ifdef PLATFORM_CCS
    ctmfstat_t *coarse = ram_MallocType(ctmfstat_t, CTMF_OPT_SCALE);
    ctmfstat_t *fine = ram_MallocType(ctmfstat_t, CTMF_OPT_SCALE*CTMF_OPT_SCALE);
#else
    ctmfstat_t coarse[CTMF_OPT_SCALE];
    ctmfstat_t fine[CTMF_OPT_SCALE*CTMF_OPT_SCALE];
#endif

    h_coarse = MallocType(ctmfstat_t, CTMF_OPT_SCALE * width);
    memset(h_coarse, 0, CTMF_OPT_SCALE * width * sizeof(ctmfstat_t));
    h_fine = MallocType(ctmfstat_t, CTMF_OPT_SCALE * CTMF_OPT_SCALE * width);
    memset(h_fine, 0, CTMF_OPT_SCALE * CTMF_OPT_SCALE * width * sizeof(ctmfstat_t));

//// h_coarse       (灰度h, x坐标)
//// coarse         (灰度h)
//// h_fine         (灰度l，x坐标，灰度h)
//// fine           (灰度l，灰度h)

    // 第0行 && 边界复制
    for (j = 0, p = src, ind = 0; j < width; j++, p += channel, ind += CTMF_OPT_SCALE)
    {
        CTMF_HIST(p, += r+2);
    }

    for (i = 1; i < r; i++, p += width)
    {
        for (j = 0, ind = 0; j < width; j++, p += channel, ind += CTMF_OPT_SCALE)
        {
            CTMF_HIST(p, ++);
        }
    }

    for (i = 0; i < height; i++)
    {
        int s;
        int subline, addline;
        ctmfstat_t *p_h_coarse;
        ctmfstat_t *p_h_fine;

        // 减去一行的统计
        subline = i-r-1;
        subline = CTMF_MAX(0, subline);
        p = src + width * channel * subline;
        for (j = 0, ind = 0; j < width; j++, p += channel, ind += CTMF_OPT_SCALE)
        {
            CTMF_HIST(p, --);
        }

        // 加入一行的统计
        addline = i+r;
        addline = CTMF_MIN( height-1, addline);
        p = src + width * channel * addline;
        for (j = 0, ind = 0; j < width; j++, p += channel, ind += CTMF_OPT_SCALE)
        {
            CTMF_HIST(p, ++);
        }

//// 处理之后，h_coarse 和 h_fine 是以 i 为中心，2r+1 行内的垂直方向累积统计 ! ////

//// 现在开始对 h_coarse 和 h_fine 在水平方向上进行累积统计，放入 coarse 和 fine 中 ! ////

        memset(coarse, 0, CTMF_OPT_SCALE*sizeof(ctmfstat_t));
        memset(fine, 0, CTMF_OPT_SCALE*CTMF_OPT_SCALE*sizeof(ctmfstat_t));
        memset(luc, 0, CTMF_OPT_SCALE*sizeof(int));

        // coarse: 第0列 && 边界复制
        s = r + 1;
        histogram_muladd(s, h_coarse, coarse);

        // coarse: 1 ... r-1 列
        for (j = 1, p_h_coarse = h_coarse + CTMF_OPT_SCALE; j < r; j++, p_h_coarse += CTMF_OPT_SCALE)
        {
            histogram_add(p_h_coarse, coarse);
        }

        // fine: 第0列 && 边界复制
        s = 2 * r + 1;
        for (k = 0, p_h_fine = h_fine; k < CTMF_OPT_SCALE; k++, p_h_fine += widths)
        {
            histogram_muladd(s, p_h_fine, fine + k*CTMF_OPT_SCALE);
        }

        for (j = 0; j < width; j++)
        {
            int sum;
            int subcol, addcol;
            ctmfstat_t *segment;
            int x, b, start, end;

            // 加入一列的统计
            addcol = j + r;
            addcol = CTMF_MIN(addcol, width-1);
            p_h_coarse = h_coarse + CTMF_OPT_SCALE*addcol;
            histogram_add(p_h_coarse, coarse);

            /* Find median at coarse level */
            sum = 0;
            for (k = 0; k < CTMF_OPT_SCALE; k++)
            {
                sum += coarse[k];
                if (sum > t)
                {
                    sum -= coarse[k];
                    break;
                }
            }

            // 减去一列的统计
            subcol = j - r;
            subcol = CTMF_MAX(subcol, 0);
            p_h_coarse = h_coarse + CTMF_OPT_SCALE*subcol;
            histogram_sub(p_h_coarse, coarse);

// 输出结果为 k, sum

            x = luc[k];
            segment = fine + k*CTMF_OPT_SCALE;
            start = j-r;
            end = j+r+1;
            if (x <= start)
            {
                int minend = CTMF_MIN(end, width);
                memset(segment, 0, CTMF_OPT_SCALE * sizeof(ctmfstat_t));
                p_h_fine = h_fine + CTMF_OPT_SCALE*(width*k+start);
                for (x = start; x < minend; x++, p_h_fine += CTMF_OPT_SCALE)
                {
                    histogram_add(p_h_fine, segment);
                }
                if (x < end)
                {
                    p_h_fine = h_fine + CTMF_OPT_SCALE*(width*k+(width-1));
                    histogram_muladd(end - width, p_h_fine, segment);
                    x = end;
                }
            }
            else 
            {
                for ( ; x < end; x++)
                {
                    subcol = x-2*r-1;
                    subcol = CTMF_MAX(subcol, 0);
                    p_h_fine = h_fine + CTMF_OPT_SCALE*(width*k+subcol);
                    histogram_sub( p_h_fine, segment );

                    addcol = CTMF_MIN(x, width-1);
                    p_h_fine = h_fine + CTMF_OPT_SCALE*(width*k+addcol);
                    histogram_add( p_h_fine, segment );
                }
            }
            luc[k] = x;

            /* Find median in segment */
            for (b = 0; b < CTMF_OPT_SCALE; b++)
            {
                sum += segment[b];
                if (sum > t)
                {
                    dst[(width*i+j)*channel] = CTMF_OPT_SCALE*k + b;
                    break;
                }
            }
        }
    }

#ifdef PLATFORM_CCS
    ram_FreeType(coarse, ctmfstat_t, CTMF_OPT_SCALE);
    ram_FreeType(fine, ctmfstat_t, CTMF_OPT_SCALE*CTMF_OPT_SCALE);
#endif

    FreeType(h_coarse, ctmfstat_t, CTMF_OPT_SCALE * width);
    FreeType(h_fine, ctmfstat_t, CTMF_OPT_SCALE * CTMF_OPT_SCALE * width);

    return;
}

