#ifndef CTMF_OPT_H
#define CTMF_OPT_H

#ifdef __cplusplus
extern "C" {
#endif

extern void ctmfopt(const unsigned char* src,
              unsigned char* dst,
              const int height,
              const int width,
              const int channel,
              const int r);

#ifdef __cplusplus
}
#endif

#endif
