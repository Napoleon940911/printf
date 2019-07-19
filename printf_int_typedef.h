#ifndef _PRINTF_INT_TYPEDEF_H_
#define _PRINTF_INT_TYPEDEF_H_

#ifdef __cplusplus
extern "C" {
#endif

// when migrating to other platforms, the following "typedef" should be redefined

// the following "typedef" are especially for CC2530

typedef   signed char  int8_t;
typedef unsigned char  uint8_t;
typedef   signed short int16_t;
typedef unsigned short uint16_t;
typedef   signed long  int32_t;
typedef unsigned long  uint32_t;

typedef  int32_t       intmax_t;
typedef uint32_t       uintmax_t;
typedef  int16_t       intptr_t;
typedef uint16_t       uintptr_t;

#ifdef __cplusplus
}
#endif

#endif  // _PRINTF_INT_TYPEDEF_H_
