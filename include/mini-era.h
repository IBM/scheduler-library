#ifndef _MINI_ERA_H_
#define _MINI_ERA_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#ifndef __user
#define __user
#endif
#endif /* __KERNEL__ */

#include <esp.h>
#include <esp_accelerator.h>

struct vitdodec_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned cbps;
	unsigned ntraceback;
	unsigned data_bits;
	unsigned src_offset;
	unsigned dst_offset;
};

#define VITDODEC_IOC_ACCESS	_IOW ('S', 0, struct vitdodec_access)

typedef int8_t vitHW_token_t;


// This is for the FFT Accelerator

#if (USE_FFT_FX == 64)
//typedef unsigned long long fftHW_token_t;
typedef int64_t fftHW_token_t;
typedef double fftHW_native_t;
#define fx2float fixed64_to_double
#define float2fx double_to_fixed64
#define FX_IL 42
#elif (USE_FFT_FX == 32)
typedef int fftHW_token_t;
typedef float fftHW_native_t;
#define fx2float fixed32_to_float
#define float2fx float_to_fixed32
#define FX_IL 14
#endif /* FFT_FX_WIDTH */

#if USE_FFT_ACCEL_VERSION == 1
/* <<--params-def-->> */
/* #define FFTHW_LOGN_SAMPLES     14 */
/* #define FFTHW_LEN         (1 << FFTHW_LOGN_SAMPLES) */
#define FFTHW_NO_BITREV    0
#define FFTHW_DO_BITREV    1
/* <<--params-->> */
//const int32_t fftHW_do_bitrev = FFTHW_DO_BITREV;
//const int32_t fftHW_len = FFTHW_LEN;
//const int32_t fftHW_logn_samples = FFTHW_LOGN_SAMPLES;

 struct fftHW_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned logn_samples;
	unsigned do_bitrev;
	unsigned src_offset;
	unsigned dst_offset;
 };
#elif USE_FFT_ACCEL_VERSION == 2
 /* <<--params-def-->> */
 #define FFTHW_NO_INVERSE    0
 #define FFTHW_NO_SHIFT      0
 #define FFTHW_DO_INVERSE    1
 #define FFTHW_DO_SHIFT      1
 /* <<--params-->> */

 struct fftHW_access {
	struct esp_access esp;
	/* <<--regs-->> */
	unsigned logn_samples;
	unsigned num_ffts;
	unsigned do_inverse;
	unsigned do_shift;
	unsigned scale_factor; // (not used)
	unsigned src_offset;
	unsigned dst_offset;
 };
#else
 struct fftHW_acess { 
	unsigned NOT_A_VALID_FFT_ACCE_VERSION;
 };
#endif
#define FFTHW_IOC_ACCESS	_IOW ('S', 0, struct fftHW_access)


#endif /* _MINI_ERA_H_ */
