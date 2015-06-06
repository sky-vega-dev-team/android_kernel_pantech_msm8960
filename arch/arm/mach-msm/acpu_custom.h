/*
 *  acpu_custom.h
 *
 *  Copyright
 *            (C) 2015 Le Hoang <icy.lord.love.angel@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ARCH_ARM_MACH_MSM_ACPU_CUSTOM_H
#define __ARCH_ARM_MACH_MSM_ACPU_CUSTOM_H

//Define for acpuclock-8064.c, acpuclock-krait.c, board-8064-regulator.c, msm_dcvs.c
#if defined(CONFIG_CPU_OVERCLOCK) || defined(CONFIG_LOW_CPUCLOCKS)
#define HFPLL_MIN_VDD		800000//Stock is 850000	/* uV */
#define HFPLL_MAX_VDD		1350000//stock is 1300000	/* uV */
#else
#define HFPLL_MIN_VDD		850000
#define HFPLL_MAX_VDD		1300000
#endif

//Loạn não :3
#ifdef CONFIG_CPU_OVERCLOCK
#define NUM_FREQS 10
#else
#define NUM_FREQS 0
#endif

#ifdef CONFIG_LOW_CPUCLOCKS
#define FREQ_TABLE_SIZE		(36 + NUM_FREQS)
#define DCVS_MAX_NUM_FREQS	(16 + NUM_FREQS)
#else
#define DCVS_MAX_NUM_FREQS	(15 + NUM_FREQS)
#define FREQ_TABLE_SIZE		(35 + NUM_FREQS)
#endif

//#define MAX_VDD_MEM_DIG	1150000 /* uV */



#endif /*__ARCH_ARM_MACH_MSM_ACPU_CUSTOM_H*/
