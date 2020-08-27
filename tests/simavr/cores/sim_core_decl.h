// Autogenerated using normal simavr
// and manually tweaked
#ifndef __SIM_CORE_DECL_H__
#define __SIM_CORE_DECL_H__

#include "sim_core_config.h"
#if CONFIG_MEGA2560
extern avr_kind_t mega2560;
#endif
#if CONFIG_MEGA328
extern avr_kind_t mega328;
#endif

extern avr_kind_t * avr_kind[];
#ifdef AVR_KIND_DECL
avr_kind_t * avr_kind[] = {
#if CONFIG_MEGA2560
	&mega2560,
#endif
#if CONFIG_MEGA328
	&mega328,
#endif
	NULL
};
#endif

#endif