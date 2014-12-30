/**********************************************************************************
 * Si8240 Linux Driver
 *
 * Copyright (C) 2011-2012 Silicon Image Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the
 * GNU General Public License for more details.
 *
 **********************************************************************************/

/*
 *****************************************************************************
 * @file  EDID.c
 *
 * @brief Implementation of the Foo API.
 *
 *****************************************************************************
*/


#include "si_memsegsupport.h"
#include "si_platform.h"
#include "si_common.h"
#if !defined(__KERNEL__) //(
#include "hal_timers.h"
#else //)(
#include "sii_hal.h"
#endif //)
#include "si_cra.h"
#include "si_cra_cfg.h"
#include "si_common_regs.h"
#include "si_drv_hdmi_tx_lite_edid.h"
#include "si_bitdefs.h"
#include "si_mhl_defs.h"
#include "si_mhl_tx_base_drv_api.h"

