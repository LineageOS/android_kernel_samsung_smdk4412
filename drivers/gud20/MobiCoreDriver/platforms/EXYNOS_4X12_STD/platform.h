/*
 * Header file of MobiCore Driver Kernel Module Platform
 * specific structures
 *
 * Internal structures of the McDrvModule
 *
 * Header file the MobiCore Driver Kernel Module,
 * its internal structures and defines.
 *
 * <-- Copyright Giesecke & Devrient GmbH 2009-2012 -->
 * <-- Copyright Trustonic Limited 2013 -->
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MC_DRV_PLATFORM_H_
#define _MC_DRV_PLATFORM_H_

/* MobiCore Interrupt. */
#define MC_INTR_SSIQ                        (32 + 114)

/* Enable MobiCore mem traces */
#define MC_MEM_TRACES

/* Enable Runtime Power Management */
#ifdef CONFIG_PM_RUNTIME
 #define MC_PM_RUNTIME
#endif

/* Enable Fastcall worker thread */
#define MC_FASTCALL_WORKER_THREAD

#endif /* _MC_DRV_PLATFORM_H_ */
