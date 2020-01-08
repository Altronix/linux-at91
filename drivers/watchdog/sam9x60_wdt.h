/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * drivers/watchdog/sam9x60_wdt.h
 *
 * Copyright (C) 2019 Microchip Technology, Inc.
 *
 * SAM9X60 Watchdog Timer (WDT) - System peripherals registers.
 * Based on SAM9X60 datasheet.
 *
 */

#ifndef SAM9X60_WDT_H
#define SAM9X60_WDT_H

#define AT91_WDT_CR		0x00			/* Watchdog Control Register */
#define		AT91_WDT_WDRSTT		BIT(0)		/* Restart */
#define		AT91_WDT_KEY		(0xa5 << 24)		/* KEY Password */

#define AT91_WDT_MR		0x04			/* Watchdog Mode Register */
#define		AT91_WDT_PERIODRST	BIT(4)		/* Period Reset */
#define		AT91_WDT_RPTHRST	BIT(5)		/* Minimum Restart Period */
#define		AT91_WDT_WDDIS		BIT(12)		/* Disable */
#define		AT91_WDT_WDDBGHLT	BIT(28)		/* Debug Halt */
#define		AT91_WDT_WDIDLEHLT	BIT(29)		/* Idle Halt */

#define AT91_WDT_VR		0x08			/* Watchdog Timer Value Register */

#define AT91_WDT_WLR		0x0c
#define		AT91_WDT_COUNTER	(0xfff << 0)		/* Watchdog Period Value */
#define		AT91_WDT_SET_COUNTER(x)	((x) & AT91_WDT_COUNTER)

#define AT91_WDT_IER		0x14			/* Interrupt Enable Register */
#define		AT91_WDT_PERINT		BIT(0)		/* Period Interrupt Enable */
#define AT91_WDT_IDR		0x18			/* Interrupt Disable Register */
#define AT91_WDT_ISR		0x1c			/* Interrupt Status Register */

#endif
