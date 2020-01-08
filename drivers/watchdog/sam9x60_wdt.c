// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Microchip SAM9X60 Watchdog Timer
 *
 * Copyright (C) 2019 Microchip Technology, Inc.
 * Author: Eugen Hristev <eugen.hristev@microchip.com>
 *
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/watchdog.h>

#include "sam9x60_wdt.h"

/* minimum and maximum watchdog timeout, in seconds */
#define MIN_WDT_TIMEOUT		1
#define MAX_WDT_TIMEOUT		16
#define WDT_DEFAULT_TIMEOUT	MAX_WDT_TIMEOUT

#define WDT_SEC2TICKS(s)	((s) ? (((s) << 8) - 1) : 0)

struct sam9x60_wdt {
	struct watchdog_device	wdd;
	void __iomem		*reg_base;
	u32			mr;
	u32			ir;
	unsigned long		last_ping;
};

static int wdt_timeout;
static bool nowayout = WATCHDOG_NOWAYOUT;

module_param(wdt_timeout, int, 0);
MODULE_PARM_DESC(wdt_timeout,
		 "Watchdog timeout in seconds. (default = "
		 __MODULE_STRING(WDT_DEFAULT_TIMEOUT) ")");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

#define wdt_enabled (!(wdt->mr & AT91_WDT_WDDIS))

#define wdt_read(wdt, field) \
	readl_relaxed((wdt)->reg_base + (field))

/* 4 slow clock periods is 4/32768 = 122.07us*/
#define WDT_DELAY	usecs_to_jiffies(123)

static void wdt_write(struct sam9x60_wdt *wdt, u32 field, u32 val)
{
	/*
	 * WDT_CR and WDT_MR must not be modified within three slow clock
	 * periods following a restart of the watchdog performed by a write
	 * access in WDT_CR.
	 */
	while (time_before(jiffies, wdt->last_ping + WDT_DELAY))
		usleep_range(30, 125);
	writel_relaxed(val, wdt->reg_base + field);
	wdt->last_ping = jiffies;
}

static void wdt_write_nosleep(struct sam9x60_wdt *wdt, u32 field, u32 val)
{
	if (time_before(jiffies, wdt->last_ping + WDT_DELAY))
		usleep_range(123, 250);
	writel_relaxed(val, wdt->reg_base + field);
	wdt->last_ping = jiffies;
}

static int sam9x60_wdt_start(struct watchdog_device *wdd)
{
	struct sam9x60_wdt *wdt = watchdog_get_drvdata(wdd);

	wdt->mr &= ~AT91_WDT_WDDIS;
	wdt_write(wdt, AT91_WDT_MR, wdt->mr);
	wdt_write_nosleep(wdt, AT91_WDT_IER, wdt->ir);

	return 0;
}

static int sam9x60_wdt_stop(struct watchdog_device *wdd)
{
	struct sam9x60_wdt *wdt = watchdog_get_drvdata(wdd);

	wdt->mr |= AT91_WDT_WDDIS;
	wdt_write(wdt, AT91_WDT_MR, wdt->mr);
	wdt_write_nosleep(wdt, AT91_WDT_IDR, wdt->ir);

	return 0;
}

static int sam9x60_wdt_ping(struct watchdog_device *wdd)
{
	struct sam9x60_wdt *wdt = watchdog_get_drvdata(wdd);

	wdt_write(wdt, AT91_WDT_CR, AT91_WDT_KEY | AT91_WDT_WDRSTT);

	return 0;
}

static int sam9x60_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	struct sam9x60_wdt *wdt = watchdog_get_drvdata(wdd);

	wdt_write(wdt, AT91_WDT_WLR,
		  AT91_WDT_SET_COUNTER(WDT_SEC2TICKS(timeout)));

	wdd->timeout = timeout;

	return 0;
}

static const struct watchdog_info sam9x60_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING,
	.identity = "Microchip SAM9X60 Watchdog",
};

static const struct watchdog_ops sam9x60_wdt_ops = {
	.owner = THIS_MODULE,
	.start = sam9x60_wdt_start,
	.stop = sam9x60_wdt_stop,
	.ping = sam9x60_wdt_ping,
	.set_timeout = sam9x60_wdt_set_timeout,
};

static irqreturn_t sam9x60_wdt_irq_handler(int irq, void *dev_id)
{
	struct sam9x60_wdt *wdt = platform_get_drvdata(dev_id);

	if (wdt_read(wdt, AT91_WDT_ISR)) {
		pr_crit("Atmel Watchdog Software Reset\n");
		emergency_restart();
		pr_crit("Reboot didn't succeed\n");
	}

	return IRQ_HANDLED;
}

static int of_sam9x60_wdt_init(struct device_node *np, struct sam9x60_wdt *wdt)
{
	const char *tmp;

	wdt->mr = AT91_WDT_WDDIS;

	if (!of_property_read_string(np, "atmel,watchdog-type", &tmp) &&
	    !strcmp(tmp, "software"))
		wdt->ir = AT91_WDT_PERINT;
	else
		wdt->mr |= AT91_WDT_PERIODRST;

	if (of_property_read_bool(np, "atmel,idle-halt"))
		wdt->mr |= AT91_WDT_WDIDLEHLT;

	if (of_property_read_bool(np, "atmel,dbg-halt"))
		wdt->mr |= AT91_WDT_WDDBGHLT;

	return 0;
}

static int sam9x60_wdt_init(struct sam9x60_wdt *wdt)
{
	u32 reg;
	/*
	 * When booting and resuming, the bootloader may have changed the
	 * watchdog configuration.
	 * If the watchdog is already running, we can safely update it.
	 * Else, we have to disable it properly.
	 */
	if (wdt_enabled) {
		wdt_write_nosleep(wdt, AT91_WDT_MR, wdt->mr);
		wdt_write_nosleep(wdt, AT91_WDT_IER, wdt->ir);
		wdt_write(wdt, AT91_WDT_WLR,
			  AT91_WDT_SET_COUNTER(WDT_SEC2TICKS(WDT_DEFAULT_TIMEOUT)));

	} else {
		reg = wdt_read(wdt, AT91_WDT_MR);
		if (!(reg & AT91_WDT_WDDIS))
			wdt_write_nosleep(wdt, AT91_WDT_MR,
					  reg | AT91_WDT_WDDIS);
	}
	return 0;
}

static int sam9x60_wdt_probe(struct platform_device *pdev)
{
	struct watchdog_device *wdd;
	struct sam9x60_wdt *wdt;
	struct resource *res;
	void __iomem *regs;
	u32 irq = 0;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdd = &wdt->wdd;
	wdd->timeout = WDT_DEFAULT_TIMEOUT;
	wdd->info = &sam9x60_wdt_info;
	wdd->ops = &sam9x60_wdt_ops;
	wdd->min_timeout = MIN_WDT_TIMEOUT;
	wdd->max_timeout = MAX_WDT_TIMEOUT;
	wdt->last_ping = jiffies;

	watchdog_set_drvdata(wdd, wdt);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	wdt->reg_base = regs;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq)
		dev_warn(&pdev->dev, "failed to get IRQ from DT\n");

	ret = of_sam9x60_wdt_init(pdev->dev.of_node, wdt);
	if (ret)
		return ret;

	if ((wdt->ir & AT91_WDT_PERINT) && irq) {
		ret = devm_request_irq(&pdev->dev, irq, sam9x60_wdt_irq_handler,
				       IRQF_SHARED | IRQF_IRQPOLL |
				       IRQF_NO_SUSPEND, pdev->name, pdev);
		if (ret) {
			dev_err(&pdev->dev,
				"cannot register interrupt handler\n");
			return ret;
		}
	}

	watchdog_init_timeout(wdd, wdt_timeout, &pdev->dev);

	ret = sam9x60_wdt_init(wdt);
	if (ret)
		return ret;

	watchdog_set_nowayout(wdd, nowayout);

	ret = watchdog_register_device(wdd);
	if (ret) {
		dev_err(&pdev->dev, "failed to register watchdog device\n");
		return ret;
	}

	platform_set_drvdata(pdev, wdt);

	dev_info(&pdev->dev, "initialized (timeout = %d sec, nowayout = %d)\n",
		 wdd->timeout, nowayout);

	return 0;
}

static int sam9x60_wdt_remove(struct platform_device *pdev)
{
	struct sam9x60_wdt *wdt = platform_get_drvdata(pdev);

	sam9x60_wdt_stop(&wdt->wdd);

	watchdog_unregister_device(&wdt->wdd);

	return 0;
}

static const struct of_device_id sam9x60_wdt_of_match[] = {
	{ .compatible = "microchip,sam9x60-wdt", },
	{ }
};
MODULE_DEVICE_TABLE(of, sam9x60_wdt_of_match);

#ifdef CONFIG_PM_SLEEP
static int sam9x60_wdt_resume(struct device *dev)
{
	struct sam9x60_wdt *wdt = dev_get_drvdata(dev);

	/*
	 * FIXME: writing MR also pings the watchdog which may not be desired.
	 * This should only be done when the registers are lost on suspend but
	 * there is no way to get this information right now.
	 */
	sam9x60_wdt_init(wdt);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sam9x60_wdt_pm_ops, NULL,
			 sam9x60_wdt_resume);

static struct platform_driver sam9x60_wdt_driver = {
	.probe		= sam9x60_wdt_probe,
	.remove		= sam9x60_wdt_remove,
	.driver		= {
		.name	= "sam9x60_wdt",
		.pm	= &sam9x60_wdt_pm_ops,
		.of_match_table = sam9x60_wdt_of_match,
	}
};
module_platform_driver(sam9x60_wdt_driver);

MODULE_AUTHOR("Eugen Hristev");
MODULE_DESCRIPTION("Microchip SAM9X60 Watchdog Timer driver");
MODULE_LICENSE("GPL v2");
