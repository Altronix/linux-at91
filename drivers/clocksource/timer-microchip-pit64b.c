// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2019 Microchip Technology Inc.
// Copyright (C) 2019 Claudiu Beznea (claudiu.beznea@microchip.com)

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>

#define MCHP_PIT64B_CR		0x00	/* Control Register */
#define MCHP_PIT64B_CR_START	BIT(0)
#define MCHP_PIT64B_CR_SWRST	BIT(8)

#define MCHP_PIT64B_MR		0x04	/* Mode Register */
#define MCHP_PIT64B_MR_CONT	BIT(0)
#define MCHP_PIT64B_MR_SGCLK	BIT(3)
#define MCHP_PIT64B_MR_SMOD	BIT(4)
#define MCHP_PIT64B_MR_PRES	GENMASK(11, 8)

#define MCHP_PIT64B_LSB_PR	0x08	/* LSB Period Register */

#define MCHP_PIT64B_MSB_PR	0x0C	/* MSB Period Register */

#define MCHP_PIT64B_IER		0x10	/* Interrupt Enable Register */
#define MCHP_PIT64B_IER_PERIOD	BIT(0)

#define MCHP_PIT64B_ISR		0x1C	/* Interrupt Status Register */
#define MCHP_PIT64B_ISR_PERIOD	BIT(0)

#define MCHP_PIT64B_TLSBR	0x20	/* Timer LSB Register */

#define MCHP_PIT64B_TMSBR	0x24	/* Timer MSB Register */

#define MCHP_PIT64B_PRES_MAX	0x10
#define MCHP_PIT64B_DEF_FREQ	2500000UL	/* 2.5 MHz */
#define MCHP_PIT64B_LSBMASK	GENMASK_ULL(31, 0)
#define MCHP_PIT64B_PRESCALER(p)	(MCHP_PIT64B_MR_PRES & ((p) << 8))

#define MCHP_PIT64B_NAME	"pit64b"

struct mchp_pit64b_common_data {
	void __iomem *base;
	struct clk *pclk;
	struct clk *gclk;
	u64 cycles;
	u8 pres;
};

struct mchp_pit64b_clksrc_data {
	struct clocksource *clksrc;
	struct mchp_pit64b_common_data *cd;
};

struct mchp_pit64b_clkevt_data {
	struct clock_event_device *clkevt;
	struct mchp_pit64b_common_data *cd;
};

static struct mchp_pit64b_data {
	struct mchp_pit64b_clksrc_data *csd;
	struct mchp_pit64b_clkevt_data *ced;
} data;

static inline u32 mchp_pit64b_read(void __iomem *base, u32 offset)
{
	return readl_relaxed(base + offset);
}

static inline void mchp_pit64b_write(void __iomem *base, u32 offset, u32 val)
{
	writel_relaxed(val, base + offset);
}

static inline u64 mchp_pit64b_get_period(void __iomem *base)
{
	u32 lsb, msb;

	/* LSB must be read first to guarantee an atomic read of the 64 bit
	 * timer.
	 */
	lsb = mchp_pit64b_read(base, MCHP_PIT64B_TLSBR);
	msb = mchp_pit64b_read(base, MCHP_PIT64B_TMSBR);

	return (((u64)msb << 32) | lsb);
}

static inline void mchp_pit64b_set_period(void __iomem *base, u64 cycles)
{
	u32 lsb, msb;

	lsb = cycles & MCHP_PIT64B_LSBMASK;
	msb = cycles >> 32;

	/* LSB must be write last to guarantee an atomic update of the timer
	 * even when SMOD=1.
	 */
	mchp_pit64b_write(base, MCHP_PIT64B_MSB_PR, msb);
	mchp_pit64b_write(base, MCHP_PIT64B_LSB_PR, lsb);
}

static inline void mchp_pit64b_reset(struct mchp_pit64b_common_data *data,
				     u32 mode, bool irq_ena)
{
	mode |= MCHP_PIT64B_PRESCALER(data->pres);
	if (data->gclk)
		mode |= MCHP_PIT64B_MR_SGCLK;

	mchp_pit64b_write(data->base, MCHP_PIT64B_CR, MCHP_PIT64B_CR_SWRST);
	mchp_pit64b_write(data->base, MCHP_PIT64B_MR, mode);
	mchp_pit64b_set_period(data->base, data->cycles);
	if (irq_ena)
		mchp_pit64b_write(data->base, MCHP_PIT64B_IER,
				  MCHP_PIT64B_IER_PERIOD);
	mchp_pit64b_write(data->base, MCHP_PIT64B_CR, MCHP_PIT64B_CR_START);
}

static u64 mchp_pit64b_read_clk(struct clocksource *cs)
{
	return mchp_pit64b_get_period(data.csd->cd->base);
}

static u64 mchp_sched_read_clk(void)
{
	return mchp_pit64b_get_period(data.csd->cd->base);
}

static struct clocksource mchp_pit64b_clksrc = {
	.name = MCHP_PIT64B_NAME,
	.mask = CLOCKSOURCE_MASK(64),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.rating = 210,
	.read = mchp_pit64b_read_clk,
};

static int mchp_pit64b_clkevt_shutdown(struct clock_event_device *cedev)
{
	mchp_pit64b_write(data.ced->cd->base, MCHP_PIT64B_CR,
			  MCHP_PIT64B_CR_SWRST);

	return 0;
}

static int mchp_pit64b_clkevt_set_periodic(struct clock_event_device *cedev)
{
	mchp_pit64b_reset(data.ced->cd, MCHP_PIT64B_MR_CONT, true);

	return 0;
}

static int mchp_pit64b_clkevt_set_oneshot(struct clock_event_device *cedev)
{
	mchp_pit64b_reset(data.ced->cd, MCHP_PIT64B_MR_SMOD, true);

	return 0;
}

static int mchp_pit64b_clkevt_set_next_event(unsigned long evt,
					     struct clock_event_device *cedev)
{
	mchp_pit64b_set_period(data.ced->cd->base, evt);
	mchp_pit64b_write(data.ced->cd->base, MCHP_PIT64B_CR,
			  MCHP_PIT64B_CR_START);

	return 0;
}

static void mchp_pit64b_clkevt_suspend(struct clock_event_device *cedev)
{
	mchp_pit64b_write(data.ced->cd->base, MCHP_PIT64B_CR,
			  MCHP_PIT64B_CR_SWRST);
	if (data.ced->cd->gclk)
		clk_disable_unprepare(data.ced->cd->gclk);
	clk_disable_unprepare(data.ced->cd->pclk);
}

static void mchp_pit64b_clkevt_resume(struct clock_event_device *cedev)
{
	u32 mode = MCHP_PIT64B_MR_SMOD;

	clk_prepare_enable(data.ced->cd->pclk);
	if (data.ced->cd->gclk)
		clk_prepare_enable(data.ced->cd->gclk);

	if (clockevent_state_periodic(data.ced->clkevt))
		mode = MCHP_PIT64B_MR_CONT;

	mchp_pit64b_reset(data.ced->cd, mode, true);
}

static struct clock_event_device mchp_pit64b_clkevt = {
	.name = MCHP_PIT64B_NAME,
	.features = CLOCK_EVT_FEAT_ONESHOT | CLOCK_EVT_FEAT_PERIODIC,
	.rating = 150,
	.set_state_shutdown = mchp_pit64b_clkevt_shutdown,
	.set_state_periodic = mchp_pit64b_clkevt_set_periodic,
	.set_state_oneshot = mchp_pit64b_clkevt_set_oneshot,
	.set_next_event = mchp_pit64b_clkevt_set_next_event,
	.suspend = mchp_pit64b_clkevt_suspend,
	.resume = mchp_pit64b_clkevt_resume,
};

static irqreturn_t mchp_pit64b_interrupt(int irq, void *dev_id)
{
	struct mchp_pit64b_clkevt_data *irq_data = dev_id;

	if (data.ced != irq_data)
		return IRQ_NONE;

	if (mchp_pit64b_read(irq_data->cd->base, MCHP_PIT64B_ISR) &
	    MCHP_PIT64B_ISR_PERIOD) {
		irq_data->clkevt->event_handler(irq_data->clkevt);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int __init mchp_pit64b_pres_compute(u32 *pres, u32 clk_rate,
					   u32 max_rate)
{
	u32 tmp;

	for (*pres = 0; *pres < MCHP_PIT64B_PRES_MAX; (*pres)++) {
		tmp = clk_rate / (*pres + 1);
		if (tmp <= max_rate)
			break;
	}

	if (*pres == MCHP_PIT64B_PRES_MAX)
		return -EINVAL;

	return 0;
}

static int __init mchp_pit64b_pres_prepare(struct mchp_pit64b_common_data *cd,
					   unsigned long max_rate)
{
	unsigned long pclk_rate, diff = 0, best_diff = ULONG_MAX;
	long gclk_round = 0;
	u32 pres, best_pres = 0;
	int ret = 0;

	pclk_rate = clk_get_rate(cd->pclk);
	if (!pclk_rate)
		return -EINVAL;

	if (cd->gclk) {
		gclk_round = clk_round_rate(cd->gclk, max_rate);
		if (gclk_round < 0)
			goto pclk;

		if (pclk_rate / gclk_round < 3)
			goto pclk;

		ret = mchp_pit64b_pres_compute(&pres, gclk_round, max_rate);
		if (ret)
			best_diff = abs(gclk_round - max_rate);
		else
			best_diff = abs(gclk_round / (pres + 1) - max_rate);
		best_pres = pres;
	}

pclk:
	/* Check if requested rate could be obtained using PCLK. */
	ret = mchp_pit64b_pres_compute(&pres, pclk_rate, max_rate);
	if (ret)
		diff = abs(pclk_rate - max_rate);
	else
		diff = abs(pclk_rate / (pres + 1) - max_rate);

	if (best_diff > diff) {
		/* Use PCLK. */
		cd->gclk = NULL;
		best_pres = pres;
	} else {
		clk_set_rate(cd->gclk, gclk_round);
	}

	cd->pres = best_pres;

	pr_info("PIT64B: using clk=%s with prescaler %u, freq=%lu [Hz]\n",
		cd->gclk ? "gclk" : "pclk", cd->pres,
		cd->gclk ? gclk_round / (cd->pres + 1)
			 : pclk_rate / (cd->pres + 1));

	return 0;
}

static int __init mchp_pit64b_dt_init_clksrc(struct mchp_pit64b_common_data *cd)
{
	struct mchp_pit64b_clksrc_data *csd;
	unsigned long clk_rate;
	int ret;

	csd = kzalloc(sizeof(*csd), GFP_KERNEL);
	if (!csd)
		return -ENOMEM;

	csd->cd = cd;

	if (csd->cd->gclk)
		clk_rate = clk_get_rate(csd->cd->gclk);
	else
		clk_rate = clk_get_rate(csd->cd->pclk);

	clk_rate = clk_rate / (cd->pres + 1);
	csd->cd->cycles = ULLONG_MAX;
	mchp_pit64b_reset(csd->cd, MCHP_PIT64B_MR_CONT, false);

	data.csd = csd;

	csd->clksrc = &mchp_pit64b_clksrc;

	ret = clocksource_register_hz(csd->clksrc, clk_rate);
	if (ret) {
		pr_debug("clksrc: Failed to register PIT64B clocksource!\n");
		goto free;
	}

	sched_clock_register(mchp_sched_read_clk, 64, clk_rate);

	return 0;

free:
	kfree(csd);
	data.csd = NULL;

	return ret;
}

static int __init mchp_pit64b_dt_init_clkevt(struct mchp_pit64b_common_data *cd,
					     u32 irq)
{
	struct mchp_pit64b_clkevt_data *ced;
	unsigned long clk_rate;
	int ret;

	ced = kzalloc(sizeof(*ced), GFP_KERNEL);
	if (!ced)
		return -ENOMEM;

	ced->cd = cd;

	if (ced->cd->gclk)
		clk_rate = clk_get_rate(ced->cd->gclk);
	else
		clk_rate = clk_get_rate(ced->cd->pclk);

	clk_rate = clk_rate / (ced->cd->pres + 1);
	ced->cd->cycles = DIV_ROUND_CLOSEST(clk_rate, HZ);

	ret = request_irq(irq, mchp_pit64b_interrupt, IRQF_TIMER, "pit64b_tick",
			  ced);
	if (ret) {
		pr_debug("clkevt: Failed to setup PIT64B IRQ\n");
		goto free;
	}

	data.ced = ced;

	/* Set up and register clockevents. */
	ced->clkevt = &mchp_pit64b_clkevt;
	ced->clkevt->cpumask = cpumask_of(0);
	ced->clkevt->irq = irq;
	clockevents_config_and_register(ced->clkevt, clk_rate, 1, ULONG_MAX);

	return 0;

free:
	kfree(ced);
	data.ced = NULL;

	return ret;
}

static int __init mchp_pit64b_dt_init(struct device_node *node)
{
	struct mchp_pit64b_common_data *cd;
	u32 irq, freq = MCHP_PIT64B_DEF_FREQ;
	int ret;

	if (data.csd && data.ced)
		return -EBUSY;

	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	cd->pclk = of_clk_get_by_name(node, "pclk");
	if (IS_ERR(cd->pclk)) {
		ret = PTR_ERR(cd->pclk);
		goto free;
	}

	cd->gclk = of_clk_get_by_name(node, "gclk");
	if (IS_ERR(cd->gclk))
		cd->gclk = NULL;

	ret = of_property_read_u32(node, "clock-frequency", &freq);
	if (ret)
		pr_debug("PIT64B: failed to read clock frequency. Using default!\n");

	ret = mchp_pit64b_pres_prepare(cd, freq);
	if (ret)
		goto free;

	cd->base = of_iomap(node, 0);
	if (!cd->base) {
		pr_debug("%s: Could not map PIT64B address!\n",
			 MCHP_PIT64B_NAME);
		ret = -ENXIO;
		goto free;
	}

	ret = clk_prepare_enable(cd->pclk);
	if (ret)
		goto unmap;

	if (cd->gclk) {
		ret = clk_prepare_enable(cd->gclk);
		if (ret)
			goto pclk_unprepare;
	}

	if (!data.ced) {
		irq = irq_of_parse_and_map(node, 0);
		if (!irq) {
			pr_debug("%s: Failed to get PIT64B clockevent IRQ!\n",
				 MCHP_PIT64B_NAME);
			ret = -ENODEV;
			goto gclk_unprepare;
		}
		ret = mchp_pit64b_dt_init_clkevt(cd, irq);
		if (ret)
			goto irq_unmap;
	} else {
		ret = mchp_pit64b_dt_init_clksrc(cd);
		if (ret)
			goto gclk_unprepare;
	}

	return 0;

irq_unmap:
	irq_dispose_mapping(irq);
gclk_unprepare:
	if (cd->gclk)
		clk_disable_unprepare(cd->gclk);
pclk_unprepare:
	clk_disable_unprepare(cd->pclk);
unmap:
	iounmap(cd->base);
free:
	kfree(cd);

	return ret;
}

CLOCKSOURCE_OF_DECLARE(mchp_pit64b_clksrc, "microchip,sam9x60-pit64b",
		       mchp_pit64b_dt_init);
