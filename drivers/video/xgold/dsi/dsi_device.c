/*
 *******************************************************************************
 *
 *  Component: XGold MIPI DSI driver
 *
 *  Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *******************************************************************************
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/rockchip_fb.h>
#include <linux/reset.h>
#include <linux/cadiz.h>

#include "dsi_device.h"
#include "dsi_hwregs.h"
#include "dsi_dts.h"

static struct xgold_mipi_dsi_device *xgold_mipi_dsi;

#ifdef CONFIG_SONY_CADIZ
extern void cadiz_suspend(void);
extern void cadiz_resume(void);
extern void cadiz_reset(void);
extern int cadiz_init_polling(u16 reg, u8 value);
extern int cadiz_i2c_reg_write(u16 reg, u8 value);
extern int cadiz_init_boot1(void);
extern int cadiz_init_ipc_ibc(void);
extern int cadiz_init_boot2(void);
extern bool cadiz_support;
#endif

bool mipi_dsi_state = false;
EXPORT_SYMBOL(mipi_dsi_state);

extern bool vop_shutdown;

bool get_mipi_state(void)
{
	return mipi_dsi_state;
}
EXPORT_SYMBOL(get_mipi_state);

static int xgold_mipi_dsi_enable(void)
{
	struct xgold_mipi_dsi_device *mipi_dsi = xgold_mipi_dsi;
	struct dsi_display *display = &mipi_dsi->display;

	if (unlikely(!mipi_dsi) || mipi_dsi->sys_state)
		return 0;

	dsi_init(display);
	if (display->power_on)
		display->power_on(display);

	dsi_config(display, DIF_TX_DATA);

#ifdef CONFIG_SONY_CADIZ
	if (cadiz_support) {
		cadiz_resume();
		usleep_range(500, 500);
		cadiz_i2c_reg_write(0x0830, 0x00);
		cadiz_init_polling(0x0207,0x40);
	}
#endif

	if (display->lcd_reset)
		display->lcd_reset(display);

	if (display->panel_init)
		display->panel_init(display);

	if (display->sleep_out)
		display->sleep_out(display);

#ifdef CONFIG_SONY_CADIZ
	if (cadiz_support) {
		cadiz_init_polling(0x0404,0x38);
		cadiz_init_boot1();
		cadiz_init_ipc_ibc();
		cadiz_init_boot2();
		usleep_range(300, 300);
	}
#endif

	dsi_config(display, DIF_TX_PIXELS);
	mipi_dsi->sys_state = true;
	mipi_dsi_state = true;

	return 0;
}

static int xgold_mipi_dsi_disable(void)
{
	struct xgold_mipi_dsi_device *mipi_dsi = xgold_mipi_dsi;
	struct dsi_display *display = &mipi_dsi->display;

	if (unlikely(!mipi_dsi) || !mipi_dsi->sys_state)
		return 0;

	usleep_range(100000, 100000);

	dsi_config(display, DIF_TX_DATA);
	dsi_init(display);

#ifdef CONFIG_SONY_CADIZ
	if (cadiz_support) {
		cadiz_reset();
	}
#endif

	dsi_config(display, DIF_TX_DATA);
	if (display->sleep_in)
		display->sleep_in(display);

	dsi_stop(display);
	usleep_range(100000, 100000);

	if (vop_shutdown && display->reset_low)
		display->reset_low(display);

#ifdef CONFIG_SONY_CADIZ
	if (cadiz_support) {
		cadiz_suspend();
	}
#endif

	if (vop_shutdown && display->power_off)
		display->power_off(display);

#ifdef CONFIG_SONY_CADIZ
	if (cadiz_support) {
		cadiz_power_control(CADIZ, false);
	}
#endif

	mipi_dsi->sys_state = false;
	mipi_dsi_state = false;

	return 0;
}

static struct rockchip_fb_trsm_ops trsm_mipi_dsi_ops = {
	.enable = xgold_mipi_dsi_enable,
	.disable = xgold_mipi_dsi_disable,
};

static int xgold_mipi_dsi_probe(struct platform_device *pdev)
{
	struct xgold_mipi_dsi_device *mipi_dsi;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(&pdev->dev, "Don't find xgold mipi dsi device tree node.\n");
		return -EINVAL;
	}

	mipi_dsi = devm_kzalloc(&pdev->dev,
				sizeof(struct xgold_mipi_dsi_device),
				GFP_KERNEL);
	if (!mipi_dsi) {
		dev_err(&pdev->dev, "kzalloc xgold_mipi_dsi device failed\n");
		return -ENOMEM;
	}

	mipi_dsi->dev = &pdev->dev;
	rockchip_get_prmry_screen(&mipi_dsi->screen);
	if (mipi_dsi->screen.type != SCREEN_MIPI) {
		dev_err(&pdev->dev, "screen is not SCREEN MIPI!\n");
		devm_kfree(&pdev->dev, mipi_dsi);
		mipi_dsi = NULL;
		return -EINVAL;
	}

	platform_set_drvdata(pdev, mipi_dsi);
	dev_set_name(mipi_dsi->dev, "xgold-mipi_dsi");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "mipi_dsi_phy");
	mipi_dsi->display.regbase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mipi_dsi->display.regbase)) {
		dev_err(&pdev->dev, "ioremap xgold mipi_dsi_phy reg failed\n");
		return PTR_ERR(mipi_dsi->display.regbase);
	}

	mipi_dsi->display.irq.err = platform_get_irq(pdev, 0);
	if (mipi_dsi->display.irq.err < 0) {
		dev_err(&pdev->dev, "Cannot find ERR IRQ for XGold MIPI DSI\n");
		return mipi_dsi->display.irq.err;
	}

	xgold_mipi_dsi = mipi_dsi;
	rockchip_fb_trsm_ops_register(&trsm_mipi_dsi_ops, SCREEN_MIPI);
	if (support_loader_display()) {
		mipi_dsi->sys_state = true;
		mipi_dsi_state = true;
	}

	dev_info(&pdev->dev, "XGold MIPI DSI driver probe success\n");
	dsi_of_parse_display(pdev, mipi_dsi);
	dsi_probe(&mipi_dsi->display);
	dsi_irq_probe(&mipi_dsi->display);

	return 0;
}

static int xgold_mipi_dsi_remove(struct platform_device *pdev)
{
	dsi_irq_remove(&xgold_mipi_dsi->display);
	devm_kfree(&pdev->dev, xgold_mipi_dsi);
	xgold_mipi_dsi = NULL;

	return 0;
}

static void xgold_mipi_dsi_shutdown(struct platform_device *pdev)
{
}

#ifdef CONFIG_OF
static const struct of_device_id xgold_mipi_dsi_dt_ids[] = {
	{.compatible = "intel,xgold-mipi_dsi",},
	{}
};
#endif

struct platform_driver xgold_mipi_dsi_driver = {
	.probe = xgold_mipi_dsi_probe,
	.remove = xgold_mipi_dsi_remove,
	.shutdown = xgold_mipi_dsi_shutdown,
	.driver = {
		.name = "xgold-mipi_dsi",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(xgold_mipi_dsi_dt_ids),
#endif
	},
};

