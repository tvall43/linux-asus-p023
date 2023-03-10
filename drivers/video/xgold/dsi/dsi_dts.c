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

#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/rockchip_fb.h>
#include <linux/reset.h>

#include "dsi_device.h"
#include "dsi_hwregs.h"
#include "dsi_dts.h"

#define PROP_DISPLAY            "intel,display"
#define PROP_SCREEN             "intel,screen"

#define PROP_DISPLAY_DCCLK      "intel,display-dc-clkrate"
#define PROP_DISPLAY_RAMLESS    "intel,display-ramless"
#define PROP_DISPLAY_FPS        "intel,display-fps"
#define PROP_DISPLAY_DATARATE   "intel,display-dsi-datarate"
#define PROP_DISPLAY_LANES      "intel,display-if-nblanes"
#define PROP_DISPLAY_PREINIT    "intel,display-preinit"
#define PROP_DISPLAY_VIDEOMODE  "intel,display-vid-mode"
#define PROP_DISPLAY_VIDEOID    "intel,display-vid-id"
#define PROP_DISPLAY_EOT        "intel,display-eot"
#define PROP_DISPLAY_GATE       "intel,display-gate"

#define PROP_DISPLAY_GPIORST    "intel,display-gpio-reset"
#define PROP_DISPLAY_GPIOVH     "intel,display-gpio-vhigh"
#define PROP_DISPLAY_GPIOVL     "intel,display-gpio-vlow"

#define GPIO_LIST_POWER_ON      "gpio-power-on"
#define GPIO_LIST_LCD_RESET     "gpio-lcd-reset"
#define GPIO_LIST_RESET_LOW     "gpio-reset-low"
#define GPIO_LIST_POWER_OFF     "gpio-power-off"

#define PROP_DISPLAY_GPIOTYPE   "intel,gpio-type"
#define PROP_DISPLAY_GPIOVALUE  "intel,gpio-value-delay"

#define CMD_LIST_INIT           "cmd-init"
#define CMD_LIST_UPDATE         "cmd-update"
#define CMD_LIST_SLEEP_IN       "cmd-sleep-in"
#define CMD_LIST_SLEEP_OUT      "cmd-sleep-out"

#define PROP_DISPLAY_CMDTYPE    "intel,cmd-type"
#define PROP_DISPLAY_CMDDATA    "intel,cmd-data"
#define PROP_DISPLAY_CMDDELAY   "intel,cmd-delay"
#define PROP_DISPLAY_CMDLP      "intel,cmd-lp"

#define PORCH_SYNC_MAX 0xFF

static struct of_device_id display_of_match[] = {
	{ .compatible = PROP_DISPLAY, },
	{ },
};

static struct of_device_id screen_of_match[] = {
	{ .compatible = PROP_SCREEN, },
	{ },
};

#ifdef CONFIG_SONY_CADIZ
#include <linux/cadiz.h>
extern bool cadiz_support;
#endif


static int dsi_of_parse_display_cmd(struct platform_device *pdev,
				    struct device_node *n,
				    struct display_msg *cmd)
{
	int ret = 0, i;
	u32 val;
	const __be32 *p;
	struct property *prop;

	cmd->flags = 0;
	cmd->length = 0;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		cmd->length++;
	};

	/* allocate data array if needed */
	if (cmd->length > 0) {
		cmd->datas = devm_kzalloc(&pdev->dev,
				cmd->length*sizeof(u8), GFP_KERNEL);
		if (!cmd->datas) {
			pr_err("%s: Can't alloc array for %s length %dbytes\n",
			       __func__, n->name, cmd->length);
			return -ENOMEM;
		}
	}

	cmd->name = n->name;

	/* populate header+data */
	i = 0;
	of_property_for_each_u32(n, PROP_DISPLAY_CMDDATA, prop, p, val) {
		if (cmd->datas)
			cmd->datas[i] = val;
		i++;
	}

	ret = of_property_read_u32(n, PROP_DISPLAY_CMDTYPE, &val);
	if (ret)
		cmd->type = 0;
	else
		cmd->type = val;

	ret = of_property_read_u32(n, PROP_DISPLAY_CMDDELAY, &cmd->delay);
	if (ret)
		cmd->delay = 0;

	ret = of_property_read_u32(n, PROP_DISPLAY_CMDLP, &val);
	if (!ret && val)
		cmd->flags |= LCD_MSG_LP;

	return 0;
}

static int dsi_of_parse_display_msglist(struct platform_device *pdev,
					struct device_node *n,
					struct display_msg **msglist)
{
	struct device_node *child;
	struct display_msg *msg;

	/* allocate cmd list */
	*msglist = devm_kzalloc(&pdev->dev, sizeof(struct display_msg),
				GFP_KERNEL);
	if (!*msglist) {
		pr_err("%s: Can't alloc commands table\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&(*msglist)->list);
	for_each_child_of_node(n, child) {
		msg = (struct display_msg *) devm_kzalloc(&pdev->dev,
			sizeof(struct display_msg), GFP_KERNEL);
		if (!msg) {
			pr_err("%s: Allocation of display msg failed\n",
			       __func__);
			return -EINVAL;
		}

		if (!dsi_of_parse_display_cmd(pdev, child, msg))
			list_add_tail(&msg->list, &(*msglist)->list);
		else
			devm_kfree(&pdev->dev, msg);
	}

	return 0;
}

static int dsi_of_parse_display_gpio(struct platform_device *pdev,
				     struct device_node *n,
				     struct display_gpio *gpio)
{
	const char *string;
	int array[2];
	int ret;

	gpio->name = n->name;
	ret = of_property_read_string(n, PROP_DISPLAY_GPIOTYPE, &string);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOTYPE);
		return ret;
	} else if (!strcmp("vhigh", string)) {
		gpio->type = DSI_GPIO_VHIGH;
	} else if (!strcmp("vlow", string)) {
		gpio->type = DSI_GPIO_VLOW;
	} else if (!strcmp("reset", string)) {
		gpio->type = DSI_GPIO_RESET;
	}

	ret = of_property_read_u32_array(n, PROP_DISPLAY_GPIOVALUE, array, 2);
	if (ret) {
		pr_err("%s: Get %s failed\n", __func__, PROP_DISPLAY_GPIOVALUE);
		return ret;
	}

	gpio->value = array[0];
	gpio->delay = array[1];

	return 0;
}

static int dsi_of_parse_display_gpiolist(struct platform_device *pdev,
					 struct device_node *n,
					 struct display_gpio **gpiolist)
{
	struct device_node *child;
	struct display_gpio *gpio;

	*gpiolist = devm_kzalloc(&pdev->dev, sizeof(struct display_gpio),
				 GFP_KERNEL);
	if (!*gpiolist) {
		pr_err("%s: Can't alloc gpio table\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&(*gpiolist)->list);
	for_each_child_of_node(n, child) {
		gpio = (struct display_gpio *) devm_kzalloc(&pdev->dev,
			sizeof(struct display_gpio), GFP_KERNEL);
		if (!gpio) {
			pr_err("%s: Allocation of display gpio failed\n",
			       __func__);
			return -EINVAL;
		}

		if (!dsi_of_parse_display_gpio(pdev, child, gpio))
			list_add_tail(&gpio->list, &(*gpiolist)->list);
		else
			devm_kfree(&pdev->dev, gpio);
	}

	return 0;
}

static int dsi_of_parse_gpio(struct platform_device *pdev,
			     struct dsi_display *display)
{
	struct device_node *screen_dev_n;
	enum of_gpio_flags gpio_flags;
	unsigned long flags;
	int ret;

	screen_dev_n = of_find_matching_node(NULL, screen_of_match);
	if (!screen_dev_n) {
		pr_err("%s: Can't find screen matching node\n", __func__);
		return -EINVAL;
	}

	display->gpio_vhigh = of_get_named_gpio_flags(screen_dev_n,
		PROP_DISPLAY_GPIOVH, 0, &gpio_flags);
	if (gpio_is_valid(display->gpio_vhigh)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(display->gpio_vhigh, flags,
				       "disp_vhigh");
		if (ret) {
			pr_err("%s: request display high power gpio fail: %d\n",
			       __func__, ret);
			display->gpio_vhigh = 0;
		}
	} else {
		display->gpio_vhigh = 0;
	}

	display->gpio_vlow = of_get_named_gpio_flags(screen_dev_n,
			PROP_DISPLAY_GPIOVL, 0, &gpio_flags);
	if (gpio_is_valid(display->gpio_vlow)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(display->gpio_vlow, flags, "disp_vlow");
		if (ret) {
			pr_err("%s: request display low power gpio fail: %d\n",
			       __func__, ret);
			display->gpio_vlow = 0;
		}
	} else {
		display->gpio_vlow = 0;
	}

	display->gpio_reset = of_get_named_gpio_flags(screen_dev_n,
			PROP_DISPLAY_GPIORST, 0, &gpio_flags);
	if (gpio_is_valid(display->gpio_reset)) {
		if (support_loader_display()) {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_LOW;
			else
				flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (gpio_flags & OF_GPIO_ACTIVE_LOW)
				flags = GPIOF_OUT_INIT_HIGH;
			else
				flags = GPIOF_OUT_INIT_LOW;
		}
		ret = gpio_request_one(display->gpio_reset, flags, "disp_rst");
		if (ret) {
			pr_err("%s: request display reset gpio fail: %d\n",
			       __func__, ret);
			display->gpio_reset = 0;
		}
	} else {
		display->gpio_reset = 0;
	}

	return 0;
}

static void
dsi_of_parse_display_timing(struct xgold_mipi_dsi_device *mipi_dsi)
{
	struct rockchip_screen *screen = &mipi_dsi->screen;
	struct dsi_display *display = &mipi_dsi->display;

	rockchip_get_prmry_screen(screen);
	display->xres = screen->mode.xres;
	display->yres = screen->mode.yres;
	if (screen->face == OUT_P565) {
		display->bpp = 16;
		display->dif.dsi.video_pixel = DSI_PIX_BIT16P;
	} else if (screen->face == OUT_P666) {
		display->bpp = 18;
		display->dif.dsi.video_pixel = DSI_PIX_BIT18P;
	} else {
		display->bpp = 24;
		display->dif.dsi.video_pixel = DSI_PIX_BIT24P;
	}

	display->dif.dsi.hfp = PIXELS_TO_BYTES(screen->mode.right_margin,
					       display->bpp);
	if (display->dif.dsi.hfp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HFP = %d, MAX HFP is %d\n", __func__,
			screen->mode.right_margin,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hfp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.hbp = PIXELS_TO_BYTES(screen->mode.left_margin,
					       display->bpp);
	if (display->dif.dsi.hbp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HBP = %d, MAX HBP is %d\n", __func__,
			screen->mode.left_margin,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hbp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.hsa = PIXELS_TO_BYTES(screen->mode.hsync_len,
					       display->bpp);
	if (display->dif.dsi.hsa > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! HSA = %d, MAX HSA is %d\n", __func__,
			screen->mode.hsync_len,
			BYTES_TO_PIXELS(PORCH_SYNC_MAX, display->bpp));
		display->dif.dsi.hsa = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vfp = screen->mode.lower_margin;
	if (display->dif.dsi.vfp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VFP = %d, MAX VFP is %d\n", __func__,
			screen->mode.lower_margin, PORCH_SYNC_MAX);
		display->dif.dsi.vfp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vbp = screen->mode.upper_margin;
	if (display->dif.dsi.vbp > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VBP = %d, MAX VBP is %d\n", __func__,
			screen->mode.upper_margin, PORCH_SYNC_MAX);
		display->dif.dsi.vbp = PORCH_SYNC_MAX;
	}

	display->dif.dsi.vsa = screen->mode.vsync_len;
	if (display->dif.dsi.vsa > PORCH_SYNC_MAX) {
		pr_info("%s: Warning! VSA = %d, MAX VSA is %d\n", __func__,
			screen->mode.vsync_len, PORCH_SYNC_MAX);
		display->dif.dsi.vsa = PORCH_SYNC_MAX;
	}

	display->dif.dsi.hfp_lp = 0;
	display->dif.dsi.hbp_lp = 0;
	display->dif.dsi.hsa_lp = 0;
}

int dsi_of_parse_display(struct platform_device *pdev,
			 struct xgold_mipi_dsi_device *mipi_dsi)
{
	int value, ret = 0;
	const char *string;
	struct device_node *display_dev_n, *child;
	struct dsi_display *display = &mipi_dsi->display;
	int index = 0;
	printk("[DISPLAY] %s: Enter\n",__func__);

	dsi_of_parse_gpio(pdev, display);
	dsi_of_parse_display_timing(mipi_dsi);
	display->dsi_reset = devm_reset_control_get(&pdev->dev, "dsi");
	if (IS_ERR(display->dsi_reset)) {
		pr_err("%s: get dsi reset control failed\n", __func__);
		display->dsi_reset = NULL;
	}

	for_each_matching_node(display_dev_n, display_of_match) {
		if (mipi_dsi->screen.index < 0 ||
		    mipi_dsi->screen.index == index++)
			break;
	}

	if (!display_dev_n) {
		pr_err("%s: Can't find display matching node\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_u32(display_dev_n, PROP_DISPLAY_DCCLK,
		&display->dif.dsi.dc_clk_rate)) {
		pr_err("%s: Can't get DC clock rate\n", __func__);
		return -EINVAL;
	}

	/* DSI_CFG default value */
	display->dif.dsi.dsi_cfg_reg = BITFLDS(EXR_DSI_CFG_VSYNC, 1) |
		BITFLDS(EXR_DSI_CFG_PSYNC, 1);

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_RAMLESS, &value);
	if (ret || value)
		display->dif.dsi.mode = DSI_VIDEO;
	else
		display->dif.dsi.mode = DSI_CMD;

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_FPS,
				   &display->fps);
	if (ret)
		display->fps = 60;

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_DATARATE,
				   &display->dif.dsi.bitrate);
	if (ret) {
		display->dif.dsi.bitrate = 0;
		printk("[DISPLAY] %s: Auto calculate MIPI dsi datarate\n", __func__);
	}

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_LANES,
				   &display->dif.dsi.nblanes);
	if (ret)
		display->dif.dsi.nblanes = 4;

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_EOT, &value);
	if (ret || value)
		display->dif.dsi.dsi_cfg_reg |= BITFLDS(EXR_DSI_CFG_EOT, 1);

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_GATE, &value);
	if (ret || value)
		display->dif.dsi.dsi_cfg_reg |= BITFLDS(EXR_DSI_CFG_GATE, 1);

#ifdef CONFIG_SONY_CADIZ
	if(!cadiz_support){
		display->dif.dsi.dsi_cfg_reg |= BITFLDS(EXR_DSI_CFG_GATE, 1);
	}
	printk("[DISPLAY] %s: display->dif.dsi.dsi_cfg_reg = 0x%x\n",
		__func__, display->dif.dsi.dsi_cfg_reg);
#endif

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_PREINIT,
				   &display->dif.dsi.display_preinit);
	if (ret)
		display->dif.dsi.display_preinit = 0;

	ret = of_property_read_string(display_dev_n, PROP_DISPLAY_VIDEOMODE,
				      &string);
	if (ret) {
		display->dif.dsi.video_mode = DSI_BURST;
	} else if (!strcmp("active", string)) {
		display->dif.dsi.video_mode = DSI_ACTIVE;
	} else if (!strcmp("pulses", string)) {
		display->dif.dsi.video_mode = DSI_PULSES;
	} else if (!strcmp("events", string)) {
		display->dif.dsi.video_mode = DSI_EVENTS;
	} else if (!strcmp("burst", string)) {
		display->dif.dsi.video_mode = DSI_BURST;
	} else {
		display->dif.dsi.video_mode = DSI_BURST;
		pr_info("%s: Unknown dsi video mode type %s\n", __func__,
			string);
	}

	ret = of_property_read_u32(display_dev_n, PROP_DISPLAY_VIDEOID,
				   &display->dif.dsi.id);
	if (ret)
		display->dif.dsi.id = 0;

	for_each_child_of_node(display_dev_n, child) {
		if (!strcmp(child->name, CMD_LIST_INIT)) {
			ret = dsi_of_parse_display_msglist(pdev, child,
				&display->msgs_init);
		} else if (!strcmp(child->name, CMD_LIST_UPDATE)) {
			ret = dsi_of_parse_display_msglist(pdev, child,
				&display->msgs_update);
		} else if (!strcmp(child->name, GPIO_LIST_POWER_ON)) {
			ret = dsi_of_parse_display_gpiolist(pdev, child,
				&display->gpios_power_on);
		} else if (!strcmp(child->name, GPIO_LIST_LCD_RESET)) {
			ret = dsi_of_parse_display_gpiolist(pdev, child,
				&display->gpios_lcd_reset);
		} else if (!strcmp(child->name, GPIO_LIST_RESET_LOW)) {
			ret = dsi_of_parse_display_gpiolist(pdev, child,
				&display->gpios_reset_low);
		} else if (!strcmp(child->name, GPIO_LIST_POWER_OFF)) {
			ret = dsi_of_parse_display_gpiolist(pdev, child,
				&display->gpios_power_off);
		} else if (!strcmp(child->name, CMD_LIST_SLEEP_IN)) {
			ret = dsi_of_parse_display_msglist(pdev, child,
				&display->msgs_sleep_in);
		} else if (!strcmp(child->name, CMD_LIST_SLEEP_OUT)) {
			ret = dsi_of_parse_display_msglist(pdev, child,
				&display->msgs_sleep_out);
		} else {
			pr_info("%s: In node %s, unexpected child %s !\n",
				__func__, display_dev_n->name, child->name);
		}

		if (ret) {
			pr_info("%s: Node %s parsing failed %d\n", __func__,
				child->name, ret);
		}
	};

	return 0;
}

