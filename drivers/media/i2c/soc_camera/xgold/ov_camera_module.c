/*
 * ov_camera_module.c
 *
 * Generic omnivision sensor driver
 *
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/delay.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-controls_intel.h>
#include <media/videobuf-core.h>
#include <linux/slab.h>
#include <linux/gcd.h>
#include <media/v4l2-controls_intel.h>

#include "ov_camera_module.h"


static struct ov_camera_module *to_ov_camera_module(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov_camera_module, sd);
}

/* ======================================================================== */

static void ov_camera_module_reset(
	struct ov_camera_module *cam_mod)
{
	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");

	cam_mod->active_config = NULL;
	cam_mod->update_config = true;
	cam_mod->frm_fmt_valid = false;
	cam_mod->frm_intrvl_valid = false;
	cam_mod->exp_config.auto_exp = false;
	cam_mod->exp_config.auto_gain = false;
	cam_mod->wb_config.auto_wb = false;
	cam_mod->hflip = false;
	cam_mod->vflip = false;
	cam_mod->auto_adjust_fps = true;
	cam_mod->rotation = 0;
	cam_mod->ctrl_updt = 0;
	cam_mod->state = OV_CAMERA_MODULE_POWER_OFF;
	cam_mod->state_before_suspend = OV_CAMERA_MODULE_POWER_OFF;
}

/* ======================================================================== */

static void ov_camera_module_set_active_config(
	struct ov_camera_module *cam_mod,
	struct ov_camera_module_config *new_config)
{

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");

	if (IS_ERR_OR_NULL(new_config)) {
		cam_mod->active_config = new_config;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"no active config\n");
	} else {
		cam_mod->ctrl_updt &= OV_CAMERA_MODULE_CTRL_UPDT_AUTO_EXP |
			OV_CAMERA_MODULE_CTRL_UPDT_AUTO_GAIN |
			OV_CAMERA_MODULE_CTRL_UPDT_AUTO_WB;
		if (new_config->auto_exp_enabled !=
			cam_mod->exp_config.auto_exp) {
			cam_mod->ctrl_updt |=
				OV_CAMERA_MODULE_CTRL_UPDT_AUTO_EXP;
			cam_mod->exp_config.auto_exp =
				new_config->auto_exp_enabled;
		}
		if (new_config->auto_gain_enabled !=
			cam_mod->exp_config.auto_gain) {
			cam_mod->ctrl_updt |=
				OV_CAMERA_MODULE_CTRL_UPDT_AUTO_GAIN;
			cam_mod->exp_config.auto_gain =
				new_config->auto_gain_enabled;
		}
		if (new_config->auto_wb_enabled !=
			cam_mod->wb_config.auto_wb) {
			cam_mod->ctrl_updt |=
				OV_CAMERA_MODULE_CTRL_UPDT_AUTO_WB;
			cam_mod->wb_config.auto_wb =
				new_config->auto_wb_enabled;
		}
		if (new_config != cam_mod->active_config) {
			cam_mod->update_config = true;
			cam_mod->active_config = new_config;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
				"activating config '%s'\n",
				new_config->name);
		}
	}
}

/* ======================================================================== */

static struct ov_camera_module_config *ov_camera_module_find_config(
	struct ov_camera_module *cam_mod,
	struct v4l2_mbus_framefmt *fmt,
	struct v4l2_subdev_frame_interval *frm_intrvl)
{
	u32 i;
	unsigned long gcdiv;
	struct v4l2_subdev_frame_interval norm_interval;

	if (!IS_ERR_OR_NULL(fmt))
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"%dx%d, fmt code 0x%04x\n",
			fmt->width, fmt->height, fmt->code);

	if (!IS_ERR_OR_NULL(frm_intrvl))
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"frame interval %d/%d\n",
			frm_intrvl->interval.numerator,
			frm_intrvl->interval.denominator);

	for (i = 0; i < cam_mod->custom.num_configs; i++) {
		if (!IS_ERR_OR_NULL(frm_intrvl)) {
			gcdiv = gcd(cam_mod->custom.configs[i].
				frm_intrvl.interval.numerator,
				cam_mod->custom.configs[i].
					frm_intrvl.interval.denominator);
			norm_interval.interval.numerator =
				cam_mod->custom.configs[i].
					frm_intrvl.interval.numerator / gcdiv;
			norm_interval.interval.denominator =
				cam_mod->custom.configs[i].
				frm_intrvl.interval.denominator / gcdiv;
			if ((frm_intrvl->interval.numerator !=
				norm_interval.interval.numerator) ||
				(frm_intrvl->interval.denominator !=
				norm_interval.interval.denominator))
				continue;
		}
		if (!IS_ERR_OR_NULL(fmt)) {
			if ((cam_mod->custom.configs[i].frm_fmt.width !=
				fmt->width) ||
				(cam_mod->custom.configs[i].frm_fmt.height !=
				fmt->height) ||
				(cam_mod->custom.configs[i].frm_fmt.code !=
				fmt->code)) {
				continue;
			}
		}
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"found matching config %s\n",
			cam_mod->custom.configs[i].name);
		return &cam_mod->custom.configs[i];
	}
	pltfrm_camera_module_pr_debug(&cam_mod->sd,
		"no matching config found\n");

	return ERR_PTR(-EINVAL);
}

/* ======================================================================== */

static int ov_camera_module_write_config(
	struct ov_camera_module *cam_mod)
{
	int ret = 0;

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");

	if (IS_ERR_OR_NULL(cam_mod->active_config)) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"no active sensor configuration");
		ret = -EFAULT;
		goto err;
	}

	ret = pltfrm_camera_module_write_reglist(&cam_mod->sd,
		cam_mod->active_config->reg_table,
		cam_mod->active_config->reg_table_num_entries);
	if (IS_ERR_VALUE(ret))
		goto err;
	ret = pltfrm_camera_module_patch_config(&cam_mod->sd,
		&cam_mod->frm_fmt,
		&cam_mod->frm_intrvl);
	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	pltfrm_camera_module_pr_err(&cam_mod->sd,
		"failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

int ov_camera_module_try_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ov_camera_module *cam_mod = to_ov_camera_module(sd);

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "%dx%d, fmt code 0x%04x\n",
		fmt->width, fmt->height, fmt->code);

	if (IS_ERR_OR_NULL(ov_camera_module_find_config(cam_mod, fmt, NULL))) {
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"format not supported\n");
		return -EINVAL;
	}
	pltfrm_camera_module_pr_debug(&cam_mod->sd, "format supported\n");

	return 0;
}

/* ======================================================================== */

int ov_camera_module_s_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);
	int ret = 0;

	pltfrm_camera_module_pr_info(&cam_mod->sd, "%dx%d, fmt code 0x%04x\n",
		fmt->width, fmt->height, fmt->code);

	if (IS_ERR_OR_NULL(ov_camera_module_find_config(cam_mod, fmt, NULL))) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"format %dx%d, code 0x%04x, not supported\n",
			fmt->width, fmt->height, fmt->code);
		ret = -EINVAL;
		goto err;
	}
	cam_mod->frm_fmt_valid = true;
	cam_mod->frm_fmt = *fmt;
	if (cam_mod->frm_intrvl_valid) {
		ov_camera_module_set_active_config(cam_mod,
			ov_camera_module_find_config(cam_mod,
				fmt, &cam_mod->frm_intrvl));
	}
	return 0;
err:
	pltfrm_camera_module_pr_err(&cam_mod->sd,
		"failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

int ov_camera_module_g_fmt(struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt)
{
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");

	if (cam_mod->active_config) {
		fmt->code = cam_mod->active_config->frm_fmt.code;
		fmt->width = cam_mod->active_config->frm_fmt.width;
		fmt->height = cam_mod->active_config->frm_fmt.height;
		return 0;
	}

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "no active config\n");

	return -1;
}

/* ======================================================================== */

int ov_camera_module_s_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *interval)
{
	struct ov_camera_module *cam_mod = to_ov_camera_module(sd);
	unsigned long gcdiv;
	struct v4l2_subdev_frame_interval norm_interval;
	int ret = 0;

	if ((0 == interval->interval.denominator) ||
		(0 == interval->interval.numerator)) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"invalid frame interval %d/%d\n",
			interval->interval.numerator,
			interval->interval.denominator);
		ret = -EINVAL;
		goto err;
	}

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "%d/%d (%dfps)\n",
		interval->interval.numerator, interval->interval.denominator,
		(interval->interval.denominator +
		(interval->interval.numerator >> 1)) /
		interval->interval.numerator);

	/* normalize interval */
	gcdiv = gcd(interval->interval.numerator,
		interval->interval.denominator);
	norm_interval.interval.numerator =
		interval->interval.numerator / gcdiv;
	norm_interval.interval.denominator =
		interval->interval.denominator / gcdiv;

	if (IS_ERR_OR_NULL(ov_camera_module_find_config(cam_mod,
			NULL, &norm_interval))) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"frame interval %d/%d not supported\n",
			interval->interval.numerator,
			interval->interval.denominator);
		ret = -EINVAL;
		goto err;
	}
	cam_mod->frm_intrvl_valid = true;
	cam_mod->frm_intrvl = norm_interval;
	if (cam_mod->frm_fmt_valid) {
		ov_camera_module_set_active_config(cam_mod,
			ov_camera_module_find_config(cam_mod,
				&cam_mod->frm_fmt, interval));
	}
	return 0;
err:
	pltfrm_camera_module_pr_err(&cam_mod->sd,
		"failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

int ov_camera_module_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "%d\n", enable);

	if (enable) {
		if (cam_mod->state == OV_CAMERA_MODULE_STREAMING)
			return 0;
		if (IS_ERR_OR_NULL(cam_mod->active_config)) {
			pltfrm_camera_module_pr_err(&cam_mod->sd,
				"no active sensor configuration, cannot start streaming\n");
			ret = -EFAULT;
			goto err;
		}
		if (cam_mod->state != OV_CAMERA_MODULE_SW_STANDBY) {
			pltfrm_camera_module_pr_err(&cam_mod->sd,
				"sensor is not powered on (in state %d), cannot start streaming\n",
				cam_mod->state);
			ret = -EINVAL;
			goto err;
		}
		if (cam_mod->update_config)
			ret = ov_camera_module_write_config(cam_mod);
			if (IS_ERR_VALUE(ret))
				goto err;
		ret = cam_mod->custom.start_streaming(cam_mod);
		if (IS_ERR_VALUE(ret))
			goto err;
		cam_mod->update_config = false;
		cam_mod->ctrl_updt = 0;
		mdelay(cam_mod->custom.power_up_delays_ms[2]);
		cam_mod->state = OV_CAMERA_MODULE_STREAMING;
	} else {
		int pclk;
		int wait_ms;
		struct ov_camera_module_timings timings;
		if (cam_mod->state != OV_CAMERA_MODULE_STREAMING)
			return 0;
		ret = cam_mod->custom.stop_streaming(cam_mod);
		if (IS_ERR_VALUE(ret))
			goto err;

		ret = ov_camera_module_ioctl(sd,
					INTEL_VIDIOC_SENSOR_MODE_DATA,
					&timings);

		cam_mod->state = OV_CAMERA_MODULE_SW_STANDBY;

		if (IS_ERR_VALUE(ret))
			goto err;

		pclk = timings.vt_pix_clk_freq_hz / 1000;

		if (!pclk)
			goto err;

		wait_ms =
			(timings.line_length_pck *
			timings.frame_length_lines) /
			pclk;

		/* wait for a frame period to make sure that there is
			no pending frame left. */

		mdelay(wait_ms + 1);
	}

	cam_mod->state_before_suspend = cam_mod->state;

	return 0;
err:
	pltfrm_camera_module_pr_err(&cam_mod->sd,
		"failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

int ov_camera_module_s_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "%d\n", on);

	if (on) {
		if (OV_CAMERA_MODULE_POWER_OFF == cam_mod->state) {
			ret = pltfrm_camera_module_s_power(&cam_mod->sd, 1);
			if (!IS_ERR_VALUE(ret)) {
				mdelay(cam_mod->custom.power_up_delays_ms[0]);
				cam_mod->state = OV_CAMERA_MODULE_HW_STANDBY;
			}
		}
		if (OV_CAMERA_MODULE_HW_STANDBY == cam_mod->state) {
			msleep(10);
			ret = pltfrm_camera_module_set_pin_state(&cam_mod->sd,
				PLTFRM_CAMERA_MODULE_PIN_DVDD,
				PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
			msleep(10);
			ret = pltfrm_camera_module_set_pin_state(&cam_mod->sd,
				PLTFRM_CAMERA_MODULE_PIN_RESET,
				PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);
			usleep_range(1000, 1500);
			ret = pltfrm_camera_module_set_pin_state(&cam_mod->sd,
				PLTFRM_CAMERA_MODULE_PIN_PD,
				PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);
			msleep(70); //ASUS_BSP+++
			if (!IS_ERR_VALUE(ret)) {
				mdelay(cam_mod->custom.power_up_delays_ms[1]);
				cam_mod->state = OV_CAMERA_MODULE_SW_STANDBY;
				if (!IS_ERR_OR_NULL(cam_mod->custom.
					check_camera_id)) {
					ret = cam_mod->custom.
						check_camera_id(cam_mod);
					if (IS_ERR_VALUE(ret)) {
						pltfrm_camera_module_pr_err(
							&cam_mod->sd,
							"camera check ID failed, powering off sensor\n");
						(void)ov_camera_module_s_power(
							sd, 0);
						goto err;
					}
				}
			}
		}
	} else {
		if (OV_CAMERA_MODULE_STREAMING == cam_mod->state) {
			ret = ov_camera_module_s_stream(sd, 0);
			if (!IS_ERR_VALUE(ret))
				cam_mod->state = OV_CAMERA_MODULE_SW_STANDBY;
		}
		if (OV_CAMERA_MODULE_SW_STANDBY == cam_mod->state) {
			msleep(30); //ASUS_BSP+++
			ret = pltfrm_camera_module_set_pin_state(
				&cam_mod->sd,
				PLTFRM_CAMERA_MODULE_PIN_PD,
				PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
			msleep(20);
			ret = pltfrm_camera_module_set_pin_state(
				&cam_mod->sd,
				PLTFRM_CAMERA_MODULE_PIN_RESET,
				PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
			msleep(20);
			ret = pltfrm_camera_module_set_pin_state(&cam_mod->sd,
				PLTFRM_CAMERA_MODULE_PIN_DVDD,
				PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);
			msleep(20);
			if (!IS_ERR_VALUE(ret))
				cam_mod->state = OV_CAMERA_MODULE_HW_STANDBY;
		}
		if (OV_CAMERA_MODULE_HW_STANDBY == cam_mod->state) {
			ret = pltfrm_camera_module_s_power(&cam_mod->sd, 0);
			if (!IS_ERR_VALUE(ret)) {
				cam_mod->state = OV_CAMERA_MODULE_POWER_OFF;
				ov_camera_module_reset(cam_mod);
			}
		}
	}

	cam_mod->state_before_suspend = cam_mod->state;

	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"%s failed, camera left in state %d\n",
			on ? "on" : "off", cam_mod->state);
		goto err;
	} else
		pltfrm_camera_module_pr_info(&cam_mod->sd,
			"camera powered %s\n", on ? "on" : "off");

	return 0;
err:
	pltfrm_camera_module_pr_err(&cam_mod->sd,
		"failed with error %d\n", ret);
	return ret;
}

/* ======================================================================== */

int ov_camera_module_g_ctrl(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct ov_camera_module *cam_mod = to_ov_camera_module(sd);
	int ret;

	pltfrm_camera_module_pr_debug(&cam_mod->sd, " id 0x%x\n", ctrl->id);

	if (ctrl->id == V4L2_CID_FLASH_LED_MODE) {
		ctrl->value = cam_mod->exp_config.flash_mode;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_FLASH_LED_MODE %d\n",
			ctrl->value);
		return 0;
	}

	if (IS_ERR_OR_NULL(cam_mod->active_config)) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"no active configuration\n");
		return -EFAULT;
	}

	if (ctrl->id == INTEL_V4L2_CID_VBLANKING) {
		ctrl->value = cam_mod->active_config->v_blanking_time_us;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"INTEL_V4L2_CID_VBLANKING %d\n",
			ctrl->value);
		return 0;
	}

	if ((cam_mod->state != OV_CAMERA_MODULE_SW_STANDBY) &&
		(cam_mod->state != OV_CAMERA_MODULE_STREAMING)) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"cannot get controls when camera is off\n");
		return -EFAULT;
	}

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		struct v4l2_subdev *af_ctrl;
		af_ctrl = pltfrm_camera_module_get_af_ctrl(sd);
		if (!IS_ERR_OR_NULL(af_ctrl)) {
			ret = v4l2_subdev_call(af_ctrl, core, g_ctrl, ctrl);
			return ret;
		}
	}

	if (!IS_ERR_OR_NULL(cam_mod->custom.g_ctrl)) {
		ret = cam_mod->custom.g_ctrl(cam_mod, ctrl->id);
		if (IS_ERR_VALUE(ret))
			return ret;
	}

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		ctrl->value = cam_mod->exp_config.gain;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			     "V4L2_CID_GAIN %d\n",
			     ctrl->value);
		break;
	case V4L2_CID_EXPOSURE:
		ctrl->value = cam_mod->exp_config.exp_time;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			     "V4L2_CID_EXPOSURE %d\n",
			     ctrl->value);
		break;
	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		ctrl->value = cam_mod->wb_config.temperature;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_WHITE_BALANCE_TEMPERATURE %d\n",
			ctrl->value);
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		ctrl->value = cam_mod->wb_config.preset_id;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE %d\n",
			ctrl->value);
		break;
	case V4L2_CID_AUTOGAIN:
		ctrl->value = cam_mod->exp_config.auto_gain;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_AUTOGAIN %d\n",
			ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ctrl->value = cam_mod->exp_config.auto_exp;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_EXPOSURE_AUTO %d\n",
			ctrl->value);
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ctrl->value = cam_mod->wb_config.auto_wb;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_AUTO_WHITE_BALANCE %d\n",
			ctrl->value);
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		ctrl->value = cam_mod->af_config.abs_pos;
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_FOCUS_ABSOLUTE %d\n",
			ctrl->value);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		/* TBD */
		/* fallthrough */
	default:
		pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"failed, unknown ctrl %d\n", ctrl->id);
		return -EINVAL;
	}

	return 0;
}

/* ======================================================================== */

int ov_camera_module_s_ext_ctrls(
	struct v4l2_subdev *sd,
	struct v4l2_ext_controls *ctrls)
{
	int i;
	int ctrl_cnt = 0;
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);
	int ret = 0;
	char *flash_driver = NULL;

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");
	if (ctrls->count == 0)
		return -EINVAL;

	flash_driver = pltfrm_camera_module_get_flash_driver_name(sd);

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_ext_control *ctrl;
		u32 ctrl_updt = 0;

		ctrl = &ctrls->controls[i];

		switch (ctrl->id) {
		case V4L2_CID_GAIN:
			ctrl_updt = OV_CAMERA_MODULE_CTRL_UPDT_GAIN;
			cam_mod->exp_config.gain = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_GAIN %d\n",
			ctrl->value);
			break;
		case V4L2_CID_FLASH_LED_MODE:
			if (ctrl->value == V4L2_FLASH_LED_MODE_NONE) {
				if (!strcmp(flash_driver, "FP6773C")) {
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_TORCH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE
					);
				} else if (!strcmp(flash_driver, "SGM3141")) {
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_TORCH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_FLASH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);
				} else {
					if (cam_mod->exp_config.flash_mode ==
						V4L2_FLASH_LED_MODE_FLASH)
						pltfrm_camera_module_set_pin_state(
						sd,
						PLTFRM_CAMERA_MODULE_PIN_FLASH,
						PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE
						);
					else if (cam_mod->exp_config.flash_mode ==
						V4L2_FLASH_LED_MODE_TORCH)
						pltfrm_camera_module_set_pin_state(
						sd,
						PLTFRM_CAMERA_MODULE_PIN_TORCH,
						PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE
						);
				}
			} else if (ctrl->value ==
					V4L2_FLASH_LED_MODE_FLASH) {
				pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_FLASH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE
					);
				if (!strcmp(flash_driver, "FP6773C")) {
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_TORCH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
				} else if (!strcmp(flash_driver, "SGM3141")) {
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_TORCH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_FLASH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
				} else {
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_TORCH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE
					);
					}
			} else if (ctrl->value ==
					V4L2_FLASH_LED_MODE_TORCH) {
				if (!strcmp(flash_driver, "SGM3141")) {
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_TORCH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_FLASH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
				} else {
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_FLASH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE
					);
					pltfrm_camera_module_set_pin_state(
					sd,
					PLTFRM_CAMERA_MODULE_PIN_TORCH,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE
					);
				}
			}
			cam_mod->exp_config.flash_mode = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
				"V4L2_CID_FLASH_LED_MODE %d\n",
				ctrl->value);
			break;
		case V4L2_CID_EXPOSURE:
			ctrl_updt = OV_CAMERA_MODULE_CTRL_UPDT_EXP_TIME;
			cam_mod->exp_config.exp_time = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_EXPOSURE %d\n",
			ctrl->value);
			break;
		case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
			ctrl_updt = OV_CAMERA_MODULE_CTRL_UPDT_WB_TEMPERATURE;
			cam_mod->wb_config.temperature = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_WHITE_BALANCE_TEMPERATURE %d\n",
			ctrl->value);
			break;
		case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
			ctrl_updt = OV_CAMERA_MODULE_CTRL_UPDT_PRESET_WB;
			cam_mod->wb_config.preset_id = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE %d\n",
			ctrl->value);
			break;
		case V4L2_CID_AUTOGAIN:
			ctrl_updt = OV_CAMERA_MODULE_CTRL_UPDT_AUTO_GAIN;
			cam_mod->exp_config.auto_gain = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_AUTOGAIN %d\n",
			ctrl->value);
			break;
		case V4L2_CID_EXPOSURE_AUTO:
			ctrl_updt = OV_CAMERA_MODULE_CTRL_UPDT_AUTO_EXP;
			cam_mod->exp_config.auto_exp = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_EXPOSURE_AUTO %d\n",
			ctrl->value);
			break;
		case V4L2_CID_AUTO_WHITE_BALANCE:
			ctrl_updt = OV_CAMERA_MODULE_CTRL_UPDT_AUTO_WB;
			cam_mod->wb_config.auto_wb = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_AUTO_WHITE_BALANCE %d\n",
			ctrl->value);
			break;
		case INTEL_V4L2_CID_AUTO_FPS:
			cam_mod->auto_adjust_fps = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"INTEL_V4L2_CID_AUTO_FPS %d\n",
			ctrl->value);
			break;
		case V4L2_CID_FOCUS_ABSOLUTE:
			{
				struct v4l2_subdev *af_ctrl;
				af_ctrl = pltfrm_camera_module_get_af_ctrl(sd);
				if (!IS_ERR_OR_NULL(af_ctrl)) {
					struct v4l2_control single_ctrl;
					single_ctrl.id =
						V4L2_CID_FOCUS_ABSOLUTE;
					single_ctrl.value = ctrl->value;
					ret = v4l2_subdev_call(af_ctrl,
						core, s_ctrl, &single_ctrl);
					return ret;
				}
			}
			ctrl_updt =
				OV_CAMERA_MODULE_CTRL_UPDT_FOCUS_ABSOLUTE;
			cam_mod->af_config.abs_pos = ctrl->value;
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
			"V4L2_CID_FOCUS_ABSOLUTE %d\n",
			ctrl->value);
			break;
		case V4L2_CID_HFLIP:
			if (ctrl->value)
				cam_mod->hflip = true;
			else
				cam_mod->hflip = false;
			break;
		case V4L2_CID_VFLIP:
			if (ctrl->value)
				cam_mod->vflip = true;
			else
				cam_mod->vflip = false;
			break;
		default:
			pltfrm_camera_module_pr_warn(&cam_mod->sd,
			"ignoring unknown ctrl 0x%x\n", ctrl->id);
			break;
		}

		if (cam_mod->state != OV_CAMERA_MODULE_SW_STANDBY &&
		cam_mod->state != OV_CAMERA_MODULE_STREAMING)
			cam_mod->ctrl_updt |= ctrl_updt;
		else if (ctrl_updt)
			ctrl_cnt++;
	}

	/* if camera module is already streaming, write through */
	if (ctrl_cnt &&
		(cam_mod->state == OV_CAMERA_MODULE_STREAMING ||
		cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY)) {
		struct ov_camera_module_ext_ctrls ov_ctrls;

		ov_ctrls.ctrls =
		(struct ov_camera_module_ext_ctrl *)
		kmalloc(ctrl_cnt*sizeof(struct ov_camera_module_ext_ctrl),
			GFP_KERNEL);

		if (ov_ctrls.ctrls) {
			for (i = 0; i < ctrl_cnt; i++) {
				ov_ctrls.ctrls[i].id = ctrls->controls[i].id;
				ov_ctrls.ctrls[i].value =
					ctrls->controls[i].value;
			}

			ov_ctrls.count = ctrl_cnt;

			ret = cam_mod->custom.s_ext_ctrls(cam_mod, &ov_ctrls);

			kfree(ov_ctrls.ctrls);
		} else
			ret = -ENOMEM;

		if (IS_ERR_VALUE(ret))
			pltfrm_camera_module_pr_debug(&cam_mod->sd,
				"failed with error %d\n", ret);
	}

	return ret;
}

/* ======================================================================== */

int ov_camera_module_s_ctrl(
	struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);
	struct v4l2_ext_control ext_ctrl[1];
	struct v4l2_ext_controls ext_ctrls;

	pltfrm_camera_module_pr_debug(&cam_mod->sd,
		"0x%x 0x%x\n", ctrl->id, ctrl->value);

	ext_ctrl[0].id = ctrl->id;
	ext_ctrl[0].value = ctrl->value;

	ext_ctrls.count = 1;
	ext_ctrls.controls = ext_ctrl;

	return ov_camera_module_s_ext_ctrls(sd, &ext_ctrls);
}

/* ======================================================================== */

long ov_camera_module_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd,
	void *arg)
{
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");

	if (cmd == INTEL_VIDIOC_SENSOR_MODE_DATA) {
		int ret;
		struct ov_camera_module_timings ov_timings;
		struct isp_supplemental_sensor_mode_data *timings =
		(struct isp_supplemental_sensor_mode_data *) arg;

		if (cam_mod->custom.g_timings)
			ret = cam_mod->custom.g_timings(cam_mod, &ov_timings);
		else
			ret = -EPERM;

		if (IS_ERR_VALUE(ret)) {
			pltfrm_camera_module_pr_err(&cam_mod->sd,
			"failed with error %d\n", ret);
			return ret;
		}

		timings->sensor_output_width = ov_timings.sensor_output_width;
		timings->sensor_output_height = ov_timings.sensor_output_height;
		timings->crop_horizontal_start =
			ov_timings.crop_horizontal_start;
		timings->crop_vertical_start = ov_timings.crop_vertical_start;
		timings->crop_horizontal_end = ov_timings.crop_horizontal_end;
		timings->crop_vertical_end = ov_timings.crop_vertical_end;
		timings->line_length_pck = ov_timings.line_length_pck;
		timings->frame_length_lines = ov_timings.frame_length_lines;
		timings->vt_pix_clk_freq_hz = ov_timings.vt_pix_clk_freq_hz;
		timings->binning_factor_x = ov_timings.binning_factor_x;
		timings->binning_factor_y = ov_timings.binning_factor_y;
		timings->coarse_integration_time_max_margin =
			ov_timings.coarse_integration_time_max_margin;
		timings->coarse_integration_time_min =
			ov_timings.coarse_integration_time_min;
		timings->fine_integration_time_max_margin =
			ov_timings.fine_integration_time_max_margin;
		timings->fine_integration_time_min =
			ov_timings.fine_integration_time_min;

		return ret;
	} else
		return -EINVAL;
}
/* ======================================================================== */

int ov_camera_module_enum_frameintervals(
	struct v4l2_subdev *sd,
	struct v4l2_frmivalenum *fival)
{
	struct ov_camera_module *cam_mod =  to_ov_camera_module(sd);

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "%d\n", fival->index);

	if (fival->index >= cam_mod->custom.num_configs)
		return -EINVAL;
	fival->pixel_format =
		cam_mod->custom.configs[fival->index].frm_fmt.code;
	fival->width = cam_mod->custom.configs[fival->index].frm_fmt.width;
	fival->height = cam_mod->custom.configs[fival->index].frm_fmt.height;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = cam_mod->custom.
		configs[fival->index].frm_intrvl.interval.numerator;
	fival->discrete.denominator = cam_mod->custom.
		configs[fival->index].frm_intrvl.interval.denominator;
	return 0;
}

/* ======================================================================== */

int ov_camera_module_write_reglist(
	struct ov_camera_module *cam_mod,
	const struct ov_camera_module_reg reglist[],
	int len)
{
	return pltfrm_camera_module_write_reglist(&cam_mod->sd, reglist, len);
}

/* ======================================================================== */

int ov_camera_module_write_reg(
	struct ov_camera_module *cam_mod,
	u16 reg,
	u8 val)
{
	return pltfrm_camera_module_write_reg(&cam_mod->sd, reg, val);
}

/* ======================================================================== */

int ov_camera_module_read_reg(
	struct ov_camera_module *cam_mod,
	u16 data_length,
	u16 reg,
	u32 *val)
{
	return pltfrm_camera_module_read_reg(&cam_mod->sd,
		data_length, reg, val);
}

/* ======================================================================== */

int ov_camera_module_read_reg_table(
	struct ov_camera_module *cam_mod,
	u16 reg,
	u32 *val)
{
	int i;

	if (cam_mod->state == OV_CAMERA_MODULE_STREAMING)
		return pltfrm_camera_module_read_reg(&cam_mod->sd,
			1, reg, val);

	if (!IS_ERR_OR_NULL(cam_mod->active_config)) {
		for (
			i = cam_mod->active_config->reg_table_num_entries - 1;
			i > 0;
			i--) {
			if (cam_mod->active_config->reg_table[i].reg == reg) {
				*val = cam_mod->active_config->reg_table[i].val;
				return 0;
			}
		}
	}

	if (cam_mod->state == OV_CAMERA_MODULE_SW_STANDBY)
		return pltfrm_camera_module_read_reg(&cam_mod->sd,
			1, reg, val);

	return -EFAULT;
}

/* ======================================================================== */

int ov_camera_module_init(struct ov_camera_module *cam_mod,
	struct ov_camera_module_custom_config *custom)
{
	int ret = 0;

	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");

	cam_mod->custom = *custom;
	ov_camera_module_reset(cam_mod);

	if (IS_ERR_OR_NULL(custom->start_streaming) ||
		IS_ERR_OR_NULL(custom->stop_streaming) ||
		IS_ERR_OR_NULL(custom->s_ctrl) ||
		IS_ERR_OR_NULL(custom->g_ctrl)) {
		pltfrm_camera_module_pr_err(&cam_mod->sd,
			"mandatory callback function is missing\n");
		ret = -EINVAL;
		goto err;
	}

	ret = pltfrm_camera_module_init(&cam_mod->sd, &cam_mod->pltfm_data);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = pltfrm_camera_module_set_pin_state(&cam_mod->sd,
					PLTFRM_CAMERA_MODULE_PIN_PD,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
	ret = pltfrm_camera_module_set_pin_state(&cam_mod->sd,
					PLTFRM_CAMERA_MODULE_PIN_RESET,
					PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
	if (IS_ERR_VALUE(ret)) {
		ov_camera_module_release(cam_mod);
		goto err;
	}

	return 0;
err:
	pltfrm_camera_module_pr_err(&cam_mod->sd,
		"failed with error %d\n", ret);
	return ret;
}

void ov_camera_module_release(struct ov_camera_module *cam_mod)
{
	pltfrm_camera_module_pr_debug(&cam_mod->sd, "\n");

	cam_mod->custom.configs = NULL;

	pltfrm_camera_module_release(&cam_mod->sd);
	v4l2_device_unregister_subdev(&cam_mod->sd);
}

