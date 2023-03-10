/*
 * Component: XGOLD audio jack driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/driver.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#endif

#include <sofia/vmm_pmic.h>

#include "xgold_jack.h"

/* FIXME */
#include "../codecs/afe_acc_det.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: jack: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: jack: "fmt, ##arg)


#define AHJ_TYPE_MIN_MV 475
#define AHJ_TYPE_MAX_MV 2800

#define HEADPHONE_MIN_MV 0
#define HEADPHONE_MAX_MV 70
/**
 * Different VBIAS settings
**/
enum xgold_vbias {
	XGOLD_VBIAS_ENABLE,
	XGOLD_VBIAS_ULP_ON,
};

enum xgold_headset_type {
	XGOLD_HEADSET_REMOVED,
	XGOLD_HEADSET,
	XGOLD_HEADPHONE,
	XGOLD_INVALID,
	XGOLD_ERROR
};

/*struct hs_cfg {
	int min_mv;
	int max_mv;
	enum xgold_headset_type type;
};*/

struct hs_key_cfg {
	int min_mv;
	int max_mv;
	enum snd_jack_types type;
	int key_code;
	int pressed;
};

/* AFE register values */
#define XGOLD_DETECT_INSERTION \
	0x80FB /* ACD1: insertion; ACD2: disabled;*/
#define XGOLD_DETECT_REMOVAL_HEADSET \
	0xC0F3 /* ACD1: removal; ACD2: headset insertion;*/
#define XGOLD_DETECT_REMOVAL_HOOK \
	0xCAF3 /* ACD1: headset removal; ACD2: hook key press;*/
#define XGOLD_DETECT_HOOK_RELEASE \
	0xC2F3 /* ACD1: removal; ACD2: hook key release;*/

/* PMIC register offset */
/* IRQ registers offsets */
#define IRQMULT_REG			0x1e
#define MIRQMULT_REG			0x1f

/* Masks and bits */
#define IRQMULT_ACCDET1_M		0x01
#define IRQMULT_ACCDET2_M		0x02
#define IRQMULT_ACCDETAUX_M		0x04
#define IRQMULT_ACCDETALL_M \
	(IRQMULT_ACCDET1_M | IRQMULT_ACCDET2_M | IRQMULT_ACCDETAUX_M)

/* PMIC registers offsets */
#define ACC_DET_LOW_REG			0x21
#define ACC_DET_HIGH_REG		0x20
#define ACC_DET_AUX_REG			0x23

#define VBIAS_SETTLING_TIME_MS		20

/* Headset keymap */
struct hs_key_cfg xgold_hs_keymap[] = {
	{0, 100, SND_JACK_BTN_0 , KEY_MEDIA, 0},
	{130, 150, SND_JACK_BTN_1, KEY_VOLUMEUP, 0},
	{275, 325, SND_JACK_BTN_2, KEY_VOLUMEDOWN, 0},
};

struct delayed_work hs_spk_lo, hs_spk_hi;
extern int Read_PROJ_ID(void);
static int PROJ_ID;

static int hp_status;

static ssize_t hpstate_show(struct device *dev, struct device_attribute *attr,
                        char *buf);

static ssize_t hpstate_store(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count);

static DEVICE_ATTR(headset_status, S_IWUSR | S_IRUGO, hpstate_show, hpstate_store);

static ssize_t hpstate_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        //printk("joe hs_show \n");
        return sprintf(buf, "%d\n", hp_status);
}

static ssize_t hpstate_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
        //printk("joe hs_store %s\n", buf);
        return 1;
}

static int jack_write(struct xgold_jack *jack, unsigned val)
{
#ifdef CONFIG_X86_INTEL_SOFIA
	return mv_svc_reg_write(jack->base_phys, val, -1);
#else
	iowrite(val, jack->mmio_base);
	return 0;
#endif
}

/* PMIC reg accesses */
static int xgold_jack_pmic_reg_read(u32 dev_addr, u32 reg_addr,
		u8 *p_reg_val)
{
	u32 vmm_addr, reg_val = 0;
	int ret;

	vmm_addr = ((dev_addr & 0xFF) << 24) | (reg_addr & 0xFF);
	ret = vmm_pmic_reg_read(vmm_addr, &reg_val);
	*p_reg_val = (u8)(reg_val & 0xFF);
	xgold_debug("%s: read @%X return %X\n", __func__, reg_addr, reg_val);

	return ret;
}

static int xgold_jack_pmic_reg_write(u32 dev_addr, u32 reg_addr, u8 reg_val)
{
	u32 vmm_addr, val = reg_val;

	vmm_addr = ((dev_addr & 0xFF) << 24) | (reg_addr & 0xFF);
	xgold_debug("%s: write @%X value %X\n", __func__, reg_addr, val);
	return vmm_pmic_reg_write(vmm_addr, val);
}

/* Call to AFE to change the VBIAS settings */
static void configure_vbias(struct xgold_jack *jack, enum xgold_vbias state)
{
	struct afe_acc_det acc_det_par;
	int ret;

	xgold_debug("--> %s: %s\n", __func__, (state == XGOLD_VBIAS_ENABLE) ?
			"XGOLD_VBIAS_ENABLE" : "XGOLD_VBIAS_ULP_ON");

	acc_det_par.vumic_conf.vmode = AFE_VUMIC_MODE_ULP;
	acc_det_par.vumic_conf.hzmic = AFE_HZVUMIC_NORMAL_POWER_DOWN;

	switch (state) {
	case XGOLD_VBIAS_ENABLE:
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_2_1_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_NORMAL;
		acc_det_par.xb_mode = AFE_XB_ON;
		break;
	case XGOLD_VBIAS_ULP_ON:
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_1_9_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_LOW_POWER;
		acc_det_par.xb_mode = AFE_XB_OFF;
		break;
	default:
		return;
	}

	if (jack->flags & XGOLD_JACK_PMIC)
		ret = pmic_afe_set_acc_det_with_lock(acc_det_par);
	else
		ret = agold_afe_set_acc_det_with_lock(acc_det_par);

	if (ret)
		xgold_err("Error when setting VBIAS!\n");

	xgold_debug("<-- %s\n", __func__);
}

static u32 read_state(struct xgold_jack *jack)
{
	int volt, ret;
        int checkgpio;

	ret = iio_read_channel_processed(jack->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return XGOLD_ERROR;
	}

	xgold_err("%s: measured voltage %d\n", __func__, volt);
        checkgpio = gpio_get_value(72);
        xgold_err("GPIO 72 :%d\n",checkgpio);

	if (volt >= AHJ_TYPE_MIN_MV && volt <= AHJ_TYPE_MAX_MV && checkgpio == 0)
		return XGOLD_HEADSET;
	else if (volt >= HEADPHONE_MIN_MV && volt <= HEADPHONE_MAX_MV && checkgpio == 0)
		return XGOLD_HEADPHONE;
	else if (volt > AHJ_TYPE_MAX_MV || (volt > 1800 && checkgpio == 1))
		return XGOLD_HEADSET_REMOVED;
	else
		return XGOLD_INVALID;
}

static void xgold_jack_acc_det_write(struct xgold_jack *jack,
		unsigned val)
{
	int ret;

	xgold_debug("%s: write val 0x%X, mode %s\n", __func__, val,
			(jack->flags & XGOLD_JACK_PMIC) ? "PMIC" : "IO");

	if (jack->flags & XGOLD_JACK_PMIC) {
		ret = xgold_jack_pmic_reg_write(jack->pmic_addr,
				ACC_DET_HIGH_REG, (val >> 8) & 0xFF);
		if (ret) {
			xgold_err("%s: cannot write ACC_DET_HIGH\n",
					__func__);
			return;
		}

		ret = xgold_jack_pmic_reg_write(jack->pmic_addr,
				ACC_DET_LOW_REG, val & 0xFF);
		if (ret) {
			xgold_err("%s: cannot write ACC_DET_LOW\n",
					__func__);
			return;
		}
	} else
		jack_write(jack, val);
}

static void xgold_jack_check(struct xgold_jack *jack)
{
	u32 state, old_state;
	u32 detect;
	int status = 0;
	enum xgold_vbias vbias;

	/*  set the flag for button thread to wait until release it.*/
	configure_vbias(jack, XGOLD_VBIAS_ENABLE);

	/* First, make sure we have a stable state.
	   Headset insertion takes a bit of time(~> 500ms),
	   so make sure that two consecutive reads agree.
	*/

	do {
		msleep(250);
		old_state = read_state(jack);
		msleep(250);
		state = read_state(jack);
	} while (state != old_state);

	if (XGOLD_ERROR == state) {
		xgold_err("Unable to determine state.\n");
		return;
	}

	switch (state) {
	case XGOLD_HEADPHONE:
		xgold_err("Headphone inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		detect = XGOLD_DETECT_REMOVAL_HEADSET;
		status = SND_JACK_HEADPHONE;
		jack->buttons_enabled = false;
                hp_status = 2;
                if (PROJ_ID == 0x2 || PROJ_ID == 0x6) {
                	schedule_delayed_work(&hs_spk_lo,msecs_to_jiffies(1500));
                }
		break;
	case XGOLD_HEADSET:
		xgold_err("Headset inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		detect = XGOLD_DETECT_REMOVAL_HOOK;
		status = SND_JACK_HEADSET;
		jack->buttons_enabled = true;
                hp_status = 1;
                if (PROJ_ID == 0x2 || PROJ_ID == 0x6) {
                	schedule_delayed_work(&hs_spk_lo,msecs_to_jiffies(1500));
                }
		break;
	case XGOLD_HEADSET_REMOVED:
		xgold_err("Headphone/headset removed\n");
		vbias = XGOLD_VBIAS_ULP_ON;
		detect = XGOLD_DETECT_INSERTION;
		jack->buttons_enabled = false;
                hp_status = 0;
                if (PROJ_ID == 0x2 || PROJ_ID == 0x6) {
                	schedule_delayed_work(&hs_spk_hi,msecs_to_jiffies(1500));
                }
		break;
	default:
		xgold_err("Invalid headset state!\n");
                hp_status = -1;
                if (PROJ_ID == 0x2 || PROJ_ID == 0x6) {
                	schedule_delayed_work(&hs_spk_hi,msecs_to_jiffies(1500));
                }
		return;
	}

	configure_vbias(jack, vbias);
	msleep(VBIAS_SETTLING_TIME_MS);

	/* Check if there really is a state change */
	if (status != (jack->hs_jack->status & SND_JACK_HEADSET)) {
		xgold_jack_acc_det_write(jack, detect);
		snd_soc_jack_report(jack->hs_jack, status, SND_JACK_HEADSET);
	}
}

static irqreturn_t xgold_jack_detection(int irq, void *data)
{
	struct xgold_jack *jack = data;

	xgold_debug("%s\n", __func__);

        xgold_jack_check((struct xgold_jack *)data);

        if (PROJ_ID == 0x2 || PROJ_ID == 0x6) {
        	gpio_set_value(80,1);
        }

	if (jack->flags & XGOLD_JACK_PMIC)
		xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
				IRQMULT_REG, IRQMULT_ACCDET1_M);

	return IRQ_HANDLED;
}

static void xgold_button_check(struct xgold_jack *jack)
{
	int ret, volt, i;
	int key_index = -1;
	u32 detect;
	int status;
	enum snd_jack_types type;

	ret = iio_read_channel_processed(jack->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return;
	}

	xgold_err("%s: measured  button voltage %d\n", __func__, volt);

	/* check if a key has been pressed and remember this*/
	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
		if ((volt >= xgold_hs_keymap[i].min_mv) &&
			(volt <= xgold_hs_keymap[i].max_mv)) {
			xgold_hs_keymap[i].pressed = 1;
			key_index = i;
			break;
		}
	}

	if (key_index > -1) {
		xgold_err("button press index %d\n", key_index);
		type = xgold_hs_keymap[key_index].type;
		status = type;
		detect = XGOLD_DETECT_HOOK_RELEASE;
	} else {
		/* key released, figure out which */
		for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
			if (1 == xgold_hs_keymap[i].pressed) {
				xgold_err("button release, index %d\n", i);
				xgold_hs_keymap[i].pressed = 0;
				key_index = i;
				type = xgold_hs_keymap[key_index].type;
				status = 0;
			}
		}
		detect = XGOLD_DETECT_REMOVAL_HOOK;
	}
	if (key_index > -1) {
		snd_soc_jack_report(jack->hs_jack, status, type);
		xgold_jack_acc_det_write(jack, detect);
	}
}

static irqreturn_t xgold_button_detection(int irq, void *data)
{
	struct xgold_jack *jack = data;

	xgold_debug("%s\n", __func__);

	if (jack->flags & XGOLD_JACK_PMIC)
		xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
				IRQMULT_REG, IRQMULT_ACCDET2_M);

	if ((jack->hs_jack->status & SND_JACK_HEADSET) != SND_JACK_HEADSET) {
		/* this interrupt may occurs in case of slow jack insertion */
		xgold_debug("button detection while no headset\n");
		return xgold_jack_detection(irq, data);
	}

	if (jack->buttons_enabled)
		xgold_button_check(jack);
	return IRQ_HANDLED;
}

int xgold_jack_setup(struct snd_soc_codec *codec, struct snd_soc_jack *hs_jack)
{
	int i, type, ret;

	xgold_debug("%s\n", __func__);

	type = SND_JACK_HEADSET;
	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++)
		type |= xgold_hs_keymap[i].type;

	ret = snd_soc_jack_new(codec, "Headset", type, hs_jack);

	if (ret) {
		xgold_err("Jack creation failed\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
		ret = snd_jack_set_key(hs_jack->jack,
			xgold_hs_keymap[i].type,
			xgold_hs_keymap[i].key_code);
		if (ret)
			xgold_err("Failed to set headset key\n");
	}

	return ret;
}

static void do_hs_spk_lo(struct work_struct *work)
{
       if (PROJ_ID == 0x2 || PROJ_ID == 0x6) {       
                gpio_set_value(80,0); 
       }
}

static void do_hs_spk_hi(struct work_struct *work)
{
       if (PROJ_ID == 0x2 || PROJ_ID == 0x6) {
       	       gpio_set_value(80,1);
       }
}

struct xgold_jack *of_xgold_jack_probe(struct platform_device *pdev,
		struct device_node *np, struct snd_soc_jack *hs_jack)
{
	struct xgold_jack *jack;
	struct resource regs;
	struct resource *res;
	int num_irq, i, ret;
	unsigned value;

	jack = devm_kzalloc(&pdev->dev, sizeof(*jack), GFP_ATOMIC);
	if (!jack) {
		xgold_err("Allocation failed!\n");
		ret = -ENOMEM;
		goto out;
	}

	jack->buttons_enabled = false;
	jack->jack_irq = jack->button_irq = -1;
	jack->hs_jack = hs_jack;

        PROJ_ID = Read_PROJ_ID();
	printk("%s : jack PROJ_ID %d\n", __func__, PROJ_ID);

	if (of_device_is_compatible(np, "intel,headset,pmic"))
		jack->flags |= XGOLD_JACK_PMIC;

	if (jack->flags & XGOLD_JACK_PMIC) {
		/* PMIC device address */
		ret = of_property_read_u32_index(np, "intel,reg", 0, &value);
		if (ret)
			goto out;

		if (value > 0xFF) {
			ret = -ERANGE;
			goto out;
		}

		jack->pmic_addr = (unsigned char)value;

		/* FIXME: should be handled by VMM, not linux driver */
		/* PMIC device address for IRQ handling */
		ret = of_property_read_u32_index(np, "intel,irq-reg", 0,
				&value);
		if (ret)
			goto out;

		if (value > 0xFF) {
			ret = -ERANGE;
			goto out;
		}

		jack->pmic_irq_addr = (unsigned char)value;
	} else {
		if (of_address_to_resource(np, 0, &regs)) {
			ret = -ENOENT;
			goto out;
		}

		jack->mmio_base = devm_ioremap(
				&pdev->dev, regs.start, resource_size(&regs));
		if (jack->mmio_base == NULL) {
			xgold_err("failed to remap I/O memory\n");
			ret = -ENXIO;
			goto out;
		}
		jack->base_phys = regs.start;
		xgold_debug("ioremap %p\n", jack->mmio_base);
	}

	num_irq = of_irq_count(np);
	if (!num_irq) {
		xgold_err("no headset plug irq defined\n");
		ret = -EINVAL;
		goto out;
	}

	res = devm_kzalloc(&pdev->dev, sizeof(*res) * num_irq, GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}

	of_irq_to_resource_table(np, res, num_irq);
	for (i = 0; i < num_irq; i++)
		if (strcmp(res[i].name, "acd1") == 0)
			jack->jack_irq = res[i].start;

	jack->iio_client = iio_channel_get(NULL, "ACCID_SENSOR");
	if (IS_ERR(jack->iio_client)) {
		xgold_err("iio channel error\n");
		ret = -EINVAL;
		goto out;
	}

	/* Configure the Accessory settings to detect Insertion */
	xgold_jack_acc_det_write(jack, XGOLD_DETECT_INSERTION);

	ret = devm_request_threaded_irq(&(pdev->dev), jack->jack_irq,
			NULL,
			xgold_jack_detection,
			IRQF_SHARED | IRQF_ONESHOT, "jack_irq", jack);
	if (ret) {
		xgold_err("setup of jack irq failed!\n");
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < num_irq; i++)
		if (strcmp(res[i].name, "acd2") == 0)
			jack->button_irq = res[i].start;

	ret = devm_request_threaded_irq(&(pdev->dev), jack->button_irq,
			NULL,
			xgold_button_detection,
			IRQF_SHARED | IRQF_ONESHOT, "button_irq", jack);
	if (ret < 0) {
		xgold_err("setup of button irq failed!\n");
		ret = -EINVAL;
		goto out;
	}

	/* FIXME: below code should be handled by irqchip level/vmm, when
	 * requesting for the PMIC ACD interrupt, and not in this driver */
	if (jack->flags & XGOLD_JACK_PMIC) {
		int tries;
		char val;

		/* Unmask IRQMULT interrupt */
		xgold_err("%s: Warning! may apply changes to MIRQMULT register\n",
				__func__);

		xgold_jack_pmic_reg_read(jack->pmic_irq_addr,
				MIRQMULT_REG, &val);
		tries = 0;
		while ((val & IRQMULT_ACCDETALL_M) && tries++ < 20) {
			xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
					MIRQMULT_REG,
					val & ~IRQMULT_ACCDETALL_M);

			/* read again to ensure Mask is correctly configured */
			xgold_jack_pmic_reg_read(jack->pmic_irq_addr,
					MIRQMULT_REG, &val);
			xgold_debug("%s: MIRQMULT is 0x%02X\n", __func__, val);
		}

		if (tries >= 20) {
			ret = -EIO;
			goto out;
		}

		xgold_err("%s MIRQLVL1 is 0x%02X\n", __func__, val);
	}
	/* end of FIXME */

        ret = device_create_file(&pdev->dev, &dev_attr_headset_status);
        if (ret < 0)
                xgold_err("%s(): add headset_status error\n", __func__);

        INIT_DELAYED_WORK(&hs_spk_lo, do_hs_spk_lo);
        INIT_DELAYED_WORK(&hs_spk_hi, do_hs_spk_hi);

	return jack;

out:
	return ERR_PTR(ret);
}

void xgold_jack_remove(struct xgold_jack *jack)
{
	if (jack && !IS_ERR(jack->iio_client))
		iio_channel_release(jack->iio_client);
}
