/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#define SMB345_NAME "smb345_charger"
#define pr_fmt(fmt) SMB345_NAME": "fmt

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/usb/phy-intel.h>

#include <linux/time.h>
#include <linux/wakelock.h>

/*
 * Development debugging is not enabled in release image to prevent
 * loss of event history in the debug array which has a limited size
 */
#include <linux/power/charger_debug.h>

/* I2C communication related */
#define I2C_RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

#define CFG_CHARGE_CURRENT            0x00
#define CFG_CHARGE_CURRENT_FCC_MASK        0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT        5
#define CFG_CHARGE_CURRENT_PCC_MASK        0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT        3
#define CFG_CHARGE_CURRENT_TC_MASK        0x07
#define CFG_CHARGE_CURRENT_ALL        0x41
#define CFG_CURRENT_LIMIT            0x01
#define CFG_CURRENT_LIMIT_DC_MASK        0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT        4
#define CFG_CURRENT_LIMIT_USB_MASK        0x0f
#define CFG_CURRENT_LIMIT_SMB346_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346_VALUE_2000 0x70
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1800 0x60
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1500 0x50
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1000 0x30
#define CFG_CURRENT_LIMIT_SMB346_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346_VALUE_500 0x10
#define IBUS_MIN_LIMIT_MA 100
#define IBUS_LIMIT_STEP_MA 400
#define IBUS_MAX_LIMIT_MA 900
#define FLOAT_VOLTAGE_REG        0x03

#define IOCHARGE_MAX_MA 1500
#define DEFAULT_CC 350
#define DEFAULT_CV 3380

#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define DBG_STATE_FILENAME "charger_state"

/* Pin and enable control */
#define CFG_PIN                    0x06
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH        0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_APSD_IRQ            BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR        BIT(2)
/* Command registers */
#define CMD_A                    0x30
#define CMD_A_CHG_ENABLED            BIT(1)
#define CMD_A_SUSPEND_ENABLED            BIT(2)
#define CMD_A_OTG_ENABLED            BIT(4)
#define CMD_A_ALLOW_WRITE            BIT(7)
#define CMD_B                    0x31
#define CMD_B_USB9_AND_HC_MODE    0x03
#define CMD_C                    0x33

/* Interrupt Status registers */
#define IRQSTAT_A                0x35
#define IRQSTAT_C                0x37
#define IRQSTAT_C_TERMINATION_STAT        BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ        BIT(1)
#define IRQSTAT_C_TAPER_IRQ            BIT(3)
#define IRQSTAT_D                0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT        BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ        BIT(3)
#define IRQSTAT_E                0x39
#define IRQSTAT_E_USBIN_UV_STAT            BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ            BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT            BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ            BIT(5)
#define IRQSTAT_F                0x3a
#define IRQSTAT_F_OTG_UV_IRQ            BIT(5)
#define IRQSTAT_F_OTG_UV_STAT            BIT(4)

/* Status registers */
#define STAT_A                    0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK        0x3f
#define STAT_B                    0x3c
#define STAT_C                    0x3d
#define STAT_C_CHG_ENABLED            BIT(0)
#define STAT_C_HOLDOFF_STAT            BIT(3)
#define STAT_C_CHG_MASK                0x06
#define STAT_C_CHG_SHIFT            1
#define STAT_C_CHG_TERM                BIT(5)
#define STAT_C_CHARGER_ERROR            BIT(6)
#define STAT_E                    0x3f

/* PMU Register used for POK status */
#define PMU_CONFIG_O    0x00U
    #define PMU_CONFIG(_base) ((_base) + PMU_CONFIG_O)

    #define PMU_CONFIG_CHGDET_O (BIT(1))
    #define PMU_CONFIG_CHGDET_M (BIT(1))

#define CHARGER_CONTROL_O 0x0
#define CHARGER_CONTROL(_base) ((_base) + CHARGER_CONTROL_O)
	#define CHARGER_CONTROL_CIEDG_O 26
	#define CHARGER_CONTROL_CIEDG_M 0x1
	#define CHARGER_CONTROL_CILVL_O 25
	#define CHARGER_CONTROL_CILVL_M 0x1
	#define CHARGER_CONTROL_CISENS_O 24
	#define CHARGER_CONTROL_CISENS_M 0x1
	#define CHARGER_CONTROL_CIEN_O 23
	#define CHARGER_CONTROL_CIEN_M 0x1
	#define CHARGER_CONTROL_CIDBT_O 17
	#define CHARGER_CONTROL_CIDBT_M 0x7
	#define CHARGER_CONTROL_CHGLVL_O 1
	#define CHARGER_CONTROL_CHGLVL_M 0x1
	/*chrg det*/
	#define CHARGER_CONTROL_CDETSENS_O 10
	#define CHARGER_CONTROL_CDETSENS_M 0x1
	#define CHARGER_CONTROL_CHDETLVL_O 6
	#define CHARGER_CONTROL_CHDETLVL_M 0x1
	#define CHARGER_CONTROL_CHDWEN_O 5
	#define CHARGER_CONTROL_CHDWEN_M 0x1

#define CHARGER_CONTROL_CIEDG_FALLING 0
#define CHARGER_CONTROL_CILVL_LOW 0
#define CHARGER_CONTROL_CISENS_EDGE 1
#define CHARGER_CONTROL_CIEN_EN 0
#define CHARGER_CONTROL_CHGLVL_LOW 0
#define CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE 0
/*chrg det*/
#define CHARGER_CONTROL_CDETSENS_EDGE 1
#define CHARGER_CONTROL_CHDETLVL_LOW 0
#define CHARGER_CONTROL_CHDETLVL_HIGH 1
#define CHARGER_CONTROL_CHDWEN_EN 1

#define CHARGER_CONTROL_WR_O 0x8
#define CHARGER_CONTROL_WR(_base) ((_base) + CHARGER_CONTROL_WR_O)
	#define CHARGER_CONTROL_WR_WS_O 0
	#define CHARGER_CONTROL_WR_WS_M 0x1

#define SMB345_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))
#define OTG_TLIM_THERM_CNTRL_REG                0x0A
#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB345_MASK(2, 2)
#define OTG_CURRENT_LIMIT_750mA    (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_500mA    BIT(3)
#define OTG_CURRENT_LIMIT_250mA    BIT(2)
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    SMB345_MASK(2, 0)
#define FLOAT_VOLTAGE_MASK                SMB345_MASK(6, 0)

#define fit_in_range(__val, __MIN, __MAX) ((__val > __MAX) ? __MAX : \
					(__val < __MIN) ? __MIN : __val)

#define SYSFS_FAKE_VBUS_SUPPORT 1

#define SYSFS_INPUT_VAL_LEN (1)
struct workqueue_struct *charger_work_queue = NULL;
struct delayed_work charger_work;

/* ================================== ASUS ================================== */
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/HWVersion.h>
#include <linux/proc_fs.h>
#include "smb345_external_include.h"
#include "ti/asus_battery.h"

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
static bool screen_off = false;
#endif
static int charger_type_report(struct notifier_block *nb, unsigned long event, void *ptr);
static struct notifier_block charger_type_notifier = {
    .notifier_call = charger_type_report,
};
extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);
extern const char* Read_HW_ID_STR(void);
extern int Read_HW_ID(void);
#define INTEL_STYLE 0
#define CFG_ADDRESS		0x0e
static struct dentry *dentry;
static struct chgr_dev_func smb345_tbl;
static int g_usb_state = CABLE_OUT;
static bool g_charging_toggle = true;
static bool g_is_power_supply_registered = false;
static spinlock_t spinlock_suspend;
static bool suspend_smb = false;
static bool suspend_otg_work_not_done = false;

/* AICL Results Table (mA) : 3Fh */
static const unsigned int aicl_results[] = {
#if defined(CONFIG_Z580C)
	/* smb358 */
	300, 500, 700, 1000, 1200, 1300, 1800, 2000
#else
	/* PF400CG, Z300C/CG, Z380C adopted smb345, smb346, smb347 */
	300, 500, 700, 900, 1200, 1500, 1800, 2000, 2200, 2500
#endif
};

#ifdef ME372CG_ENG_BUILD
bool eng_suspend_mode = false;
static bool suspend_mode(bool);
static ssize_t nasus_suspend_mode_toggle_write(struct file *file,
    const char __user *buf, size_t count, loff_t * ppos)
{
	char proc_buf[64] = {0};

	if (count > sizeof(proc_buf)) {
		BAT_DBG("%s: data error\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(proc_buf, buf, count)) {
		BAT_DBG("%s: read data from user space error\n", __func__);
		return -EFAULT;
	}

	BAT_DBG_E(" %s: %s", __func__, proc_buf);

	if (!strncmp("1", proc_buf, 1)) {
		/* turn on suspend mode in eng build */
		eng_suspend_mode = true;
		BAT_DBG_E(" %s: turn on suspend mode", __func__);
	}
	else if (!strncmp("0", proc_buf, 1)) {
		/* turn off suspend mode in eng build */
		eng_suspend_mode = false;
		BAT_DBG_E(" %s: turn off suspend mode", __func__);
	}

	suspend_mode(eng_suspend_mode);

	return count;
}
static int nasus_suspend_mode_toggle_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s: Suspend mode %s\n", __func__, eng_suspend_mode ? "On" : "Off");
	return 0;
}
static int nasus_suspend_mode_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, nasus_suspend_mode_toggle_read, NULL);
}

int init_asus_charging_limit_toggle(void)
{
	/* interface for usb-in suspend mode */
	static const struct file_operations suspend_mode_toggle_fops = {
		.owner = THIS_MODULE,
		.open = nasus_suspend_mode_toggle_open,
		.read = seq_read,
		.write = nasus_suspend_mode_toggle_write,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("driver/charger_suspend_mode", 0666, NULL,
		&suspend_mode_toggle_fops);
	if (!entry) {
		BAT_DBG_E("Unable to create suspend_mode\n");
		return -EINVAL;
	}

	return 0;
}
#else
static int twinsheaddragon_read(struct seq_file *m, void *v)
{
    seq_printf(m, "%s: Enable\n", __func__);

    setSMB345Charger(AC_IN);

    return 0;
}
static int twinsheaddragon_open(struct inode *inode, struct file *file)
{
    return single_open(file, twinsheaddragon_read, NULL);
}
int init_asus_charging_limit_toggle(void)
{
    struct proc_dir_entry *entry;

    static const struct file_operations twinsheaddragon_fops = {
        .owner = THIS_MODULE,
        .open = twinsheaddragon_open,
        .read = seq_read,
        .write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
    };

    entry = proc_create("driver/twinsheaddragon", 0666, NULL,
        &twinsheaddragon_fops);
    if (!entry) {
        BAT_DBG_E("Unable to create twinsheaddragon\n");
        return -EINVAL;
    }

    return 0;
}
#endif
/* ================================== ASUS ================================== */

enum {
	POK_B_VALID = 0,
	POK_B_INVAL,

	BOOSTOV_OCCURED = 1,

	BATUV_OCCURED = 1,

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	OVP_RECOV_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	TREG_IS_ON = 1,

	OT_RECOV_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,
};

enum charger_status {
	SMB345_STATUS_UNKNOWN,
	SMB345_STATUS_READY,
	SMB345_STATUS_FAULT,
};

/**
 * struct smb345_state - SMB345 charger current state
 * @status		charger driver status. Please see enum definition for
 *			details.
 * @vbus		equals 'VBUS_ON' (1) if valid vbus connected
 *			otherwise is 'VBUS_OFF'. '-1' -  means uninitialized.
 * @cc			current output current [mA] set on HW.
 * @max_cc		maximum output current [mA] that can be set on HW.
 * @cv			current output voltage [mV] set on HW.
 * @iterm		HW charging termination current [mA]. Not used as
 *			termination is controlled by SW.
 * @inlmt		input current limit [mA].
 * @health		charger chip health.
 * @cable_type		type of currently attached cable.
 * @charger_enabled	informs if charger is enabled for use.
 * @charging_enabled	informs if charging is currently enabled or not.
 * @to_enable_boost	informs whether boost is to be enabled
 * @boost_enabled	informs if boost mode is enabled
 * @pok_b		value of POK_B signal
 * @ovp_flag		value of OVP flag bit
 * @ovp_recov		value of OVP_RECOV flag bit
 * @t32s_timer_expired	charging watchdog timer expiration flag
 * @vbus_fault		'1' if vbus is fault, '0' otherwise
 * @treg_flag		charger thermal regulation active flag
 * @ot_recov_flag	over temperature recovery flag
 * @tsd_flag		thermal shutdown fault
 * @bat_uv		Battery voltage below 2.7v
 * @boost_ov		Boost out of regulation due to sustained current limit.
 */
struct smb345_state {
	enum charger_status status;
	int vbus;
	int cc;
	int max_cc;
	int cv;
	int iterm;
	int inlmt;
	int health;
	int cable_type;
	bool charger_enabled;
	bool charging_enabled;
	bool to_enable_boost;
	bool boost_enabled;
	unsigned int pok_b:1;
	unsigned int ovp_flag:1;
	unsigned int ovp_recov:1;
	unsigned int t32s_timer_expired:1;
	unsigned int vbus_fault:1;
	unsigned int treg_flag:1;
	unsigned int ot_recov_flag:1;
	unsigned int tsd_flag:1;

	/* Boost mode specific states */
	unsigned int bat_uv:1;
	unsigned int boost_ov:1;

};

/**
 * struct unfreezable_bh_struct -  structure describing suspend aware bottom half
 * @wq				pointer to  i2c client's structure.
 * @work			work associated with irq.
 * @evt_wakelock		wakelock acquired when bottom half is scheduled.
 * @in_suspend			Suspend flag. 'true' if driver was suspended
 *				'false' otherwise.
 * @pending_evt			pending event for bottom half execution flag.
 * @lock			spinlock protecting in_suspend and pending_evt
 *				flags.
 */
struct	unfreezable_bh_struct {
	struct workqueue_struct *wq;
	struct delayed_work work;
	struct wake_lock evt_wakelock;
	bool in_suspend;
	bool pending_evt;
	spinlock_t lock;
};


/**
 * struct smb345_charger - SMB345 charger driver internal structure
 * @client			pointer to  i2c client's structure
 * @ididev			pointer to idi device
 * @chgint_bh			structure describing bottom half of
 *				CHGINT irq. See structure definition for
 *				details.
 * @boost_op_bh			structure describing bottom half of boost
 *				enable/disable operation.
 * @charging_work		work providing charging heartbeat for PSC
 * @boost_work			work feeding watchdog during boost mode
 * @otg_handle			Pointer to USB OTG internal structure
 *				used for sending VBUS notifications.
 * @usb_psy			power supply instance struct for USB path
 * @ac_psy			power supply instance struct for AC path
 * @current_psy			pointer to psy representing current charging
 *				path.
 * @prop_lock			synchronization semaphore
 * @model_name			model name of charger chip
 * @manufacturer		manufacturer name of charger chip
 * @ack_time			last CONTINUE_CHARGING timestamp in jiffies
 * @otg_nb			OTG notifier block
 * @fake_vbus			value of fake vbus event
 * @state			charger state structure
 */
struct smb345_charger {
	struct i2c_client *client;
	struct idi_peripheral_device *ididev;

	struct unfreezable_bh_struct chgint_bh;
	struct unfreezable_bh_struct boost_op_bh;

	struct delayed_work charging_work;
	struct delayed_work boost_work;
	struct resource *ctrl_io_res;
	struct usb_phy *otg_handle;

	struct power_supply usb_psy;
	struct power_supply ac_psy;

	struct power_supply *current_psy;

	struct semaphore prop_lock;
	struct wake_lock suspend_lock;
	const char *model_name;
	const char *manufacturer;

	unsigned long ack_time;
	struct notifier_block otg_nb;

	int fake_vbus;
	struct smb345_state state;
	int chgint_irq;
	int chgdet_irq;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;

	int chg_otg_en_gpio;
	struct power_supply_cable_props *props;
	unsigned long chgint_debounce;

	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock;
	int usb_acok, dcin_ok;
	int acok_irq, dcin_irq;
	struct work_struct  chrgr_type_work;
	charger_jeita_status_t charger_jeita_status;
};

static int smb345_otg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data);

static struct smb345_charger chrgr_data = {
	.model_name = "SMB345",
	.manufacturer = "TBD",

	.otg_nb = {
		.notifier_call = smb345_otg_notification_handler,
	},

	.chgint_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.boost_op_bh = {
		.in_suspend = false,
		.pending_evt = false,
	},
	.fake_vbus = -1,

	.state = {
		.status = SMB345_STATUS_UNKNOWN,
		.vbus = -1,

	},
	.charger_jeita_status = ROOM,
};
static int smb345_enable_charging(struct smb345_charger *chrgr,
		bool enable, int ma);
static int smb345_i2c_read_reg(struct i2c_client *client,
		u8 reg_addr, u8 *data);

static int smb345_i2c_write_reg(struct i2c_client *client,
		u8 reg_addr, u8 data);

static struct charger_debug_data chrgr_dbg;

#if INTEL_STYLE
static struct power_supply_throttle smb345_dummy_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
	},
};

static char *smb345_supplied_to[] = {
		"battery",
};

static enum power_supply_property smb345_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};
#else
static char *supply_list[] = {
	"battery",
};

static enum power_supply_property asus_power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb345_power_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val);

static struct power_supply smb345_power_supplies[] = {
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
};
#endif

#ifdef SYSFS_FAKE_VBUS_SUPPORT

/**
 * fake_vbus_show	Called when value queried from driver to sysfs
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[out] pointer to buffer that is to be filled with
 *			string. Passed from driver to sysfs
 *
 * Returns:		number of characters read
 */
static ssize_t fake_vbus_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	size_t size_copied;
	int value;

	value = chrgr->fake_vbus;
	size_copied = sprintf(buf, "%d\n", value);

	return size_copied;
}

/**
 * fake_vbus_store	Called when value written from sysfs to driver
 * @dev			[in] pointer to device
 * @attr		[in] pointer to devices's attribute
 * @buf			[in] pointer to buffer containing string passed
 *			from sysfs
 * @count		[in] number of characters in string
 *
 * Returns:		number of characters written
 */
static ssize_t fake_vbus_store(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));
	int sysfs_val;
	int ret;
	size_t size_to_cpy;
	char strvalue[SYSFS_INPUT_VAL_LEN + 1];

	size_to_cpy = (count > SYSFS_INPUT_VAL_LEN) ?
				SYSFS_INPUT_VAL_LEN : count;

	strncpy(strvalue, buf, size_to_cpy);
	strvalue[size_to_cpy] = '\0';

	ret = kstrtoint(strvalue, 10, &sysfs_val);
	if (ret != 0)
		return ret;

	sysfs_val = (sysfs_val == 0) ? 0 : 1;

	down(&chrgr->prop_lock);

	chrgr->fake_vbus = -1;

	if (chrgr->state.vbus != 1) {
		pr_err("fake vbus event requested when USB cable removed !\n");
		up(&chrgr->prop_lock);
		return count;
	}

	if (sysfs_val == 0) {
		chrgr->fake_vbus = 0;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus removal sent.\n");

	} else if (sysfs_val == 1 && !chrgr->state.charging_enabled) {
		chrgr->fake_vbus = 1;
		atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
				USB_EVENT_VBUS, &chrgr->fake_vbus);

		CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_FAKE_VBUS,
						chrgr->fake_vbus, 0);
		pr_info("fake vbus connection sent\n");
	}

	up(&chrgr->prop_lock);

	return count;
}

static struct device_attribute smb345_fake_vbus_attr = {
	.attr = {
		.name = "fake_vbus_event",
		.mode = S_IRUSR | S_IWUSR,
	},
	.show = fake_vbus_show,
	.store = fake_vbus_store,
};

/**
 * smb345_setup_sysfs_attr	Sets up sysfs entries for smb345 i2c device
 * @chrgr			[in] pointer to charger driver internal
 *				structure
 */
static void smb345_setup_fake_vbus_sysfs_attr(struct smb345_charger *chrgr)
{
	struct device *dev = &chrgr->client->dev;
	int err;

	err = device_create_file(dev, &smb345_fake_vbus_attr);
	if (err)
		pr_err("Unable to create sysfs entry: '%s'\n",
				smb345_fake_vbus_attr.attr.name);
}

#else

static inline void smb345_setup_fake_vbus_sysfs_attr(
					struct smb345_charger *chrgr)
{
	(void) chrgr;
}

#endif /*SYSFS_FAKE_VBUS_SUPPORT*/

static int smb345_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	int ret;
	struct smb345_charger *chrgr  = i2c_get_clientdata(client);

	wake_lock(&chrgr->wakelock);
	ret = i2c_smbus_read_byte_data(chrgr->client, reg);
	wake_unlock(&chrgr->wakelock);
	if (ret < 0) {
		dev_warn(&chrgr->client->dev,
				"i2c read fail: can't read reg 0x%02X: %d\n",
				reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int smb345_i2c_write_reg(struct i2c_client *client,
		u8 reg, u8 val)
{
	int ret;
	struct smb345_charger *chrgr = i2c_get_clientdata(client);

	wake_lock(&chrgr->wakelock);
	ret = i2c_smbus_write_byte_data(chrgr->client, reg, val);
	wake_unlock(&chrgr->wakelock);
	if (ret < 0) {
		dev_err(&chrgr->client->dev,
				"i2c write fail: can't write %02X to %02X: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb345_masked_read(struct i2c_client *client, int reg, u8 mask)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = smb345_i2c_read_reg(client, reg, &temp);
        if (rc) {
            retry_count--;
            pr_err("*smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return -1;
    }

    temp &= mask;
    return temp;
}

static int smb345_masked_write(struct i2c_client *client, int reg,
                u8 mask, u8 val)
{
    s32 rc;
    u8 temp;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        rc = smb345_i2c_read_reg(client, reg, &temp);
        if (rc) {
            retry_count--;
            pr_err("*smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_i2c_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    temp &= ~mask;
    temp |= val & mask;

    retry_count = I2C_RETRY_COUNT;
    do
    {
        rc = smb345_i2c_write_reg(client, reg, temp);
        if (rc) {
            retry_count--;
            pr_err("*smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
            msleep(I2C_RETRY_DELAY);
        }
    } while (rc && retry_count > 0);
    if (rc) {
        pr_err("smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
        return rc;
    }

    return 0;
}

static int smb345_set_writable(struct smb345_charger *chrgr, bool writable)
{
#if INTEL_STYLE
	int ret;
	u8 val = 0;

	ret = smb345_i2c_read_reg(chrgr->client, CMD_A, &val);
	if (ret != 0)
		return ret;

	if (writable)
		val |= CMD_A_ALLOW_WRITE;
	else
		val &= ~CMD_A_ALLOW_WRITE;

	return smb345_i2c_write_reg(chrgr->client, CMD_A, val);
#endif
	/* FIXME: WORKAROUND overwrite the value in 30h to only enable write for Z300 */
	return smb345_i2c_write_reg(chrgr->client, CMD_A, CMD_A_ALLOW_WRITE);
}

int smb345_get_soc_control_float_vol(int bat_tempr, int bat_volt)
{
    int ret;

    /* read 03h[5:0]="011110" or "101010"*/
    ret = smb345_masked_read(chrgr_data.client,
                FLOAT_VOLTAGE_REG,
                FLOAT_VOLTAGE_MASK);

    BAT_DBG_E(" Charger get battery info T:%d, V:%d, 03h[5:0]=" BYTETOBINARYPATTERN "\n",
        bat_tempr, bat_volt, BYTETOBINARY(ret));
    return ret;
}

static int __maybe_unused smb345_set_voreg(struct smb345_charger *chrgr,
				int volt_to_set, int *volt_set, bool propagate)
{
	return 0;
}

static int __maybe_unused smb345_set_iocharge(
			struct smb345_charger *chrgr, int curr_to_set,
						int *curr_set, bool propagate)
{
	return 0;
}

static int __maybe_unused smb345_set_ibus_limit(struct smb345_charger *chrgr,
						int ilim_to_set, int *ilim_set)
{
	return 0;
}

static int __maybe_unused smb345_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	return 0;
}

static int __maybe_unused smb345_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	return 0;
}

static void smb345_set_boost(struct work_struct *work)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;
	int on = chrgr->state.to_enable_boost;

	pr_info("%s(): %s\n", __func__, (on) ? "enable" : "disable");

	ret = smb345_set_writable(chrgr, true);
	if (ret < 0)
		goto boost_fail;


#if INTEL_STYLE
	if (on) {
		/* I2C disable OTG function */
		ret = smb345_masked_write(chrgr->client, CMD_A, BIT(4), 0);
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			goto boost_fail;
		}

		/* Switching Frequency change 1.5Mhz 07h[7]="1" */
		ret = smb345_masked_write(chrgr->client, CFG_CHARGE_CURRENT_TC_MASK,
				BIT(7), BIT(7));
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			goto boost_fail;
		}

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG, OTG_CURRENT_LIMIT_AT_USBIN_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			goto boost_fail;
		}

		/* Set OTG battery UVLO threshold to 2.7V: 0Ah[1:0]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG, OTG_BATTERY_UVLO_THRESHOLD_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG battery UVLO threshold to 2.7V ret=%d\n",
					ret);
			goto boost_fail;
		}

		/* Toggle to enable OTG function: output low */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 1);

		/* Set OTG current limit to 500mA: 0Ah[3:2]="01" */
		ret = smb345_masked_write(chrgr->client, OTG_TLIM_THERM_CNTRL_REG,
				BIT(2) | BIT(3), BIT(2));

		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
			goto boost_fail;
		}
	} else {
		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(chrgr->client,
				OTG_TLIM_THERM_CNTRL_REG, OTG_CURRENT_LIMIT_AT_USBIN_MASK, 0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			goto boost_fail;
		}

		/* Toggle to enable OTG function: output high */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 0);
	}
#else
	ret = on ? setSMB345Charger(ENABLE_5V) : setSMB345Charger(DISABLE_5V);
	if (ret)
		goto boost_fail;
#endif

	return;
boost_fail:
	pr_err("%s: fail to boost\n", __func__);
}
/**
 * smb345_chgint_cb_work_func	function executed by
 *				smb345_charger::chgint_cb_work work
 * @work			[in] pointer to associated 'work' structure
 */
static void smb345_chgint_cb_work_func(struct work_struct *work)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	struct smb345_charger *chrgr = &chrgr_data;
	int ret, vbus_state_prev;
	u8 status;
	int i;

	down(&chrgr->prop_lock);
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_CHGINT_CB, 0, 0);

	vbus_state_prev = chrgr->state.vbus;

	pm_state_en = idi_peripheral_device_pm_get_state_handler(chrgr->ididev, "enable");

	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		goto chgint_fail;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "disable");

	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		goto chgint_fail;
	}

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		goto chgint_fail;
	}

#if INTEL_STYLE
	ret = smb345_i2c_read_reg(chrgr->client, STAT_C, &status);
	chrgr->state.vbus = (status & STAT_C_CHG_ENABLED) ? VBUS_ON : VBUS_OFF;
#else
	status = smb345_masked_read(chrgr->client, IRQSTAT_F, BIT(0));
	pr_err("status=0x%02X\n", status);
	chrgr->state.vbus = (status) ? VBUS_ON : VBUS_OFF;
#endif

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false); /*FIXME DO I NEED THIS */
	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
	if (gpio_get_value(chrgr->dcin_ok)) {
#endif
     /* If vbus status changed, then notify USB OTG */
	if (chrgr->state.vbus != vbus_state_prev) {
		if (chrgr->otg_handle) {
			pr_info("%s: vbus changed: %d\n", __func__, chrgr->state.vbus);
			atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
					USB_EVENT_VBUS, &chrgr->state.vbus);

			CHARGER_DEBUG_REL(
					chrgr_dbg, CHG_DBG_VBUS, chrgr->state.vbus, 0);
		}
	}
	else { // FIXME: workaround for Z380C due to Charger IC does not imediately update STAT_C information
		for (i=0; i<20; i++) {
			msleep(50);
			ret = smb345_i2c_read_reg(chrgr->client, IRQSTAT_F, &status);
			if (!ret) {
				chrgr->state.vbus = (status & STAT_C_CHG_ENABLED) ? VBUS_ON : VBUS_OFF;
				pr_info("status=0x%02x\n", status);
				if (chrgr->state.vbus != vbus_state_prev) {
					if (chrgr->otg_handle) {
						pr_info("%s: *** vbus changed(%d): %d\n", __func__, i, chrgr->state.vbus);
						atomic_notifier_call_chain(&chrgr->otg_handle->notifier,
								USB_EVENT_VBUS, &chrgr->state.vbus);

						CHARGER_DEBUG_REL(
							chrgr_dbg, CHG_DBG_VBUS, chrgr->state.vbus, 0);
						break;
					}
				}
			}
	    }
	}
#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
	}
	else {
		pr_info("%s: ignore informing USB with DCIN from Stand/Cover power\n",
			__func__);
	}
#endif

	/* XXX: CHGDET configuration */
	ret = idi_client_ioread(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), &regval);

	/* charger detection CHARGER_CONTROL_CHDETLVL*/
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);

	if (chrgr->state.vbus) {
		regval |= (CHARGER_CONTROL_CHDETLVL_HIGH << CHARGER_CONTROL_CHDETLVL_O);
	} else {
		regval |= (CHARGER_CONTROL_CHDETLVL_LOW << CHARGER_CONTROL_CHDETLVL_O);
	}

	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), regval);

	/* charger control WR strobe */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	/* This second triggering of the write strobe is intentional
	 * to make sure CHARGER_CTRL value is propagated to HW properly
	 */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

chgint_fail:
	up(&chrgr->prop_lock);
}

bool POWER_OK(void)
{
    int ret;
    u8 value = 0;

    ret = smb345_i2c_read_reg(chrgr_data.client, IRQSTAT_F, &value);
    if (!ret) {
        BAT_DBG(" %s: status=0x%02X\n", __func__, value);
        return ((value & BIT(0)) ? true : false);
    }

    BAT_DBG_E(" %s: failed to check POWER_OK. charger I2C error!\n", __func__);
    return false;
}
EXPORT_SYMBOL_GPL(POWER_OK);

static int unfreezable_bh_create(struct unfreezable_bh_struct *bh,
		const char *wq_name, const char *wakelock_name,
		void (*work_func)(struct work_struct *work))
{
	spin_lock_init(&bh->lock);

	/* Create private, single-threaded workqueue instead of using one of
	the system predefined workqueues to reduce latency */
	bh->wq = create_singlethread_workqueue(wq_name);

	if (NULL == bh->wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&bh->work, work_func);

	wake_lock_init(&bh->evt_wakelock,
			WAKE_LOCK_SUSPEND,
			wakelock_name);

	return 0;
}

static void unfreezable_bh_destroy(struct unfreezable_bh_struct *bh)
{
	cancel_delayed_work_sync(&bh->work);

	destroy_workqueue(bh->wq);

	wake_lock_destroy(&bh->evt_wakelock);
}

static void unfreezable_bh_schedule(struct unfreezable_bh_struct *bh)
{
	struct smb345_charger *chrgr = &chrgr_data;
	spin_lock(&bh->lock);

	if (!bh->in_suspend) {
		queue_delayed_work(bh->wq, &bh->work, chrgr->chgint_debounce);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	} else {
		bh->pending_evt = true;
	}

	spin_unlock(&bh->lock);
}

static void __maybe_unused unfreezable_bh_resume(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;
	struct smb345_charger *chrgr = &chrgr_data;

	spin_lock_irqsave(&bh->lock, flags);

	bh->in_suspend = false;
	if (bh->pending_evt) {
		bh->pending_evt = false;

		queue_delayed_work(bh->wq, &bh->work, chrgr->chgint_debounce);

		wake_lock_timeout(&bh->evt_wakelock, EVT_WAKELOCK_TIMEOUT);
	}

	spin_unlock_irqrestore(&bh->lock, flags);
}

static void __maybe_unused unfreezable_bh_suspend(struct unfreezable_bh_struct *bh)
{
	unsigned long flags;

	spin_lock_irqsave(&bh->lock, flags);
	bh->in_suspend = true;
	spin_unlock_irqrestore(&bh->lock, flags);
}

static irqreturn_t smb345_charger_chgint_cb(int irq, void *dev)
{
	struct smb345_charger *chrgr = dev;

	if (g_usb_state == ENABLE_5V) {
		pr_info("OTG cable attached. Skip Vbus notification\n");
		return IRQ_HANDLED;
	}
	pr_info("*********************** USB Connector ***********************\n");
	unfreezable_bh_schedule(&chrgr->chgint_bh);
	return IRQ_HANDLED;
}

/**
 * smb345_configure_pmu_irq - function configuring PMU's Charger status IRQ
 * @chrgr		[in] pointer to charger driver internal structure
 */
static int smb345_configure_pmu_irq(struct smb345_charger *chrgr)
{
	int ret;

#if 0
	/* register callback with PMU for CHGINT irq */
	ret = request_irq(chrgr->chgint_irq,
		smb345_charger_chgint_cb,
			IRQF_NO_SUSPEND, SMB345_NAME, chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}
#endif
	/* register callback with PMU for CHGDET irq */
	ret = request_irq(chrgr->chgdet_irq,
		smb345_charger_chgint_cb,
			IRQF_NO_SUSPEND, SMB345_NAME, chrgr);
	if (ret != 0) {
		pr_err("Failed to register @PMU for GHGINT irq! ret=%d", ret);
		return ret;
	}

	return 0;
}

static int smb345_configure_pmu_regs(struct smb345_charger *chrgr)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	if (!chrgr->ctrl_io_res || !chrgr->ididev)
		return -EINVAL;

	pm_state_en =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(
				chrgr->ididev, "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	ret = idi_client_ioread(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), &regval);


	/* ChargerResetLevel - CHARGER_CONTROL_CHGLVL_LOW */
	regval &= ~(CHARGER_CONTROL_CHGLVL_M << CHARGER_CONTROL_CHGLVL_O);
	regval |= (CHARGER_CONTROL_CHGLVL_LOW << CHARGER_CONTROL_CHGLVL_O);

	/* charger detection CHARGER_CONTROL_CDETSENS*/
	regval &= ~(CHARGER_CONTROL_CDETSENS_M << CHARGER_CONTROL_CDETSENS_O);
	regval |= (CHARGER_CONTROL_CDETSENS_EDGE << CHARGER_CONTROL_CDETSENS_O);

	/* charger detection CHARGER_CONTROL_CHDETLVL*/
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);
	regval |= (CHARGER_CONTROL_CHDETLVL_LOW << CHARGER_CONTROL_CHDETLVL_O);

	/* charger detection CHARGER_CONTROL_CHDWEN*/
	regval &= ~(CHARGER_CONTROL_CHDWEN_M << CHARGER_CONTROL_CHDWEN_O);
	regval |= (CHARGER_CONTROL_CHDWEN_EN << CHARGER_CONTROL_CHDWEN_O);

	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL(chrgr->ctrl_io_res->start), regval);

	/* charger control WR strobe */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	/* This second triggering of the write strobe is intentional
	 * to make sure CHARGER_CTRL value is propagated to HW properly
	 */
	ret = idi_client_iowrite(chrgr->ididev,
			CHARGER_CONTROL_WR(chrgr->ctrl_io_res->start),
			BIT(CHARGER_CONTROL_WR_WS_O));

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return 0;
}

static void do_charger(struct work_struct *work)
{
	struct smb345_charger *chrgr = &chrgr_data;

	down(&chrgr->prop_lock);

	pr_info("%s(), ma = %u, type = %d, connect = %d\n",
			__func__, chrgr->props->ma, chrgr->props->chrg_type,
			chrgr->props->chrg_evt);

	if (chrgr->props->chrg_evt == POWER_SUPPLY_CHARGER_EVENT_CONNECT) {
		chrgr->state.cable_type = chrgr->props->chrg_type;
		switch (chrgr->props->chrg_type) {
		case POWER_SUPPLY_CHARGER_TYPE_NONE:
			break;

		case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
			setSMB345Charger(USB_IN);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
			smb345_enable_charging(chrgr, true, chrgr->props->ma);
			setSMB345Charger(AC_IN);

			break;

		default:
			break;

		}
	} else if (chrgr->props->chrg_evt
			== POWER_SUPPLY_CHARGER_EVENT_DISCONNECT) {
		smb345_enable_charging(chrgr, false, 0);
		setSMB345Charger(CABLE_OUT);
	}
	up(&chrgr->prop_lock);
}

static int smb345_write(struct smb345_charger *chrgr, u8 reg, u8 val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		wake_lock(&chrgr->wakelock);
		ret = i2c_smbus_write_byte_data(chrgr->client, reg, val);
		wake_unlock(&chrgr->wakelock);
		if (ret < 0) {
			retry_count--;
			dev_warn(&chrgr->client->dev,
					"fail to write reg %02xh: %d\n",
					reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb345_read(struct smb345_charger *chrgr, u8 reg)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		wake_lock(&chrgr->wakelock);
		ret = i2c_smbus_read_byte_data(chrgr->client, reg);
		wake_unlock(&chrgr->wakelock);
		if (ret < 0) {
			retry_count--;
			dev_warn(&chrgr->client->dev, "fail to read reg %02xh: %d\n",
					reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}
static int smb345_set_current_limits(struct smb345_charger *chrgr,
		int current_limit, bool is_twinsheaded)
{
	int ret;
	ret = smb345_set_writable(chrgr, true);
	if (ret < 0) {
		pr_info("%s() set writable as true failed, return %d\n",
				__func__, ret);
		return ret;
	}

	pr_info("%s()-cable_type = %d, current_limit = %d\n",
			__func__, chrgr->state.cable_type, current_limit);
	if (chrgr->state.cable_type == POWER_SUPPLY_CHARGER_TYPE_USB_SDP
			&& current_limit > 500) {
		pr_info("USB IN but current limit > 500, return\n");
		return 0;
	}

	switch (current_limit) {
	case 500:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_500);
		break;
	case 700:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_700);
		break;
	case 1000:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1000);
		break;

	case 1200:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1200);
		break;

	case 1500:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1500);
		break;

	case 1800:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_1800);
		break;

	case 2000:
		ret = smb345_masked_write(chrgr->client,
					CFG_CURRENT_LIMIT,
					CFG_CURRENT_LIMIT_SMB346_MASK,
					CFG_CURRENT_LIMIT_SMB346_VALUE_2000);
		break;
	default:
		break;
	}
	return ret;
}

static int smb345_enable_charging(struct smb345_charger *chrgr,
		bool enable, int ma)
{
	int ret = 0;

#if !INTEL_STYLE
	return ret;
#endif

	ret = smb345_set_writable(chrgr, true);
	if (ret < 0) {
		pr_err("%s() set writable as true failed, return %d\n",
				__func__, ret);
		goto fail;
	}

	pr_info("%s(), enable = %d for type=%d, ma=%d\n",
			__func__, enable, chrgr->state.cable_type, ma);
	if (enable == true) {
		if (ma < 500)
			return 0;

		/* config charge voltage as 4.3v */
		ret = smb345_masked_write(chrgr->client,
					FLOAT_VOLTAGE_REG,
					BIT(0) | BIT(1) | BIT(2) |
					BIT(3) | BIT(4) | BIT(5),
					BIT(0) | BIT(1) | BIT(5));


		/* charger enable */
		ret = smb345_read(chrgr, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CTRL_MASK;
		if (enable) {
			/*
			 * Set Pin active low
			 * (ME371MG connect EN to GROUND)
			 */
			ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
		} else {
		}
		ret = smb345_write(chrgr, CFG_PIN, ret);
		if (ret < 0)
			goto fail;


		if (smb345_set_current_limits(chrgr, ma, false) < 0) {
			dev_err(&chrgr->client->dev,
				"%s: fail to set max current limits\n",
				__func__);
			goto fail;
		}

#if INTEL_STYLE
		if (chrgr->state.cable_type ==
				POWER_SUPPLY_CHARGER_TYPE_USB_SDP) {
			power_supply_changed(&chrgr->usb_psy);
		} else if (chrgr->state.cable_type
				== POWER_SUPPLY_CHARGER_TYPE_USB_CDP ||
				chrgr->state.cable_type
				== POWER_SUPPLY_CHARGER_TYPE_USB_DCP) {
			power_supply_changed(&chrgr->ac_psy);
		}
#endif

	} else {
#if INTEL_STYLE
		power_supply_changed(&chrgr->ac_psy);
		power_supply_changed(&chrgr->usb_psy);
#endif
	}

fail:
	return ret;
}

static int smb345_otg_notification_handler(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct smb345_charger *chrgr = &chrgr_data;
	chrgr->props = (struct power_supply_cable_props *)data;

	switch (event) {
	case USB_EVENT_CHARGER:
		queue_delayed_work(charger_work_queue, &charger_work, 0);
		break;
	case INTEL_USB_DRV_VBUS:
		pr_info("%s() INTEL_USB_DRV_VBUS\n", __func__);
		if (!data)
			return NOTIFY_BAD;
		chrgr->state.to_enable_boost = *((bool *)data);
		unfreezable_bh_schedule(&chrgr->boost_op_bh);
		break;

	default:
		break;
	}
	return NOTIFY_OK;
}

static int smb345_set_pinctrl_state(struct i2c_client *client,
		struct pinctrl_state *state)
{
	struct smb345_charger *chrgr = i2c_get_clientdata(client);
	int ret;

	ret = pinctrl_select_state(chrgr->pinctrl, state);
	if (ret != 0) {
		pr_err("failed to configure CHGRESET pin !\n");
		return -EIO;
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/* USBIN suspend mode */
static bool suspend_mode(bool toggle)
{
	int ret;

	if (!chrgr_data.client) {
		pr_err("Warning: I2C client is null\n");
		return false;
	}

	pr_info("%s:", __func__);
	ret = smb345_set_writable(&chrgr_data, true);

	if (toggle) {
		/* enter suspend mode                                                 */
		/* Input to System FET (Suspend) On/Off Control 02h[7]="1"            */
		/* USBIN Suspend Mode 30h[2]="1"                                      */
		/* DCIN Suspend Mode 31h[2]="1"                                       */
		ret = smb345_masked_write(chrgr_data.client,
			0x02,
			BIT(7),
			BIT(7));
		if (ret) {
			pr_err("fail to set Input to System FET (Suspend) ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x30,
			BIT(2),
			BIT(2));
		if (ret) {
			pr_err("fail to set USBIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x31,
			BIT(2),
			BIT(2));
		if (ret) {
			pr_err("fail to set DCIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
	}
	else {
		/* leave suspend mode                                                 */
		/* Input to System FET (Suspend) On/Off Control 02h[7]="1"            */
		/* USBIN Suspend Mode 30h[2]="0"                                      */
		/* DCIN Suspend Mode 31h[2]="0"                                       */
		ret = smb345_masked_write(chrgr_data.client,
			0x02,
			BIT(7),
			BIT(7));
		if (ret) {
			pr_err("fail to set Input to System FET (Suspend) ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x30,
			BIT(2),
			0);
		if (ret) {
			pr_err("fail to set USBIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
		ret = smb345_masked_write(chrgr_data.client,
			0x31,
			BIT(2),
			0);
		if (ret) {
			pr_err("fail to set DCIN Suspend Mode ret=%d\n", ret);
			return ret;
		}
	}
}


/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
{
	if (val >= size)
		return tbl[size-1];;
	return tbl[val];
}

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct smb345_charger *chrgr = i2c_get_clientdata(to_i2c_client(dev));

	ret = smb345_read(chrgr, STAT_E);
	if (ret < 0) {
		pr_info("%s: ERROR: i2c read error\n", __func__);
		return sprintf(buf, "%d\n", -EIO);
	}

	ret &= 0x0F;
	return sprintf(buf, "%d\n",
			hw_to_current(aicl_results, ARRAY_SIZE(aicl_results), ret));
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	ret = smb345_get_charging_status();
	if (ret == POWER_SUPPLY_STATUS_CHARGING || ret == POWER_SUPPLY_STATUS_FULL)
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_input_current.attr,
	&dev_attr_charge_status.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

static void aicl_cur_control(int usb_state)
{
	int aicl_result;
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;

	if (usb_state != AC_IN)
		return;

	/* check if AICL result > 500mA. 3Fh[3:0]="0001" . Do nothing if true */
	if (smb345_masked_read(chrgr->client, 0x3f, 0x0f) > 0x01)
		return;

	/* Disable AICL - Write 02h[4]="0" */
	ret = smb345_masked_write(chrgr->client,
		0x02,
		BIT(4),
		0);
	if (ret) {
		pr_err("fail to disable AICL. ret=%d\n", ret);
		return;
	}

	/* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" (smb345, smb346, smb347)
							- Write 01h[7:4]="0100" (smb358) */
	ret = smb345_masked_write(chrgr->client,
		0x01,
		BIT(3) | BIT(2) | BIT(1) | BIT(0),
		BIT(2));
	if (ret) {
		pr_err("fail to Set I_USB_IN=1200mA. ret=%d\n", ret);
		return;
	}

	/* Enable AICL - Write 02h[4]="1" */
	ret = smb345_masked_write(chrgr->client,
		0x02,
		BIT(4),
		BIT(4));
	if (ret) {
		pr_err("fail to enable AICL. ret=%d\n", ret);
		return;
	}
}

int smb345_charging_toggle(charging_toggle_level_t level, bool on)
{
    int ret = 0;
    int charging_toggle;
    static charging_toggle_level_t old_lvl = JEITA;
    char *level_str[] = {
        "BALANCE",
        "JEITA",
        "FLAGS",
    };
	struct smb345_charger *chrgr = &chrgr_data;
    charging_toggle = g_charging_toggle;

    BAT_DBG("%s: old_lvl:%s, charging_toggle:%s, level:%s, on:%s\n",
        __func__,
        level_str[old_lvl],
        charging_toggle ? "YES" : "NO",
        level_str[level],
        on ? "YES" : "NO");

    /* do charging or not? */
    if (level != FLAGS) {
        if (on) {
            /* want to start charging? */
            if (level == JEITA) {
                if (!charging_toggle) {
                    /* want to restart charging? */
                    if (old_lvl != JEITA) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *\n",
                            __func__);
                        return -1;
                    }
                }
            }
            else if (level == BALANCE) {
                if (!charging_toggle) {
                    /* want to restart charging? */
                    if (old_lvl != BALANCE) {
                        /* reject the request! someone stop charging before */
                        BAT_DBG_E("%s: * reject RESTART charging to phone! *\n",
                        __func__);
                        return -1;
                    }
                }
            }
            else {
                /* what the hell are you? */
            }
        }
        else {
            /* want to stop charging? just do it! */
        }
    }
    else {
        /* it's the highest level. just do it! */
    }

    /* level value assignment */
    old_lvl = level;

    if (!on)
        BAT_DBG_E(" %s: *** charging toggle: OFF ***\n",
            __func__);
    else
        BAT_DBG(" %s: --------------- charging toggle: ON ---------------\n",
            __func__);

    ret = smb345_set_writable(chrgr, true);
    if (ret < 0)
        return ret;

    /* Config CFG_PIN register */
    ret = smb345_read(chrgr, CFG_PIN);
    if (ret < 0)
        goto out;

    /*
     * Make the charging functionality controllable by a write to the
     * command register unless pin control is specified in the platform
     * data.
     */
    ret &= ~CFG_PIN_EN_CTRL_MASK;
    if (on) {
        /* set Pin Controls - active low (ME371MG connect EN to GROUND) */
        ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
    } else {
        /* Do nothing, 0 means i2c control
            . I2C Control - "0" in Command Register disables charger */
    }

    ret = smb345_write(chrgr, CFG_PIN, ret);
    if (ret < 0)
        goto out;

    g_charging_toggle = on;

out:
    return ret;
}

bool smb345_has_charger_error(void)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;

	ret = smb345_read(chrgr, STAT_C);
	if (ret < 0)
		return true;

	if (ret & STAT_C_CHARGER_ERROR)
		return true;

	return false;
}

int smb345_get_charging_status(void)
{
	int ret, status;
	int irqstat_c;
	struct smb345_charger *chrgr = &chrgr_data;

	ret = smb345_read(chrgr, STAT_C);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	irqstat_c = smb345_read(chrgr, IRQSTAT_C);
	if (irqstat_c < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	dev_info(&chrgr->client->dev,
		"Charging Status: STAT_C:0x%x\n", ret);

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
    }

	return status;
}

static int smb345_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
	int ret = 0;
	int usb_state;
	int chrg_status;

	usb_state = g_usb_state;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = (usb_state == USB_IN) ? 1 : 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = (usb_state == AC_IN) ? 1 : 0;
		} else {
			ret = -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			/* for ATD test to acquire the status about charger ic */
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}

			chrg_status = smb345_get_charging_status();
			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			} else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}

			chrg_status = smb345_get_charging_status();
			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			}
			else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int smb345_register_power_supply(struct device *dev)
{
	int ret;

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_USB-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply USB\n");
		goto batt_err_reg_fail_usb;
	}

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_AC-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply AC\n");
		goto batt_err_reg_fail_ac;
	}

	g_is_power_supply_registered = true;
	BAT_DBG_E("%s(): done\n", __func__);
	return 0;

batt_err_reg_fail_ac:
	power_supply_unregister(&smb345_power_supplies[CHARGER_USB-1]);
batt_err_reg_fail_usb:
	return ret;
}

int smb345_dump_registers(struct seq_file *s)
{
	struct smb345_charger *smb;
	int ret;
	u8 reg;

	smb = &chrgr_data;

	BAT_DBG(" %s:\n", __func__);
	BAT_DBG(" Control registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");

	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Command registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Command registers:\n");
		seq_printf(s, "==================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}

	ret = smb345_read(smb, CMD_A);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_A, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_A, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_B);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_B, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_C);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_C, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_C, BYTETOBINARY(ret));
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Interrupt status registers:\n");
	BAT_DBG(" ===========================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Interrupt status registers:\n");
		seq_printf(s, "===========================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Status registers:\n");
	BAT_DBG(" =================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Status registers:\n");
		seq_printf(s, "=================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}

static int smb345_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");

	smb345_dump_registers(s);

	return 0;
}

static int smb345_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb345_debugfs_show, inode->i_private);
}

static const struct file_operations smb345_debugfs_fops = {
	.open        = smb345_debugfs_open,
	.read        = seq_read,
	.llseek      = seq_lseek,
	.release     = single_release,
};

int smb345_soc_control_jeita(void)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;

	pr_info("%s:", __func__);
	ret = smb345_set_writable(chrgr, true);
	if (ret < 0)
		return ret;

	/* write 0bh[5:4]="11" */
	ret = smb345_masked_write(chrgr->client,
		0x0b,
		BIT(5) | BIT(4),
		BIT(5) | BIT(4));
	if (ret) {
		pr_err("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	/* write 07h[1:0]="00" */
	ret = smb345_masked_write(chrgr->client,
		0x07,
		BIT(1) | BIT(0),
		0);
	if (ret) {
		pr_err("fail to set Soft Hot Limit Behavior to No Response ,ret=%d\n", ret);
		return ret;
	}

	/* write 07h[3:2]="00" */
	ret = smb345_masked_write(chrgr->client,
		0x07,
		BIT(3) | BIT(2),
		0);
	if (ret) {
		pr_err("fail to set SOFT_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	return ret;
}

int smb345_charger_control_jeita(void)
{
    int ret;

    BAT_DBG(" %s:", __func__);

    /* Set Hard Hot Limit as 53 Deg. C 0Bh[5:4]="00" */
    ret = smb345_masked_write(chrgr_data.client,
            0x0B,
            BIT(5) | BIT(4),
            0);

    /* Set Soft Hot Limit Behavior as Float Voltage Compensation 07h[1:0]="10" */
    ret = smb345_masked_write(chrgr_data.client,
            0x07,
            BIT(1) | BIT(0),
            BIT(1));

    /* Set Soft Hot Limit Behavior as Float Voltage Compensation 07h[3:2]="01" */
    ret = smb345_masked_write(chrgr_data.client,
            0x07,
            BIT(3) | BIT(2),
            BIT(2));

    /* Charging Enable */
    ret = smb345_charging_toggle(JEITA, true);

    /* set Recharge Voltage = Vflt-100mV 04h[3]="1" */
    ret = smb345_masked_write(chrgr_data.client,
            0x04,
            BIT(3),
            BIT(3));

    if (ret)
        pr_err("%s: i2c communication failure!", __func__);

    return ret;
}

int smb3xx_jeita_control(int bat_tempr, int bat_volt)
{
    static charger_jeita_status_t ori_charger_jeita_status = ROOM;
    int ret;

    if (screen_off) {
        BAT_DBG(" %s: Screen Off. Charger Control HW JEITA\n", __func__);
        smb345_charger_control_jeita();
        goto others;
    }

    smb345_soc_control_jeita();

    ori_charger_jeita_status = chrgr_data.charger_jeita_status;

    /* condition judgement 1: */
    if (ori_charger_jeita_status == FREEZE) {
        if (bat_tempr >= 45)
            /* control diagram II: */
            goto control_diagram_II;
        else
            /* control diagram I: */
            goto control_diagram_I;
    }

    /* condition judgement 2: */
    if (ori_charger_jeita_status == COLD) {
        if (bat_tempr >= 180) {
            /* control diagram III: */
            goto control_diagram_III;
        }
        else if (bat_tempr < 15) {
            /* control diagram I: */
            goto control_diagram_I;
        }
        else {
            /* control diagram II: */
            goto control_diagram_II;
        }
    }

    /* condition judgement 3: */
    if (ori_charger_jeita_status == ROOM) {
        if (bat_tempr < 150) {
            /* control diagram II: */
            goto control_diagram_II;
        }
        else if (bat_tempr >= 500) {
            if (bat_volt >= 4100) {
                /* control diagram V: */
                goto control_diagram_V;
            }
            else {
                /* control diagram IV: */
                goto control_diagram_IV;
            }
        }
        else
            /* control diagram III: */
            goto control_diagram_III;
    }

    /* condition judgement 4: */
    if (ori_charger_jeita_status == HOT) {
        // state before is "Charging Disable"
        if (smb345_get_soc_control_float_vol(bat_tempr, bat_volt) == 0x2A) {
            if (bat_tempr < 470) {
                /* control diagram III: */
                goto control_diagram_III;
            }
            else if (bat_tempr > 550) {
                /* control diagram VI: */
                goto control_diagram_VI;
            }
            else {
                if (bat_volt >= 4100) {
                    /* control diagram V: */
                    goto control_diagram_V;
                }
                else {
                    /* control diagram IV: */
                    goto control_diagram_IV;
                }
            }
        }
        else { // state before is "Charging Enable"
            if (bat_tempr < 470) {
                /* control diagram III: */
                goto control_diagram_III;
            }
            else if (bat_tempr > 550) {
                /* control diagram VI: */
                goto control_diagram_VI;
            }
            else {
                /* control diagram IV: */
                goto control_diagram_IV;
            }
        }
    }

    /* condition judgement 5: */
    if (ori_charger_jeita_status == OVERHEAT) {
        if (bat_tempr < 520) {
            /* control diagram IV: */
            goto control_diagram_IV;
        }
        else {
            /* control diagram VI: */
            goto control_diagram_VI;
        }
    }

    BAT_DBG_E(" %s: *** ERROR ***\n", __func__);
    goto others;

control_diagram_I:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7) |          BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    chrgr_data.charger_jeita_status = FREEZE;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_II:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    chrgr_data.charger_jeita_status = COLD;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_III:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    chrgr_data.charger_jeita_status = ROOM;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_IV:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
                     BIT(4) | BIT(3) | BIT(2) | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, true);
    chrgr_data.charger_jeita_status = HOT;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_CHARGING;

control_diagram_V:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    chrgr_data.charger_jeita_status = HOT;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

control_diagram_VI:
    ret = smb345_masked_write(chrgr_data.client,
            0x03,
            BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
            BIT(5)          | BIT(3)          | BIT(1));
    ret = smb345_masked_write(chrgr_data.client,
            0x00,
            BIT(7) | BIT(6) | BIT(5),
            BIT(7)          | BIT(5));
    ret = smb345_charging_toggle(JEITA, false);
    chrgr_data.charger_jeita_status = OVERHEAT;
    if (ori_charger_jeita_status != chrgr_data.charger_jeita_status)
        BAT_DBG(" %s: ori_JEITA: %d, new_JEITA: %d, %d(0.1C), %dmV\n",
            __func__,
            ori_charger_jeita_status,
            chrgr_data.charger_jeita_status, bat_tempr, bat_volt);
    return POWER_SUPPLY_STATUS_DISCHARGING;

others:
    return POWER_SUPPLY_STATUS_CHARGING;
}

static void smb3xx_config_max_current(int usb_state)
{
	struct smb345_charger *chrgr = &chrgr_data;

	if (usb_state == AC_IN) {
		if (!gpio_get_value(chrgr->usb_acok)) {
			/* check if Max USBIN input current limit is 1200mA 01h[3:0]="0100" */
			if (smb345_masked_read(chrgr->client, 0x01, 0x0f) < 0x04) {

				/* Disable AICL - Write 02h[4]="0" */
				if (smb345_masked_write(chrgr->client, 0x02, BIT(4), 0x00)) {
					dev_err(&chrgr->client->dev,
					"%s: fail to disable AICL\n", __func__);
					return;
				}

				/* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" */
				if (smb345_masked_write(chrgr->client, 0x01, 0x0f, 0x04)) {
					dev_err(&chrgr->client->dev,
					"%s: fail to set max current limits for USB_IN\n",
					__func__);
					return;
				}

				/* Enable AICL - Write 02h[4]="1" */
				if (smb345_masked_write(chrgr->client, 0x02, BIT(4), 0x10)) {
					dev_err(&chrgr->client->dev,
					"%s: fail to enable AICL\n", __func__);
					return;
				}
			}
        }
        else if (!gpio_get_value(chrgr->dcin_ok)) {
			/* check if Max DCIN input current limit is 900mA 01h[7:4]="0011" */
			if (smb345_masked_read(chrgr->client, 0x01, 0xf0) < 0x30) {

				/* Disable AICL - Write 02h[4]="0" */
				if (smb345_masked_write(chrgr->client, 0x02, BIT(4), 0x00)) {
					dev_err(&chrgr->client->dev,
					"%s: fail to disable AICL\n", __func__);
					return;
				}

				/* Set I_DC_IN=1200mA - Write 01h[7:4]="0011" */
				if (smb345_masked_write(chrgr->client, 0x01, 0xf0, 0x30)) {
					dev_err(&chrgr->client->dev,
					"%s: fail to set max current limits for USB_IN\n",
					__func__);
					return;
				}

				/* Enable AICL - Write 02h[4]="1" */
				if (smb345_masked_write(chrgr->client, 0x02, BIT(4), 0x10)) {
					dev_err(&chrgr->client->dev,
					"%s: fail to enable AICL\n", __func__);
					return;
				}
			}
		}
		else {
			BAT_DBG(" %s: ============================ what the hell are you? ============================\n"
				,__func__);
		}
	}
}

#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
static int smb3xx_pre_config(void)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;

	if (!gpio_get_value(chrgr->usb_acok)) {
		/* set Input Source Priority to USBIN. 02h[2]="1" */
		ret = smb345_masked_write(chrgr->client,
			0x02,
			BIT(2),
			BIT(2));
		if (ret < 0)
			goto fail;
	}

	/* set fast charge current: 2000mA */
	/* set pre charge current: 150mA */
	/* set termination current: 250mA */
	ret = smb345_write(chrgr,
			0x00,
			0xad);
	if (ret < 0)
		goto fail;

	/* set cold soft limit current: 900mA write 0Ah[7:6]="10"*/
	ret = smb345_masked_write(chrgr->client,
			0x0a,
			BIT(7) | BIT(6),
			BIT(7));
	if (ret < 0)
		goto fail;

	/* set Battery OV does not end charge cycle. 02h[1]="0" */
	ret = smb345_masked_write(chrgr->client,
			0x02,
			BIT(1),
			0);
	if (ret < 0)
		goto fail;

	/* set Float Voltage: 4.34V. 03h[5:0]="101010" */
	ret = smb345_masked_write(chrgr->client,
			0x03,
			BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0),
			BIT(5) | BIT(3) | BIT(1));
	if (ret < 0)
		goto fail;

fail:
	return ret;
}
#else
static int smb3xx_pre_config() { return 0; }
#endif

static void smb345_config_max_current(int usb_state)
{
	struct smb345_charger *chrgr = &chrgr_data;

	if (usb_state != AC_IN && usb_state != USB_IN)
		return;

	if (smb345_set_writable(chrgr, true) < 0) {
		BAT_DBG("%s: smb345_set_writable failed!\n", __func__);
		return;
	}

	smb3xx_pre_config();
	smb3xx_config_max_current(usb_state);
	smb345_soc_control_jeita();

	pr_info("%s: charger type:%d done.\n", __func__, usb_state);
}

#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG) || defined(CONFIG_Z380C)
static int config_otg_regs(int toggle)
{
	int ret;
	struct smb345_charger *chrgr = &chrgr_data;

	if (toggle) {
		/* Set OTG current limit to 250mA 0Ah[3:2]="01" */
		ret = smb345_masked_write(chrgr->client,
			0x0A,
			BIT(3) | BIT(2),
			BIT(2));
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Enable OTG function. 09h[7:6]="01"*/
		ret = smb345_masked_write(chrgr->client,
			0x09,
			BIT(7) | BIT(6),
			BIT(6));
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			return ret;
		}

		/* Toggle to enable OTG function: */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 1);

		/* Set OTG current limit to 500mA 0Ah[3:2]="10" */
		ret = smb345_masked_write(chrgr->client,
			0x0A,
			BIT(3) | BIT(2),
			BIT(3));
		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
			return ret;
		}
	}
	else {
		/* disable OTG */
		/* Set OTG current limit to 250mA 0Ah[3:2]="01" */
		ret = smb345_masked_write(chrgr->client,
			0x0A,
			BIT(3) | BIT(2),
			BIT(2));
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Toggle to disable OTG function: */
		gpio_direction_output(chrgr->chg_otg_en_gpio, 0);
	}

	return ret;
}
#else
static int config_otg_regs(int toggle) { return 0; }
#endif

static int otg(int toggle)
{
	int ret;

    ret = smb345_set_writable(&chrgr_data, true);
    if (ret < 0)
        return ret;

	ret = config_otg_regs(toggle);
	if (ret < 0)
		return ret;

	return 0;
}

int setSMB345Charger(int usb_state)
{
	int ret = 0;

	if (g_is_power_supply_registered) {
		switch (usb_state){
		case USB_IN:
			power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
			break;
		case AC_IN:
			power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			break;
		case CABLE_OUT:
			power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
			break;
		}
	}

	switch (usb_state)
	{
	case USB_IN:
		BAT_DBG(" usb_state: USB_IN\n");
		usb_to_battery_callback(USB_PC);

		BAT_DBG(" %s: config current when USB_IN\n", __func__);
		smb345_config_max_current(USB_IN);
		break;
	case AC_IN:
		BAT_DBG(" usb_state: AC_IN\n");
		usb_to_battery_callback(USB_ADAPTER);

		BAT_DBG(" %s: config current when AC_IN\n", __func__);
		smb345_config_max_current(AC_IN);
		break;
	case CABLE_OUT:
		BAT_DBG(" usb_state: CABLE_OUT\n");
		usb_to_battery_callback(NO_CABLE);
		break;
	case ENABLE_5V:
		BAT_DBG(" usb_state: ENABLE_5V\n");

		spin_lock(&spinlock_suspend);
		if (suspend_smb) {
			suspend_otg_work_not_done = true;
			BAT_DBG(" %s: ******************* postpone turn on OTG 5V to resume function *******************\n", __func__);
			spin_unlock(&spinlock_suspend);
			break;
		}
		spin_unlock(&spinlock_suspend);

		ret = otg(1);
		break;
	case DISABLE_5V:
		BAT_DBG(" usb_state: DISABLE_5V\n");

		spin_lock(&spinlock_suspend);
		if (suspend_smb) {
			suspend_otg_work_not_done = true;
			BAT_DBG(" %s: ******************* postpone turn off OTG 5V to resume function *******************\n", __func__);
			spin_unlock(&spinlock_suspend);
			break;
		}
		spin_unlock(&spinlock_suspend);

		ret = otg(0);
		break;
	default:
		BAT_DBG(" ERROR: wrong usb state value = %d\n", usb_state);
		ret = 1;
	}

	return ret;
}

void chrgr_type_work_func(struct work_struct *work)
{ setSMB345Charger(g_usb_state); }

static int charger_type_report(struct notifier_block *nb, unsigned long event, void *ptr)
{
    switch (event) {
    case POWER_SUPPLY_CHARGER_TYPE_NONE:
        BAT_DBG_E("-----------------------------------------------TYPE_NONE\n");
        /* wake lock to prevent system instantly
           enter S3 while it's in resuming flow */
        wake_lock_timeout(&chrgr_data.wakelock, HZ);

        g_usb_state = CABLE_OUT;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
        BAT_DBG_E("-----------------------------------------------USB_SDP\n");
        g_usb_state = USB_IN;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
    case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
    case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
    case POWER_SUPPLY_CHARGER_TYPE_AC:
        BAT_DBG_E("-----------------------------------------------USB_DCP\n");
        g_usb_state = AC_IN;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG:
        BAT_DBG_E("-----------------------------------------------OTG\n");
        g_usb_state = ENABLE_5V;
        setSMB345Charger(g_usb_state);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG_ASYNC:
        BAT_DBG_E("-----------------------------------------------OTG\n");
        g_usb_state = ENABLE_5V;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG_OUT:
        BAT_DBG_E("-----------------------------------------------OTG_OUT\n");
        g_usb_state = DISABLE_5V;
        setSMB345Charger(g_usb_state);
        break;
    case POWER_SUPPLY_CHARGER_TYPE_OTG_OUT_ASYNC:
        BAT_DBG_E("-----------------------------------------------OTG_OUT_ASYNC\n");
        g_usb_state = DISABLE_5V;
        schedule_work(&chrgr_data.chrgr_type_work);
        break;
    default:
        BAT_DBG_E("-----------------------------------------------FUCK *\n");
        break;
    }

    return NOTIFY_OK;
}

static irqreturn_t smb345_inok_interrupt(int irq, void *data)
{
    struct smb345_charger *smb = data;

    /* wake lock to prevent system instantly
       enter S3 while it's in resuming flow */
    wake_lock_timeout(&smb->wakelock, HZ);

    if (gpio_get_value(smb->usb_acok)) {
        dev_warn(&smb->client->dev,
            "%s: >>> INOK pin (HIGH) <<<\n", __func__);
    }
    else {
        dev_warn(&smb->client->dev,
            "%s: >>> INOK pin (LOW) <<<\n", __func__);
    }

    return IRQ_HANDLED;
}

static void config_suspend_irq(int irq)
{
    if (irq != -1) {
        disable_irq(irq);
        enable_irq_wake(irq);
    }
}

static void config_resume_irq(int irq)
{
    if (irq != -1) {
        enable_irq(irq);
        disable_irq_wake(irq);
    }
}

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
void smb345_screen_changed_listener(const int state)
{
    if (state == NOTIFY_WHEN_SCREEN_OFF) {
        BAT_DBG(" %s: ScreenOff\n", __func__);

        screen_off = true;

        /* JEITA function by charger protection */
        if (g_usb_state == AC_IN || g_usb_state == USB_IN)
        smb345_charger_control_jeita();
    }
    else if (state == NOTIFY_WHEN_SCREEN_ON) {
        BAT_DBG(" %s: ScreenOn\n", __func__);

        screen_off = false;
    }
}
#endif

bool PAD_CHARGING_FULL(void)
{
    int ret;

    ret = smb345_set_writable(&chrgr_data, true);
    if (ret < 0)
        return false;

    ret = smb345_read(&chrgr_data, IRQSTAT_C);
    if (ret < 0)
        return false;

    if (ret & IRQSTAT_C_TERMINATION_STAT) {
        /* set to almost full if battery is in termination charging current mode */
        BAT_DBG_E(" TERMINATION CHARING CURRENT hit...\n");
        return true;
    }

    return false;
}
EXPORT_SYMBOL_GPL(PAD_CHARGING_FULL);

static int smb345_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct usb_phy *otg_handle;
	struct smb345_charger *chrgr = &chrgr_data;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	int ret;
	int hwid;
	u32 val;

	pr_info("++++++++++++++++++++++++++++ [%s] ++++++++++++++++++++++++++++\n",
		__func__);
	INIT_CHARGER_DEBUG_ARRAY(chrgr_dbg);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_PROBE, 0, 0);

	/* Get pinctrl configurations */
	chrgr->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(chrgr->pinctrl))
		BUG();

	chrgr->pins_default = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_DEFAULT);
	if (IS_ERR(chrgr->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	chrgr->pins_sleep = pinctrl_lookup_state(chrgr->pinctrl,
						PINCTRL_STATE_SLEEP);
	if (IS_ERR(chrgr->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	chrgr->pins_inactive = pinctrl_lookup_state(chrgr->pinctrl,
						"inactive");
	if (IS_ERR(chrgr->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

	/* Register i2c clientdata before calling pinctrl */
	i2c_set_clientdata(client, chrgr);

	pr_info("%s\n", __func__);

	chrgr->client = client;
	wake_lock_init(&chrgr->wakelock, WAKE_LOCK_SUSPEND, "smb347");
	spin_lock_init(&spinlock_suspend);

	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (otg_handle == NULL) {
		pr_err("ERROR!: getting OTG transceiver failed\n");
		return -EINVAL;
	}
	chrgr->otg_handle = otg_handle;

	/*
	 * Get interrupt from device tree
	 */
	chrgr->chgint_irq = irq_of_parse_and_map(np, 0);
	if (!chrgr->chgint_irq) {
		ret = -EINVAL;
		pr_err("can't get irq\n");
		goto remap_fail;
	}
	client->irq = chrgr->chgint_irq;

	chrgr->chgdet_irq = irq_of_parse_and_map(np, 1);
	if(!chrgr->chgdet_irq) {
		ret = -EINVAL;
		pr_err("Unable to retrieve IRQ for charger detection\n");
		goto remap_fail;
	}

	chrgr->chg_otg_en_gpio = of_get_named_gpio(np, "intel,otg-enable", 0);
	hwid = Read_HW_ID();
#if defined(CONFIG_Z380C)
	if (hwid == HW_ID_SR)
		chrgr->chg_otg_en_gpio = of_get_named_gpio(np, "intel,otg-enable-SR", 0);
#endif

	if (!gpio_is_valid(chrgr->chg_otg_en_gpio)) {
		pr_err("Unable to retrieve intel,otg-enable pin\n");
	} else {
		ret = gpio_request(chrgr->chg_otg_en_gpio, "smb346_otg");
		if (ret < 0)
			pr_err("%s: request CHG_OTG gpio fail!\n", __func__);
	}

#if !INTEL_STYLE
	if (ret < 0)
		goto pre_fail;
	/* disable OTG function initially */
	gpio_direction_output(chrgr->chg_otg_en_gpio, 0);
#endif

#if defined(CONFIG_Z300C) || defined(CONFIG_Z300CG)
	chrgr->usb_acok = of_get_named_gpio_flags(np, "intel,usb_acok", 0, 0);
	if (!gpio_is_valid(chrgr->usb_acok)) {
		pr_err("%s(): can't get gpio: intel,usb_acok\n",
			__func__);
		goto fail;
	}

	pr_info("chrgr->usb_acok: %d\n", chrgr->usb_acok);

	ret = gpio_request_one(chrgr->usb_acok, GPIOF_IN, "usb_acok");
	if (ret < 0) {
		pr_err("%s: request usb_acok gpio fail!\n", __func__);
		goto fail;
	}

	chrgr->acok_irq = irq_of_parse_and_map(np, 2);
	if (IS_ERR_VALUE(chrgr->acok_irq)) {
		pr_err("%s: request acok_irq fail!\n", __func__);
		goto fail;
	}

	ret = devm_request_irq(dev, chrgr->acok_irq,
					smb345_inok_interrupt,
					IRQF_SHARED | IRQF_NO_SUSPEND,
					"usbin_det",
					chrgr);
	if (ret < 0) {
		pr_err("%s: config usb_acok gpio as IRQ fail!\n", __func__);
		goto fail;
	}

	chrgr->dcin_ok = of_get_named_gpio_flags(np, "intel,dcin_ok", 0, 0);
	if (!gpio_is_valid(chrgr->dcin_ok)) {
		pr_err("%s(): can't get gpio: intel,dcin_ok\n",
			__func__);
		goto fail;
	}

	pr_info("chrgr->dcin_ok: %d\n", chrgr->dcin_ok);

	ret = gpio_request_one(chrgr->dcin_ok, GPIOF_IN, "dcin_ok");
	if (ret < 0) {
		pr_err("%s: request dcin_ok gpio fail!\n", __func__);
		goto fail;
	}

	chrgr->dcin_irq = irq_of_parse_and_map(np, 3);
	if (IS_ERR_VALUE(chrgr->dcin_irq)) {
		pr_err("%s: request dcin_irq fail!\n", __func__);
		goto fail;
	}

	ret = devm_request_irq(dev, chrgr->dcin_irq,
					smb345_inok_interrupt,
					IRQF_SHARED | IRQF_NO_SUSPEND,
					"dcin_det",
					chrgr);
	if (ret < 0) {
		pr_err("%s: config dcin_ok gpio as IRQ fail!\n", __func__);
		goto fail;
	}
#endif

	ret = of_property_read_u32(np, "intel,chgint-debounce", &val);
	if (ret) {
		pr_err("Unable to retrieve intel,chgint-debounce\n");
		val = 0;
	}
	chrgr->chgint_debounce = msecs_to_jiffies(val);

	/* Setup the PMU registers. The charger IC reset line
	   is deasserted at this point. From this point onwards
	   the charger IC will act upon the values stored in its
	   registers instead of displaying default behaviour  */
	ret = smb345_configure_pmu_regs(chrgr);
	if (ret != 0)
		goto pre_fail;

	ret = smb345_set_writable(chrgr, true);

	if (ret != 0)
		pr_err("failed to enable writable\n");

	smb345_dump_registers(NULL);

	/* Refer to SMB345 Application Note 72 to solve serious problems */
	ret = smb345_masked_write(chrgr->client,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			OTG_CURRENT_LIMIT_250mA);
	if (ret < 0)
		return ret;

	sema_init(&chrgr->prop_lock, 1);

	/* Set up the wake lock to prevent suspend when charging. */
	wake_lock_init(&chrgr->suspend_lock,
			WAKE_LOCK_SUSPEND,
			"smb345_wake_lock");

	/* It is OK to setup the wake lock here, because it is
	before the interrupt is later enabled with the call:
	smb345_configure_pmu_irq(). */
	if (unfreezable_bh_create(&chrgr->chgint_bh, "chrgr_wq",
			"smb345_evt_lock", smb345_chgint_cb_work_func)) {
		ret = -ENOMEM;
		goto wq_creation_fail;
	}

	ret = smb345_set_pinctrl_state(client, chrgr->pins_default);
	if (ret != 0)
		return ret;

#if INTEL_STYLE
	chrgr->usb_psy.name           = "usb_charger";
	chrgr->usb_psy.type           = POWER_SUPPLY_TYPE_USB;
	chrgr->usb_psy.properties     = smb345_power_props;
	chrgr->usb_psy.num_properties = ARRAY_SIZE(smb345_power_props);
	chrgr->usb_psy.get_property   = smb345_charger_get_property;
	chrgr->usb_psy.set_property   = smb345_charger_set_property;
	chrgr->usb_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->usb_psy.supplied_to = smb345_supplied_to;
	chrgr->usb_psy.num_supplicants = ARRAY_SIZE(smb345_supplied_to);
	chrgr->usb_psy.throttle_states = smb345_dummy_throttle_states;
	chrgr->usb_psy.num_throttle_states =
				ARRAY_SIZE(smb345_dummy_throttle_states);

	chrgr->current_psy = &chrgr->usb_psy;

	ret = power_supply_register(&client->dev, &chrgr->usb_psy);
	if (ret)
		goto fail;

	chrgr->ac_psy.name           = "ac_charger";
	chrgr->ac_psy.type           = POWER_SUPPLY_TYPE_MAINS;
	chrgr->ac_psy.properties     = smb345_power_props;
	chrgr->ac_psy.num_properties = ARRAY_SIZE(smb345_power_props);
	chrgr->ac_psy.get_property   = smb345_charger_get_property;
	chrgr->ac_psy.set_property   = smb345_charger_set_property;
	chrgr->ac_psy.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB_SDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_DCP |
					POWER_SUPPLY_CHARGER_TYPE_USB_CDP |
					POWER_SUPPLY_CHARGER_TYPE_USB_FLOATING;
	chrgr->ac_psy.supplied_to = smb345_supplied_to;
	chrgr->ac_psy.num_supplicants = ARRAY_SIZE(smb345_supplied_to);
	chrgr->ac_psy.throttle_states = smb345_dummy_throttle_states;
	chrgr->ac_psy.num_throttle_states =
				ARRAY_SIZE(smb345_dummy_throttle_states);

	ret = power_supply_register(&client->dev, &chrgr->ac_psy);
	if (ret)
		goto fail_ac_registr;
#else
	ret = smb345_register_power_supply(&client->dev);
	if (ret < 0)
		return ret;
#endif


	chrgr->ack_time = jiffies;

	ret = smb345_configure_pmu_irq(chrgr);
	if (ret != 0)
		goto pmu_irq_fail;

	i2c_set_clientdata(client, chrgr);

	smb345_setup_fake_vbus_sysfs_attr(chrgr);
	charger_work_queue = create_singlethread_workqueue("charger_workqueue");

	if (unfreezable_bh_create(&chrgr->boost_op_bh, "boost_op_wq",
				"smb345_boost_lock", smb345_set_boost)) {
		ret = -ENOMEM;
		goto pmu_irq_fail;
	}

#if INTEL_STYLE
	ret = usb_register_notifier(otg_handle, &chrgr->otg_nb);
	if (ret) {
		pr_err("ERROR!: registration for OTG notifications failed\n");
		goto boost_reg_fail;
	}
	INIT_DELAYED_WORK(&charger_work, do_charger);
#endif
	device_init_wakeup(&client->dev, true);

    INIT_WORK(&chrgr->chrgr_type_work, chrgr_type_work_func);
    cable_status_register_client(&charger_type_notifier);

	/* create /sys/kernel/debug/smb */
	dentry = debugfs_create_file("smb", S_IRUGO,
				NULL, &chrgr, &smb345_debugfs_fops);

	/* create /sys/bus/idi/devices/i2c_5-0/i2c-5/5-006a/ interface */
	sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

	ret = init_asus_charging_limit_toggle();
	if (ret) {
		BAT_DBG_E("Unable to create proc file\n");
		return ret;
	}

	/* register callback function */
	//smb345_tbl.soc_control_float_vol = smb345_soc_control_float_vol;
	//smb345_tbl.get_soc_control_float_vol = smb345_get_soc_control_float_vol;
	smb345_tbl.charging_toggle = smb345_charging_toggle;
	smb345_tbl.dump_registers = smb345_dump_registers;
	smb345_tbl.jeita_control = smb3xx_jeita_control;
	smb345_tbl.aicl_control = aicl_cur_control;
	asus_register_power_supply_charger(dev, &smb345_tbl);

	pr_info("---------------------------- [%s] ----------------------------\n",
		__func__);

	/* Read the VBUS presence status for initial update by
	making a dummy interrupt bottom half invocation */
	queue_delayed_work(chrgr->chgint_bh.wq, &chrgr->chgint_bh.work, msecs_to_jiffies(1000));

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
	callback_struct = register_screen_state_notifier(&smb345_screen_changed_listener);
#endif
	return 0;

boost_reg_fail:
	unfreezable_bh_destroy(&chrgr->boost_op_bh);
pmu_irq_fail:
#if INTEL_STYLE
	power_supply_unregister(&chrgr->ac_psy);
#endif
fail_ac_registr:
#if INTEL_STYLE
	power_supply_unregister(&chrgr->usb_psy);
#endif
fail:
	unfreezable_bh_destroy(&chrgr->chgint_bh);
	if (!IS_ERR_OR_NULL(dentry))
		debugfs_remove(dentry);
pre_fail:
wq_creation_fail:
remap_fail:
	usb_put_phy(otg_handle);
	return ret;
}

static int __exit smb345_i2c_remove(struct i2c_client *client)
{
	int ret = 0;
	struct smb345_charger *chrgr = &chrgr_data;
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_I2C_REMOVE, 0, 0);

	free_irq(client->irq, chrgr);
#if INTEL_STYLE
	power_supply_unregister(&chrgr_data.usb_psy);
	power_supply_unregister(&chrgr_data.ac_psy);
#else
	power_supply_unregister(&smb345_power_supplies[CHARGER_USB-1]);
	power_supply_unregister(&smb345_power_supplies[CHARGER_AC-1]);
#endif
	wake_lock_destroy(&chrgr_data.suspend_lock);

	unfreezable_bh_destroy(&chrgr_data.chgint_bh);
	unfreezable_bh_destroy(&chrgr_data.boost_op_bh);

	cancel_delayed_work_sync(&chrgr_data.charging_work);
	if (chrgr_data.otg_handle)
		usb_put_phy(chrgr_data.otg_handle);

	if (!IS_ERR_OR_NULL(dentry))
		debugfs_remove(dentry);

	ret = smb345_set_pinctrl_state(client, chrgr->pins_inactive);
	if (ret != 0)
		return ret;
    return 0;
}

static int smb345_idi_probe(struct idi_peripheral_device *ididev,
					const struct idi_device_id *id)
{
	struct resource *res;
	struct smb345_charger *chrgr = &chrgr_data;
	int ret = 0;

	spin_lock_init(&chrgr_dbg.lock);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_PROBE, 0, 0);
	pr_info("%s\n", __func__);

	res = idi_get_resource_byname(&ididev->resources,
			IORESOURCE_MEM, "registers");

	if (res == NULL) {
		pr_err("getting pmu_chgr_ctrl resources failed!\n");
		return -EINVAL;
	}

	chrgr->ctrl_io_res = res;

	chrgr->ididev = ididev;

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
								__func__);
		return ret;
	}

	return 0;
}

static int __exit smb345_idi_remove(struct idi_peripheral_device *ididev)
{
	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_info("%s\n", __func__);

	return 0;
}

static int smb345_suspend(struct device *dev)
{
	struct smb345_charger *chrgr = &chrgr_data;
	unsigned long flags;

	spin_lock_irqsave(&spinlock_suspend, flags);
	suspend_smb = true;
	spin_unlock_irqrestore(&spinlock_suspend, flags);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_SUSPEND_OK, 0, 0);
	if (device_may_wakeup(dev)) {
		pr_info("%s: enable wakeirq\n", __func__);
		enable_irq_wake(chrgr->chgdet_irq);
	}
	unfreezable_bh_suspend(&chrgr_data.chgint_bh);
	unfreezable_bh_suspend(&chrgr_data.boost_op_bh);
	return 0;
}

static int smb345_resume(struct device *dev)
{
	struct smb345_charger *chrgr = &chrgr_data;
	unsigned long flags;

	spin_lock_irqsave(&spinlock_suspend, flags);
	suspend_smb = false;
	if (suspend_otg_work_not_done) {
		wake_lock_timeout(&chrgr_data.suspend_lock, HZ);
		schedule_work(&chrgr_data.chrgr_type_work);
	}
	spin_unlock_irqrestore(&spinlock_suspend, flags);

	CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_RESUME, 0, 0);

	unfreezable_bh_resume(&chrgr_data.chgint_bh);
	unfreezable_bh_resume(&chrgr_data.boost_op_bh);

	if (device_may_wakeup(dev)) {
		pr_info("%s: disable wakeirq\n", __func__);
		disable_irq_wake(chrgr->chgdet_irq);
	}
	return 0;
}

static int smb345_shutdown(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\n", __func__);

	/* JEITA function by charger protection */
	if (g_usb_state == AC_IN || g_usb_state == USB_IN)
		smb345_charger_control_jeita();

	/* registers dump */
	smb345_dump_registers(NULL);

	return 0;
}

const struct dev_pm_ops smb345_pm = {
	.suspend = smb345_suspend,
	.resume = smb345_resume,
};


static const struct i2c_device_id smb345_id[] = {
	{"smb345_charger", 0}, { }
};

MODULE_DEVICE_TABLE(i2c, smb345_id);

static struct i2c_driver smb345_i2c_driver = {
	.probe          = smb345_i2c_probe,
	.remove         = smb345_i2c_remove,
	.shutdown       = smb345_shutdown,
	.id_table       = smb345_id,
	.driver = {
		.name   = SMB345_NAME,
		.owner  = THIS_MODULE,
		.pm = &smb345_pm,
	},
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CHG,
	},

	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver smb345_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "smb345_idi",
		.pm = NULL,
	},
	.p_type = IDI_CHG,
	.id_table = idi_ids,
	.probe  = smb345_idi_probe,
	.remove = smb345_idi_remove,
};


static int __init smb345_init(void)
{
	int ret;

	pr_info("++++++++++++++++++++++++++++ [%s] ++++++++++++++++++++++++++++\n",
		__func__);
	ret = idi_register_peripheral_driver(&smb345_idi_driver);
	if (ret)
		return ret;


	ret = i2c_add_driver(&smb345_i2c_driver);
	if (ret)
		return ret;

	return 0;
}

module_init(smb345_init);

static void __exit smb345_exit(void)
{
	i2c_del_driver(&smb345_i2c_driver);
}
module_exit(smb345_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Charger Driver for SMB345 charger IC");
