/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Chris Chang chris1_chang@asus.com
 */
#ifndef __ASUS_BATTERY_H__
#define __ASUS_BATTERY_H__

#include <linux/i2c.h>

enum cable_type {
	NO_CABLE = 0,
	USB_ADAPTER,
	USB_PC,
	NUM_CABLE_STATES,
};

enum drv_status {
	DRV_NOT_READY = 0,
	DRV_INIT,
	DRV_INIT_OK,
	DRV_REGISTER,
	DRV_REGISTER_OK,
	DRV_BATTERY_LOW,
	DRV_SHUTDOWN,
};

enum dev_status {
	DEV_NOT_READY = 0,
	DEV_INIT,
	DEV_INIT_OK,
};

struct battery_dev_info {
	enum dev_status status;
	u32 test_flag;
	struct i2c_client *i2c;
	int update_status;
};

struct asus_bat_config {
	int battery_support;
	int polling_time;
	int critical_polling_time;
	int slave_addr;
};

struct asus_batgpio_set {
	int active;
	int bitmap;
	int ctraddr;
	int icaddr;
	int idaddr;
	int ipcaddr;
	int ipdaddr;
	int int_active;
	int int_bitmap;
	int int_sts_addr;
	int int_ctrl_addr;
	int int_type_addr;
	int int_type_val_clr;
	int int_type_val_set;
};

/*
 * Return an unsigned integer value of the predicted remaining battery capacity
 * expressd as a percentage of FullChargeCapacity(), with a range of 0 to 100%.
 * Or < 0 if something fails.
 */
/* int asus_battery_dev_read_percentage(void); */

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
/* int asus_battery_dev_read_volt(void); */

/*
 * Return the battery average current
 * Note that current can be negative signed as well(-65536 ~ 65535).
 * So, the value get from device need add a base(0x10000) to
 * be a positive number if no error.
 * Or < 0 if something fails.
 */
/* int asus_battery_dev_read_current(void); */

/*
 * Return an unsigned integer value of the predicted charge or
 * energy remaining in the battery. The value is reported
 * in units of mWh.
 * Or < 0 if something fails.
 */
/* int asus_battery_dev_read_av_energy(void); */

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
/* int asus_battery_dev_read_temp(void); */

struct dev_func {
	/* state of charge */
	int (*read_percentage)(void);
	/* voltage */
	int (*read_volt)(void);
	/* average current */
	int (*read_current)(void);
	/* average energy */
	int (*read_av_energy)(void);
	/* temperature */
	int (*read_temp)(void);
	/* full charge capacity */
	int (*read_fcc)(void);
	/* nominal available capacity */
	int (*read_nac)(void);
	/* charging cycle count */
	int (*read_cc)(void);
	/* remaining capacity */
	int (*read_rm)(void);
};

struct chgr_dev_func {
	/* charger driver related */
	int (*soc_control_float_vol)(int bat_temp);
	int (*charging_toggle)(enum charging_toggle_level level, bool on);
	int (*dump_registers)(struct seq_file *s);
};

/* power relative function, called some times by other drivers,
   but could only service one device.
*/
int asus_battery_init(
	u32 polling_time,
	u32 critical_polling_time,
	u32 test_flag
);

void asus_battery_exit(void);
int asus_register_power_supply(struct device *dev, struct dev_func *tbl);
int asus_register_power_supply_charger(struct device *dev,
		struct chgr_dev_func *tbl);
int asus_battery_low_event(void);
u32 asus_update_all(void);
void asus_queue_update_all(void);
void asus_cancel_work(void);
void usb_to_battery_callback(u32);

struct battery_info_reply {
	u32 status;			/* Battery status */
	u32 health;			/* Battery health */
	u32 present;			/* Battery present */
	u32 technology;			/* Battery technology */
	u32 batt_volt;			/* Battery voltage */
	int batt_current;		/* Battery current */
	u32 batt_energy;		/* Battery energy */
	u32 percentage;			/* Battery percentage,
					   -1 is unknown status */
	u32 raw_percentage;		/* Battery percentage,
					   raw value acquire from Gauge
					   (not yet shifted) */
	u32 fcc;			/* Battery full charge capacity */
	u32 cc;				/* Battery charging cycle count */
	u32 rm;				/* Battery remaining capacity */
	u32 nac;			/* Battery nominal available capacity */
	u32 level;			/* Battery level */
	int batt_temp;			/* Battery Temperature */
	const char *manufacturer;	/* Battery Manufacturer */
	const char *model;		/* Battery Gauge IC model */
	const char *serial_number;	/* Battery Gauge IC firmware version */
#ifdef CONFIG_FE170CG_BATTERY_BQ27520
	const char *chemical_id;	/* Battery Gauge IC cell data version */
	const char *fw_cfg_version;	/* Battery Gauge IC firmware config
					   version */
#endif
	u32 polling_time;
	u32 critical_polling_time;
	u32 cable_status;
	u32 test_flag;
#ifdef FE170CG_ENG_BUILD
	bool eng_charging_limit;
#endif
	enum drv_status drv_status;	/* Battery driver status */
	struct dev_func *tbl;
	struct chgr_dev_func *tbl_chgr;
#ifndef FE170CG_USER_BUILD
	bool emerg_poll;		/* Battery emergency polling toggle */
#endif
};

#define TEST_INFO_NO_REG_POWER  BIT0
#define TEST_INFO_PROC_DUMP     BIT2

#define BATTERY_DEFAULT_POLL_TIME	(60 * HZ)	/* 60 seconds */
#define BATTERY_CRITICAL_POLL_TIME	(5 * HZ)	/* 5 seconds */
#define BATTERY_EMERGENCY_POLL_TIME	(2 * HZ)	/* 2 seconds */

/* #define BATTERY_PROC_NAME  "driver/virtual_battery" */
#define HIGH_TEMPERATURE_TO_STOP_CHARGING	(550)	/* unit: 0.1C */
#define LOW_TEMPERATURE_TO_STOP_CHARGING	(0)	/* unit: 0.1C */

#define BATTERY_TAG "<BATT>"
#define BAT_DBG(...)  pr_info(BATTERY_TAG __VA_ARGS__)
#define BAT_DBG_L(level, ...)  pr_devel(level BATTERY_TAG __VA_ARGS__)
#define BAT_DBG_E(...)  pr_err(BATTERY_TAG __VA_ARGS__)

#define BATTERY_I2C_ANY_ADDR    0xFF

#define BYTETOBINARYPATTERN "%d%d%d%d-%d%d%d%db"
#define BYTETOBINARY(byte) \
	((byte) & 0x80 ? 1 : 0), \
	((byte) & 0x40 ? 1 : 0), \
	((byte) & 0x20 ? 1 : 0), \
	((byte) & 0x10 ? 1 : 0), \
	((byte) & 0x08 ? 1 : 0), \
	((byte) & 0x04 ? 1 : 0), \
	((byte) & 0x02 ? 1 : 0), \
	((byte) & 0x01 ? 1 : 0)

#define BATTERY_PERCENTAGE_SHIFT

#endif /* __ASUS_BATTERY_H__ */
