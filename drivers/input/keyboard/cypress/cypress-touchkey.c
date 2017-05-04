/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * BLN code originally by neldar. Adapted for SGSII by creams. Ported
 * by gokhanmoral.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/earlysuspend.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "issp_extern.h"
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT540E
#include <linux/i2c/mxt540e.h>
#else
#include <linux/i2c/mxt224_u1.h>
#endif
#include <linux/i2c/touchkey_i2c.h>

/* M0 Touchkey temporary setting */

#if defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_M3)
#define CONFIG_MACH_Q1_BD
#elif defined(CONFIG_MACH_C1) && !defined(CONFIG_TARGET_LOCALE_KOR)
#define CONFIG_MACH_Q1_BD
#elif defined(CONFIG_MACH_C1) && defined(CONFIG_TARGET_LOCALE_KOR)
/* C1 KOR doesn't use Q1_BD */
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT_TEMP)
/* Temp Fix NAGSM_SEL_ANDROID_MOHAMMAD_ANSARI_20111224*/
#define CONFIG_TARGET_LOCALE_NAATT
#endif

static int touchkey_keycode[] = { 0,
#if defined(TK_USE_4KEY_TYPE_ATT)
	KEY_MENU, KEY_ENTER, KEY_BACK, KEY_END,

#elif defined(TK_USE_4KEY_TYPE_NA)
	KEY_SEARCH, KEY_BACK, KEY_HOME, KEY_MENU,

#elif defined(TK_USE_2KEY_TYPE_M0)
	KEY_BACK, KEY_MENU,

#else
	KEY_MENU, KEY_BACK,

#endif
};
static const int touchkey_count = sizeof(touchkey_keycode) / sizeof(int);

#if defined(TK_HAS_AUTOCAL)
static u16 raw_data0;
static u16 raw_data1;
static u16 raw_data2;
static u16 raw_data3;
static u8 idac0;
static u8 idac1;
static u8 idac2;
static u8 idac3;
static u8 touchkey_threshold;

static int touchkey_autocalibration(struct touchkey_i2c *tkey_i2c);
#endif

#if defined(CONFIG_TARGET_LOCALE_KOR)
#ifndef TRUE
#define TRUE	1
#endif

#ifndef FALSE
#define FALSE	0
#endif

#if defined(SEC_TKEY_EVENT_DEBUG)
static bool g_debug_tkey = TRUE;
#else
static bool g_debug_tkey = FALSE;
#endif
#endif

static int touchkey_i2c_check(struct touchkey_i2c *tkey_i2c);

static u16 menu_sensitivity;
static u16 back_sensitivity;
#if defined(TK_USE_4KEY)
static u8 home_sensitivity;
static u8 search_sensitivity;
#endif

static int touchkey_enable;
static bool touchkey_probe = true;
static DEFINE_MUTEX(touchkey_enable_mutex);

static const struct i2c_device_id sec_touchkey_id[] = {
	{"sec_touchkey", 0},
	{}
};

#ifdef CONFIG_TOUCHKEY_BLN
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#define BLN_VERSION 9

bool bln_enabled = false;
bool bln_ongoing = false;
bool bln_blink_enabled = false;
bool bln_suspended = false;

static struct touchkey_i2c *bln_tkey_i2c = NULL;
static void enable_led_notification(void);
static void disable_led_notification(void);
static struct wake_lock bln_wake_lock;

/*
 * Standard CM7 LED Notification functionality.
 */

/* Timer related declares */
static struct timer_list led_timer;

static void bl_on();
static void bl_off(struct work_struct *bl_off_work);
static DECLARE_WORK(bl_off_work, bl_off);

static struct timer_list notification_timer;
static void notification_off(struct work_struct *notification_off_work);
static DECLARE_WORK(notification_off_work, notification_off);

static void led_fadeout_process(struct work_struct *led_fadeout_work);
static DECLARE_WORK(led_fadeout_work, led_fadeout_process);

static void led_fadein_process(struct work_struct *led_fadein_work);
static DECLARE_WORK(led_fadein_work, led_fadein_process);

#define BL_STANDARD	3000
#define BL_MIN		2500
#define BL_MAX		3300

int notification_timeout = -1;
int led_disabled = 0; // 1= force disable the touchkey backlight
int led_timeout;
bool led_in_fadein = false;
bool led_in_fadeout = false;
static DEFINE_SEMAPHORE(enable_sem);

static struct timer_list breathing_timer;
static void breathe(struct work_struct *breathe_work);
void stop_breathing(void);
static DECLARE_WORK(breathe_work, breathe);

/* Breathing variables */
#define MAX_BREATHING_STEPS 10
static unsigned int breathing = 0;
static int breathing_step_count = 0;
struct breathing_step {
	int start; //mV
	int end; //mV
	int period; //ms
	int step; //mV
};
struct breathing_step breathing_steps[MAX_BREATHING_STEPS];
static int breathing_idx = 0;
static int breathing_step_idx = 0;

static unsigned int touchkey_voltage = BL_STANDARD;
static unsigned int touchkey_voltage_saved = BL_STANDARD;
static unsigned int touchkey_voltage_brightness = BL_STANDARD;
static bool dyn_brightness = false;
static bool blnww = false;

#define FADEIN_STEP_MS 50
#define FADEOUT_STEP_MS 50
static int led_fadein = 0, led_fadeout = 0;
static int led_on_touch = 0;

#endif

MODULE_DEVICE_TABLE(i2c, sec_touchkey_id);

extern int get_touchkey_firmware(char *version);
static int touchkey_led_status;
static int touchled_cmd_reversed;

static int touchkey_debug_count;
static char touchkey_debug[104];

#ifdef LED_LDO_WITH_REGULATOR
static void change_touch_key_led_voltage(int vol_mv)
{
	printk(KERN_DEBUG "[TouchKey] %s: %dmV\n",
		       __func__, vol_mv);

	if (vol_mv > touchkey_voltage_brightness) {
		printk(KERN_DEBUG "[TouchKey-LED] %s: Ignoring LED voltage %dmV, max is set at %dmV\n", __func__, vol_mv, touchkey_voltage_brightness);
	} else {
		struct regulator *tled_regulator;

		tled_regulator = regulator_get(NULL, "touch_led");
		if (IS_ERR(tled_regulator)) {
			pr_err("%s: failed to get resource %s\n", __func__,
			       "touch_led");
			return;
		}
		regulator_set_voltage(tled_regulator, vol_mv * 1000, vol_mv * 1000);
		regulator_put(tled_regulator);
	}
}

static ssize_t brightness_control(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int data;

	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_DEBUG "[TouchKey] touch_led_brightness: %d\n", data);
		change_touch_key_led_voltage(data);
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}

	return size;
}

#ifdef CONFIG_TOUCHKEY_BLN
struct regulator {
        struct device *dev;
        struct list_head list;
        int uA_load;
        int min_uV;
        int max_uV;
        char *supply_name;
        struct device_attribute dev_attr;
        struct regulator_dev *rdev;
};

void set_touch_constraints(bool blnstatus)
{
	struct regulator *r;

	if (!blnww && blnstatus) return;

	r = regulator_get(NULL, "touch_led");
	r->rdev->constraints->state_mem.enabled = blnstatus;
	r->rdev->constraints->state_mem.disabled = !blnstatus;
	r = regulator_get(NULL, "touch");
	r->rdev->constraints->state_mem.enabled = blnstatus;
	r->rdev->constraints->state_mem.disabled = !blnstatus;
}

void update_touchkey_brightness(unsigned int level)
{
	if (dyn_brightness) {
		if (level > 0) {
			printk(KERN_DEBUG "[TouchKey-LED] %s: %d\n", __func__, level);
			touchkey_voltage_brightness = BL_MIN + ((((level * 100 / 255) * (BL_MAX - BL_MIN)) / 100) / 50) * 50;
		} else {
			printk(KERN_DEBUG "[TouchKey-LED] %s: Ignoring brightness : %d\n", __func__, level);
		}
	}
}

#endif
#endif

static void set_touchkey_debug(char value)
{
	if (touchkey_debug_count == 100)
		touchkey_debug_count = 0;

	touchkey_debug[touchkey_debug_count] = value;
	touchkey_debug_count++;
}

static int i2c_touchkey_read(struct i2c_client *client,
		u8 reg, u8 *val, unsigned int len)
{
	int err = 0;
	int retry = 3;
#if !defined(TK_USE_GENERAL_SMBUS)
	struct i2c_msg msg[1];
#endif

	if ((client == NULL) || !(touchkey_enable == 1)
	    || !touchkey_probe) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled. %d\n",
		       __LINE__);
		return -ENODEV;
	}

	while (retry--) {
#if defined(TK_USE_GENERAL_SMBUS)
		err = i2c_smbus_read_i2c_block_data(client,
				KEYCODE_REG, len, val);
#else
		msg->addr = client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(client->adapter, msg, 1);
#endif

		if (err >= 0)
			return 0;
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		mdelay(10);
	}
	return err;

}

static int i2c_touchkey_write(struct i2c_client *client,
		u8 *val, unsigned int len)
{
	int err = 0;
	int retry = 3;
	mutex_lock(&touchkey_enable_mutex);
#if !defined(TK_USE_GENERAL_SMBUS)
	struct i2c_msg msg[1];
#endif

	if ((client == NULL) || !(touchkey_enable == 1)
	    || !touchkey_probe) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled. %d\n",
		       __LINE__);
		mutex_unlock(&touchkey_enable_mutex);
		return -ENODEV;
	}

	while (retry--) {
#if defined(TK_USE_GENERAL_SMBUS)
		err = i2c_smbus_write_i2c_block_data(client,
				KEYCODE_REG, len, val);
#else
		msg->addr = client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(client->adapter, msg, 1);
#endif

		if (err >= 0) {
			mutex_unlock(&touchkey_enable_mutex);
			return 0;
		}
		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		mdelay(10);
	}
	mutex_unlock(&touchkey_enable_mutex);
	return err;
}

#if defined(TK_HAS_AUTOCAL)
static int touchkey_autocalibration(struct touchkey_i2c *tkey_i2c)
{
	u8 data[6] = { 0, };
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;

#if defined(CONFIG_TARGET_LOCALE_NA)
	if (tkey_i2c->module_ver < 8)
		return -1;
#endif

	while (retry < 3) {
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 4);
		if (ret < 0) {
			printk(KERN_ERR "[TouchKey]i2c read fail.\n");
			return ret;
		}
		printk(KERN_DEBUG
				"[TouchKey] data[0]=%x data[1]=%x data[2]=%x data[3]=%x\n",
				data[0], data[1], data[2], data[3]);

		/* Send autocal Command */
		data[0] = 0x50;
		data[3] = 0x01;

		count = i2c_touchkey_write(tkey_i2c->client, data, 4);

		msleep(100);

		/* Check autocal status */
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 6);

		if ((data[5] & TK_BIT_AUTOCAL)) {
			printk(KERN_DEBUG "[Touchkey] autocal Enabled\n");
			break;
		} else
			printk(KERN_DEBUG
			       "[Touchkey] autocal disabled, retry %d\n",
			       retry);

		retry = retry + 1;
	}

	if (retry == 3)
		printk(KERN_DEBUG "[Touchkey] autocal failed\n");

	return count;
}
#endif

#ifdef CONFIG_TARGET_LOCALE_NAATT
static ssize_t set_touchkey_autocal_testmode(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;
	u8 set_data;
	int on_off;

	if (sscanf(buf, "%d\n", &on_off) == 1) {
		printk(KERN_ERR "[TouchKey] Test Mode : %d\n", on_off);

		if (on_off == 1) {
			set_data = 0x40;
			count = i2c_touchkey_write(tkey_i2c->client,
					&set_data, 1);
		} else {
			tkey_i2c->pdata->power_on(0);
			msleep(50);
			tkey_i2c->pdata->power_on(1);
			msleep(50);
#if defined(TK_HAS_AUTOCAL)
			touchkey_autocalibration(tkey_i2c);
#endif
		}
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}

	return count;
}
#endif

#if defined(TK_HAS_AUTOCAL)
static ssize_t touchkey_raw_data0_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[18] =%d,data[19] = %d\n", __func__,
	       data[18], data[19]);
	raw_data0 = ((0x00FF & data[18]) << 8) | data[19];
#elif defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1)\
|| defined(CONFIG_MACH_M3)\
	 || defined(CONFIG_MACH_T0)
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__,
	       data[16], data[17]);
	raw_data0 = ((0x00FF & data[16]) << 8) | data[17]; /* menu*/
#elif defined(CONFIG_MACH_Q1_BD)
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__,
	       data[16], data[17]);
	raw_data0 = ((0x00FF & data[14]) << 8) | data[15];
#else
	printk(KERN_DEBUG "called %s data[18] =%d,data[19] = %d\n", __func__,
	       data[10], data[11]);
	raw_data0 = ((0x00FF & data[10]) << 8) | data[11];
#endif
	return sprintf(buf, "%d\n", raw_data0);
}

static ssize_t touchkey_raw_data1_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[20] =%d,data[21] = %d\n", __func__,
	       data[20], data[21]);
	raw_data1 = ((0x00FF & data[20]) << 8) | data[21];
#elif defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1)\
|| defined(CONFIG_MACH_M3)\
	 || defined(CONFIG_MACH_T0)
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__,
	       data[14], data[15]);
	raw_data1 = ((0x00FF & data[14]) << 8) | data[15]; /*back*/
#elif defined(CONFIG_MACH_Q1_BD)
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__,
			   data[14], data[15]);
	raw_data1 = ((0x00FF & data[16]) << 8) | data[17];
#else
	printk(KERN_DEBUG "called %s data[20] =%d,data[21] = %d\n", __func__,
	       data[12], data[13]);
	raw_data1 = ((0x00FF & data[12]) << 8) | data[13];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data1);
}

static ssize_t touchkey_raw_data2_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[22] =%d,data[23] = %d\n", __func__,
	       data[22], data[23]);
	raw_data2 = ((0x00FF & data[22]) << 8) | data[23];
#else
	printk(KERN_DEBUG "called %s data[22] =%d,data[23] = %d\n", __func__,
	       data[14], data[15]);
	raw_data2 = ((0x00FF & data[14]) << 8) | data[15];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data2);
}

static ssize_t touchkey_raw_data3_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[26] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[24] =%d,data[25] = %d\n", __func__,
	       data[24], data[25]);
	raw_data3 = ((0x00FF & data[24]) << 8) | data[25];
#else
	printk(KERN_DEBUG "called %s data[24] =%d,data[25] = %d\n", __func__,
	       data[16], data[17]);
	raw_data3 = ((0x00FF & data[16]) << 8) | data[17];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data3);
}

static ssize_t touchkey_idac0_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[6] =%d\n", __func__, data[6]);
	idac0 = data[6];
	return sprintf(buf, "%d\n", idac0);
}

static ssize_t touchkey_idac1_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[7] = %d\n", __func__, data[7]);
	idac1 = data[7];
	return sprintf(buf, "%d\n", idac1);
}

static ssize_t touchkey_idac2_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[8] =%d\n", __func__, data[8]);
	idac2 = data[8];
	return sprintf(buf, "%d\n", idac2);
}

static ssize_t touchkey_idac3_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8)
		return 0;
#endif

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[9] = %d\n", __func__, data[9]);
	idac3 = data[9];
	return sprintf(buf, "%d\n", idac3);
}

static ssize_t touchkey_threshold_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[4] = %d\n", __func__, data[4]);
	touchkey_threshold = data[4];
	return sprintf(buf, "%d\n", touchkey_threshold);
}
#endif

#if defined(TK_HAS_FIRMWARE_UPDATE)
static int touchkey_firmware_update(struct touchkey_i2c *tkey_i2c)
{
	int retry = 3;
	int ret = 0;
	char data[3];

	disable_irq(tkey_i2c->irq);


	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk(KERN_DEBUG
		"[TouchKey] i2c read fail. do not excute firm update.\n");
		data[1] = 0;
		data[2] = 0;
	}

	printk(KERN_ERR "%s F/W version: 0x%x, Module version:0x%x\n", __func__,
	data[1], data[2]);

	tkey_i2c->firmware_ver = data[1];
	tkey_i2c->module_ver = data[2];

#if defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1) \
|| defined(CONFIG_MACH_M3) || defined(CONFIG_MACH_T0)
	if ((tkey_i2c->firmware_ver < TK_FIRMWARE_VER) &&
	    (tkey_i2c->module_ver <= TK_MODULE_VER)) {
#else
	if ((tkey_i2c->firmware_ver < TK_FIRMWARE_VER) &&
		(tkey_i2c->module_ver == TK_MODULE_VER)) {
#endif
		printk(KERN_DEBUG "[TouchKey] firmware auto update excute\n");

		tkey_i2c->update_status = TK_UPDATE_DOWN;

		while (retry--) {
			if (ISSP_main(tkey_i2c) == 0) {
				printk(KERN_DEBUG
				       "[TouchKey]firmware update succeeded\n");
				tkey_i2c->update_status = TK_UPDATE_PASS;
				msleep(50);
				break;
			}
			msleep(50);
			printk(KERN_DEBUG
			       "[TouchKey] firmware update failed. retry\n");
		}
		if (retry <= 0) {
			tkey_i2c->pdata->power_on(0);
			tkey_i2c->update_status = TK_UPDATE_FAIL;
			printk(KERN_DEBUG
			       "[TouchKey] firmware update failed.\n");
		}
		ret = touchkey_i2c_check(tkey_i2c);
		if (ret < 0) {
			printk(KERN_DEBUG
				"[TouchKey] i2c read fail.\n");
			return TK_UPDATE_FAIL;
		}
#if defined(CONFIG_TARGET_LOCALE_KOR)
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
		if (ret < 0) {
			printk(KERN_DEBUG
			"[TouchKey] i2c read fail. do not excute firm update.\n");
		}
		tkey_i2c->firmware_ver = data[1];
		tkey_i2c->module_ver = data[2];
#endif
		printk(KERN_DEBUG "[TouchKey] firm ver = %d, module ver = %d\n",
			tkey_i2c->firmware_ver, tkey_i2c->module_ver);
	} else {
		printk(KERN_DEBUG
		       "[TouchKey] firmware auto update do not excute\n");
		printk(KERN_DEBUG
		       "[TouchKey] firmware_ver(banary=%d, current=%d)\n",
		       TK_FIRMWARE_VER, tkey_i2c->firmware_ver);
		printk(KERN_DEBUG
		       "[TouchKey] module_ver(banary=%d, current=%d)\n",
		       TK_MODULE_VER, tkey_i2c->module_ver);
	}
	enable_irq(tkey_i2c->irq);
	return TK_UPDATE_PASS;
}
#else
static int touchkey_firmware_update(struct touchkey_i2c *tkey_i2c)
{
	char data[3];
	int retry;
	int ret = 0;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk(KERN_DEBUG
		       "[TouchKey] i2c read fail. do not excute firm update.\n");
		return ret;
	}

	printk(KERN_ERR "%s F/W version: 0x%x, Module version:0x%x\n", __func__,
	       data[1], data[2]);
	retry = 3;

	tkey_i2c->firmware_ver = data[1];
	tkey_i2c->module_ver = data[2];

	if (tkey_i2c->firmware_ver < 0x0A) {
		tkey_i2c->update_status = TK_UPDATE_DOWN;
		while (retry--) {
			if (ISSP_main(tkey_i2c) == 0) {
				printk(KERN_ERR
				       "[TOUCHKEY]Touchkey_update succeeded\n");
				tkey_i2c->update_status = TK_UPDATE_PASS;
				break;
			}
			printk(KERN_ERR "touchkey_update failed...retry...\n");
		}
		if (retry <= 0) {
			tkey_i2c->pdata->power_on(0);
			tkey_i2c->update_status = TK_UPDATE_FAIL;
			ret = TK_UPDATE_FAIL;
		}
	} else {
		if (tkey_i2c->firmware_ver >= 0x0A) {
			printk(KERN_ERR
			       "[TouchKey] Not F/W update. Cypess touch-key F/W version is latest\n");
		} else {
			printk(KERN_ERR
			       "[TouchKey] Not F/W update. Cypess touch-key version(module or F/W) is not valid\n");
		}
	}
	return ret;
}
#endif

#ifdef CONFIG_TOUCHKEY_BLN
void cypress_notify_touch(void)
{
	if (!bln_suspended && led_on_touch) {
		bl_on();
	}
}

#endif

#ifndef TEST_JIG_MODE
static irqreturn_t touchkey_interrupt(int irq, void *dev_id)
{
#ifdef CONFIG_TOUCHKEY_BLN
        printk(KERN_ERR "[TouchKey] interrupt touchkey\n");
#endif
	struct touchkey_i2c *tkey_i2c = dev_id;
	u8 data[3];
	int ret;
	int retry = 10;
	int keycode_type = 0;
	int pressed;

	set_touchkey_debug('a');

	if (!atomic_read(&tkey_i2c->keypad_enable)) {
		return;
	}

	retry = 3;
	while (retry--) {
		ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
		if (!ret)
			break;
		else {
			printk(KERN_DEBUG
			       "[TouchKey] i2c read failed, ret:%d, retry: %d\n",
			       ret, retry);
			continue;
		}
	}
	if (ret < 0)
		return IRQ_HANDLED;

	set_touchkey_debug(data[0]);

	keycode_type = (data[0] & TK_BIT_KEYCODE);
	pressed = !(data[0] & TK_BIT_PRESS_EV);

	if (keycode_type <= 0 || keycode_type >= touchkey_count) {
		printk(KERN_DEBUG "[Touchkey] keycode_type err\n");
		return IRQ_HANDLED;
	}

	if (pressed)
		set_touchkey_debug('P');

	if (get_tsp_status() && pressed)
		printk(KERN_DEBUG "[TouchKey] touchkey pressed but don't send event because touch is pressed.\n");
	else {
		input_report_key(tkey_i2c->input_dev,
				 touchkey_keycode[keycode_type], pressed);
		input_sync(tkey_i2c->input_dev);
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		printk(KERN_DEBUG "[TouchKey] keycode:%d pressed:%d\n",
		   touchkey_keycode[keycode_type], pressed);
#else
		printk(KERN_DEBUG "[TouchKey] pressed:%d\n",
			pressed);
#endif

		#if defined(CONFIG_TARGET_LOCALE_KOR)
		if (g_debug_tkey == true) {
			printk(KERN_DEBUG "[TouchKey] keycode[%d]=%d pressed:%d\n",
			keycode_type, touchkey_keycode[keycode_type], pressed);
		} else {
			printk(KERN_DEBUG "[TouchKey] pressed:%d\n", pressed);
		}
		#endif
	}
	set_touchkey_debug('A');
#ifdef CONFIG_TOUCHKEY_BLN
	if (pressed)
		cypress_notify_touch();
#endif
	return IRQ_HANDLED;
}
#else
static irqreturn_t touchkey_interrupt(int irq, void *dev_id)
{
#ifdef CONFIG_TOUCHKEY_BLN
        printk(KERN_ERR "[TouchKey-BLN] interrupt touchkey\n");
#endif
	struct touchkey_i2c *tkey_i2c = dev_id;
	u8 data[18];
	int ret;
	int retry = 10;
	int keycode_type = 0;
	int pressed;

#if 0
	if (gpio_get_value(_3_GPIO_TOUCH_INT)) {
		printk(KERN_DEBUG "[TouchKey] Unknown state.\n", __func__);
		return IRQ_HANDLED;
	}
#endif

	set_touchkey_debug('a');

	retry = 3;
	while (retry--) {
#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_BD)\
	 || defined(CONFIG_MACH_C1)
		ret = i2c_touchkey_read(tkey_i2c->client,
				KEYCODE_REG, data, 18);
#else
		ret = i2c_touchkey_read(tkey_i2c->client,
				KEYCODE_REG, data, 10);
#endif
		if (!ret)
			break;
		else {
			printk(KERN_DEBUG
			       "[TouchKey] i2c read failed, ret:%d, retry: %d\n",
			       ret, retry);
			continue;
		}
	}
	if (ret < 0)
		return IRQ_HANDLED;

#if defined(CONFIG_TARGET_LOCALE_NA)
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	menu_sensitivity = data[11];
	home_sensitivity = data[13];
	search_sensitivity = data[15];
	back_sensitivity = data[17];
#else
	if (tkey_i2c->module_ver >= 8) {
		menu_sensitivity = data[17];
		home_sensitivity = data[15];
		search_sensitivity = data[11];
		back_sensitivity = data[13];
	} else {
		menu_sensitivity = data[6];
		home_sensitivity = data[7];
		search_sensitivity = data[8];
		back_sensitivity = data[9];
	}
#endif
#elif defined(CONFIG_MACH_Q1_BD) || defined(CONFIG_MACH_C1)
	menu_sensitivity = data[13];
	back_sensitivity = data[11];
#else
	menu_sensitivity = data[7];
	back_sensitivity = data[9];
#endif				/* CONFIG_TARGET_LOCALE_NA  */

	set_touchkey_debug(data[0]);

	keycode_type = (data[0] & TK_BIT_KEYCODE);
	pressed = !(data[0] & TK_BIT_PRESS_EV);

	if (keycode_type <= 0 || keycode_type >= touchkey_count) {
		printk(KERN_DEBUG "[Touchkey] keycode_type err\n");
		return IRQ_HANDLED;
	}

	if (pressed)
		set_touchkey_debug('P');

	if (get_tsp_status() && pressed)
		printk(KERN_DEBUG "[TouchKey] touchkey pressed"
		       " but don't send event because touch is pressed.\n");
	else {
		input_report_key(touchkey_driver->input_dev,
				 touchkey_keycode[keycode_type], pressed);
		input_sync(touchkey_driver->input_dev);
		/* printk(KERN_DEBUG "[TouchKey] keycode:%d pressed:%d\n",
		   touchkey_keycode[keycode_index], pressed); */
	}

	if (keycode_type == 1)
		printk(KERN_DEBUG "search key sensitivity = %d\n",
		       search_sensitivity);
	if (keycode_type == 2)
		printk(KERN_DEBUG "back key sensitivity = %d\n",
		       back_sensitivity);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (keycode_type == 3)
		printk(KERN_DEBUG "home key sensitivity = %d\n",
		       home_sensitivity);
	if (keycode_type == 4)
		printk(KERN_DEBUG "menu key sensitivity = %d\n",
		       menu_sensitivity);
#endif

	set_touchkey_debug('A');
	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static int sec_touchkey_early_suspend(struct early_suspend *h)
{
	struct touchkey_i2c *tkey_i2c =
		container_of(h, struct touchkey_i2c, early_suspend);
	int ret;
	int i;

	disable_irq(tkey_i2c->irq);
	ret = cancel_work_sync(&tkey_i2c->update_work);
	if (ret) {
		printk(KERN_DEBUG "[Touchkey] enable_irq ret=%d\n", ret);
		enable_irq(tkey_i2c->irq);
	}

	/* release keys */
	for (i = 1; i < touchkey_count; ++i) {
		input_report_key(tkey_i2c->input_dev,
				 touchkey_keycode[i], 0);
	}
	input_sync(tkey_i2c->input_dev);

	mutex_lock(&touchkey_enable_mutex);
	touchkey_enable = 0;

	set_touchkey_debug('S');
	printk(KERN_DEBUG "[TouchKey] sec_touchkey_early_suspend\n");
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		mutex_unlock(&touchkey_enable_mutex);
		return 0;
	}


#ifdef CONFIG_TOUCHKEY_BLN
	if (!bln_ongoing) {
		led_fadeout_process(NULL);
	}
	bln_suspended = 1;
#endif

	/* disable ldo18 */
	tkey_i2c->pdata->led_power_on(0);

	/* disable ldo11 */
	tkey_i2c->pdata->power_on(0);


	mutex_unlock(&touchkey_enable_mutex);
	return 0;
}

static int sec_touchkey_late_resume(struct early_suspend *h)
{
	struct touchkey_i2c *tkey_i2c =
		container_of(h, struct touchkey_i2c, early_suspend);
#ifdef TEST_JIG_MODE
	unsigned char get_touch = 0x40;
#endif

	set_touchkey_debug('R');
	printk(KERN_DEBUG "[TouchKey] sec_touchkey_late_resume\n");

	/* Avoid race condition with LED notification disable */
#ifdef CONFIG_TOUCHKEY_BLN
	down(&enable_sem);
#endif
	mutex_lock(&touchkey_enable_mutex);

	/* enable ldo11 */
	tkey_i2c->pdata->power_on(1);

	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		mutex_unlock(&touchkey_enable_mutex);
		return 0;
	}
#ifdef CONFIG_TOUCHKEY_BLN
	bln_suspended = 0;
	/* see if late_resume is running before DISABLE_BL */
	if (bln_ongoing) {
		/* if a notification timeout was set, disable the timer */
		if (notification_timeout > 0) {
			del_timer(&notification_timer);
		}
		if (breathing) stop_breathing();

		/* we were using a wakelock, unlock it */
		if (wake_lock_active(&bln_wake_lock)) {
			printk(KERN_DEBUG "[TouchKey] touchkey clear wake_lock\n");
			wake_unlock(&bln_wake_lock);
		}
		/* force DISABLE_BL to ignore the led state because we want it left on */
		bln_ongoing = 0;
	}

	cancel_work_sync(&led_fadein_work);
	cancel_work_sync(&led_fadeout_work);
	del_timer(&notification_timer);
	del_timer(&led_timer);
#else
	msleep(50);
	tkey_i2c->pdata->led_power_on(1);
#endif
	touchkey_enable = 1;
	mutex_unlock(&touchkey_enable_mutex);

#if defined(TK_HAS_AUTOCAL)
	touchkey_autocalibration(tkey_i2c);
#endif

	if (touchled_cmd_reversed) {
		touchled_cmd_reversed = 0;
		i2c_touchkey_write(tkey_i2c->client,
			(u8 *) &touchkey_led_status, 1);
		printk(KERN_DEBUG "[Touchkey-BLN] LED returned on\n");
	}
#ifdef TEST_JIG_MODE
	i2c_touchkey_write(tkey_i2c->client, &get_touch, 1);
#endif

	enable_irq(tkey_i2c->irq);

#ifdef CONFIG_TOUCHKEY_BLN
	bl_on();

	/* Avoid race condition with LED notification disable */
	up(&enable_sem);
#endif
	return 0;
}
#endif

static int touchkey_i2c_check(struct touchkey_i2c *tkey_i2c)
{
	char data[3] = { 0, };
	int ret = 0;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk(KERN_ERR "[TouchKey] module version read fail\n");
		return ret;
	}

	tkey_i2c->firmware_ver = data[1];
	tkey_i2c->module_ver = data[2];

	return ret;
}

ssize_t touchkey_update_read(struct file *filp, char *buf, size_t count,
			     loff_t *f_pos)
{
	char data[3] = { 0, };

	get_touchkey_firmware(data);
	put_user(data[1], buf);

	return 1;
}

static ssize_t touch_version_read(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	char data[3] = { 0, };
	int count;

	i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);

	count = sprintf(buf, "0x%x\n", data[1]);

	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%x\n", data[1]);
	printk(KERN_DEBUG "[TouchKey] module_version_read 0x%x\n", data[2]);

	return count;
}

static ssize_t touch_version_write(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] input data --> %s\n", buf);

	return size;
}

void touchkey_update_func(struct work_struct *work)
{
	struct touchkey_i2c *tkey_i2c =
		container_of(work, struct touchkey_i2c, update_work);
	int retry = 3;
#if defined(CONFIG_TARGET_LOCALE_NAATT)
	char data[3];
	i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	printk(KERN_DEBUG "[%s] F/W version: 0x%x, Module version:0x%x\n",
	       __func__, data[1], data[2]);
#endif
	tkey_i2c->update_status = TK_UPDATE_DOWN;
	printk(KERN_DEBUG "[TouchKey] %s start\n", __func__);
	touchkey_enable = 0;
	while (retry--) {
		if (ISSP_main(tkey_i2c) == 0) {
			printk(KERN_DEBUG
			       "[TouchKey] touchkey_update succeeded\n");
			msleep(50);
			touchkey_enable = 1;
#if defined(TK_HAS_AUTOCAL)
			touchkey_autocalibration(tkey_i2c);
#endif
			tkey_i2c->update_status = TK_UPDATE_PASS;
			enable_irq(tkey_i2c->irq);
			return;
		}
		tkey_i2c->pdata->power_on(0);
	}
	enable_irq(tkey_i2c->irq);
	tkey_i2c->update_status = TK_UPDATE_FAIL;
	printk(KERN_DEBUG "[TouchKey] touchkey_update failed\n");
	return;
}

static ssize_t touch_update_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG
		       "[TouchKey] Skipping f/w update : module_version =%d\n",
		       tkey_i2c->module_ver);
		tkey_i2c->update_status = TK_UPDATE_PASS;
		return 1;
	} else {
#endif				/* CONFIG_TARGET_LOCALE_NA */
		printk(KERN_DEBUG "[TouchKey] touchkey firmware update\n");

		if (*buf == 'S') {
			disable_irq(tkey_i2c->irq);
			schedule_work(&tkey_i2c->update_work);
		}
		return size;
#ifdef CONFIG_TARGET_LOCALE_NA
	}
#endif				/* CONFIG_TARGET_LOCALE_NA */
}

static ssize_t touch_update_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;

	printk(KERN_DEBUG
	       "[TouchKey] touch_update_read: update_status %d\n",
	       tkey_i2c->update_status);

	if (tkey_i2c->update_status == TK_UPDATE_PASS)
		count = sprintf(buf, "PASS\n");
	else if (tkey_i2c->update_status == TK_UPDATE_DOWN)
		count = sprintf(buf, "Downloading\n");
	else if (tkey_i2c->update_status == TK_UPDATE_FAIL)
		count = sprintf(buf, "Fail\n");

	return count;
}

static ssize_t touchkey_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	printk(KERN_DEBUG "[TouchKey] %s: %s", __func__, buf);

	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int data;
	int ret;
	static const int ledCmd[] = {TK_CMD_LED_OFF, TK_CMD_LED_ON};

#if defined(CONFIG_TARGET_LOCALE_KOR)
	if (touchkey_probe == false)
		return size;
#endif
	ret = sscanf(buf, "%d", &data);
	if (ret != 1) {
		printk(KERN_DEBUG "[TouchKey] %s, %d err\n",
			__func__, __LINE__);
		return size;
	}

#ifdef CONFIG_TOUCHKEY_BLN
	if (bln_enabled && bln_ongoing) {
		printk(KERN_DEBUG "[TouchKey-BLN] %s: Ignoring because BLN is enabled\n",
			__func__);
		return size;
	}

	update_touchkey_brightness(data);
	if (data == 0) {
		// Only turn off if no led_timeout has been set.
		if (led_timeout <= 0) 
			bl_off(NULL);
	} else {
		bl_on();
	}
	data = data ? 1 : 0;
#else
	if (data != 0 && data != 1) {
		printk(KERN_DEBUG "[TouchKey] %s wrong cmd %x\n",
			__func__, data);
		return size;
	}
#endif
#if defined(CONFIG_TARGET_LOCALE_NA)
	if (tkey_i2c->module_ver >= 8)
		data = ledCmd[data];
#else
	data = ledCmd[data];
#endif

#ifndef CONFIG_TOUCHKEY_BLN
	ret = i2c_touchkey_write(tkey_i2c->client, (u8 *) &data, 1);
	if (ret == -ENODEV)
		touchled_cmd_reversed = 1;
#endif
	touchkey_led_status = data;
	return size;
}

#if defined(TK_USE_4KEY)
static ssize_t touchkey_menu_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[12] =%d,data[13] = %d\n",
		       __func__, data[12], data[13]);
		menu_sensitivity = ((0x00FF & data[12]) << 8) | data[13];
	} else {
		printk(KERN_DEBUG "called %s data[17] =%d\n", __func__,
		       data[17]);
		menu_sensitivity = data[17];
	}
#else
	printk(KERN_DEBUG "called %s data[10] =%d,data[11] = %d\n", __func__,
	       data[10], data[11]);
	menu_sensitivity = ((0x00FF & data[10]) << 8) | data[11];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", menu_sensitivity);
}

static ssize_t touchkey_home_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[10] =%d,data[11] = %d\n",
		       __func__, data[10], data[11]);
		home_sensitivity = ((0x00FF & data[10]) << 8) | data[11];
	} else {
		printk(KERN_DEBUG "called %s data[15] =%d\n", __func__,
		       data[15]);
		home_sensitivity = data[15];
	}
#else
	printk(KERN_DEBUG "called %s data[12] =%d,data[13] = %d\n", __func__,
	       data[12], data[13]);
	home_sensitivity = ((0x00FF & data[12]) << 8) | data[13];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", home_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[8] =%d,data[9] = %d\n",
		       __func__, data[8], data[9]);
		back_sensitivity = ((0x00FF & data[8]) << 8) | data[9];
	} else {
		printk(KERN_DEBUG "called %s data[13] =%d\n", __func__,
		       data[13]);
		back_sensitivity = data[13];
	}
#else
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__,
	       data[14], data[15]);
	back_sensitivity = ((0x00FF & data[14]) << 8) | data[15];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", back_sensitivity);
}

static ssize_t touchkey_search_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	u8 data[18] = { 0, };
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
	if (tkey_i2c->module_ver < 8) {
		printk(KERN_DEBUG "called %s data[6] =%d,data[7] = %d\n",
		       __func__, data[6], data[7]);
		search_sensitivity = ((0x00FF & data[6]) << 8) | data[7];
	} else {
		printk(KERN_DEBUG "called %s data[11] =%d\n", __func__,
		       data[11]);
		search_sensitivity = data[11];
	}
#else
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__,
	       data[16], data[17]);
	search_sensitivity = ((0x00FF & data[16]) << 8) | data[17];
#endif				/* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", search_sensitivity);
}
#else
static ssize_t touchkey_menu_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
#if defined(CONFIG_MACH_Q1_BD) \
|| (defined(CONFIG_MACH_C1) && defined(CONFIG_TARGET_LOCALE_KOR))\
	|| defined(CONFIG_MACH_T0)
	u8 data[14] = { 0, };
	int ret;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 14);

	printk(KERN_DEBUG "called %s data[12] = %d, data[13] =%d\n", __func__,
			data[12], data[13]);
	menu_sensitivity = ((0x00FF & data[12]) << 8) | data[13];
	printk(KERN_DEBUG "called %s menu_sensitivity =%d\n", __func__,
			menu_sensitivity);

#else
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	menu_sensitivity = data[7];
#endif
	return sprintf(buf, "%d\n", menu_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
#if defined(CONFIG_MACH_Q1_BD) \
	|| (defined(CONFIG_MACH_C1) && defined(CONFIG_TARGET_LOCALE_KOR))\
	|| defined(CONFIG_MACH_T0)
	u8 data[14] = { 0, };
	int ret;

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 14);

	printk(KERN_DEBUG "called %s data[10] = %d, data[11] =%d\n", __func__,
			data[10], data[11]);
	back_sensitivity =((0x00FF & data[10]) << 8) | data[11];
	printk(KERN_DEBUG "called %s back_sensitivity =%d\n", __func__,
			back_sensitivity);
#else
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "called %s\n", __func__);
	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 10);
	back_sensitivity = data[9];
#endif
	return sprintf(buf, "%d\n", back_sensitivity);
}
#endif

#if defined(TK_HAS_AUTOCAL)
static ssize_t autocalibration_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int data;

	sscanf(buf, "%d\n", &data);

	if (data == 1)
		touchkey_autocalibration(tkey_i2c);

	return size;
}

static ssize_t autocalibration_status(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	u8 data[6];
	int ret;
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);

	printk(KERN_DEBUG "[Touchkey] %s\n", __func__);

	ret = i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 6);
	if ((data[5] & TK_BIT_AUTOCAL))
		return sprintf(buf, "Enabled\n");
	else
		return sprintf(buf, "Disabled\n");

}
#endif				/* CONFIG_TARGET_LOCALE_NA */

static ssize_t touch_sensitivity_control(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	unsigned char data = 0x40;
	i2c_touchkey_write(tkey_i2c->client, &data, 1);
	return size;
}

static ssize_t set_touchkey_firm_version_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return sprintf(buf, "0x%x\n", TK_FIRMWARE_VER);
}

static ssize_t set_touchkey_update_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;
	int retry = 3;

	tkey_i2c->update_status = TK_UPDATE_DOWN;

	disable_irq(tkey_i2c->irq);

#ifdef TEST_JIG_MODE
	unsigned char get_touch = 0x40;
#endif

	while (retry--) {
		if (ISSP_main(tkey_i2c) == 0) {
			printk(KERN_ERR
			       "[TouchKey]Touchkey_update succeeded\n");
			tkey_i2c->update_status = TK_UPDATE_PASS;
			count = 1;
			msleep(50);
			break;
		}
		printk(KERN_ERR "touchkey_update failed... retry...\n");
	}
	if (retry <= 0) {
		/* disable ldo11 */
		tkey_i2c->pdata->power_on(0);
		count = 0;
		printk(KERN_ERR "[TouchKey]Touchkey_update fail\n");
		tkey_i2c->update_status = TK_UPDATE_FAIL;
		enable_irq(tkey_i2c->irq);
		return count;
	}

#ifdef TEST_JIG_MODE
	i2c_touchkey_write(tkey_i2c->client, &get_touch, 1);
#endif

#if defined(TK_HAS_AUTOCAL)
	touchkey_autocalibration(tkey_i2c);
#endif

	enable_irq(tkey_i2c->irq);

	return count;

}

static ssize_t set_touchkey_firm_version_read_show(struct device *dev,
						   struct device_attribute
						   *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	char data[3] = { 0, };
	int count;

	i2c_touchkey_read(tkey_i2c->client, KEYCODE_REG, data, 3);
	count = sprintf(buf, "0x%x\n", data[1]);

	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%x\n", data[1]);
	printk(KERN_DEBUG "[TouchKey] module_version_read 0x%x\n", data[2]);
	return count;
}

static ssize_t set_touchkey_firm_status_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);
	int count = 0;

	printk(KERN_DEBUG
	       "[TouchKey] touch_update_read: update_status %d\n",
	       tkey_i2c->update_status);

	if (tkey_i2c->update_status == TK_UPDATE_PASS)
		count = sprintf(buf, "PASS\n");
	else if (tkey_i2c->update_status == TK_UPDATE_DOWN)
		count = sprintf(buf, "Downloading\n");
	else if (tkey_i2c->update_status == TK_UPDATE_FAIL)
		count = sprintf(buf, "Fail\n");

	return count;
}

static ssize_t sec_keypad_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&tkey_i2c->keypad_enable));
}

static ssize_t sec_keypad_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct touchkey_i2c *tkey_i2c = dev_get_drvdata(dev);

	unsigned int val = 0;
	sscanf(buf, "%d", &val);
	val = (val == 0 ? 0 : 1);
	atomic_set(&tkey_i2c->keypad_enable, val);
	if (val) {
		set_bit(KEY_BACK, tkey_i2c->input_dev->keybit);
		set_bit(KEY_MENU, tkey_i2c->input_dev->keybit);
		set_bit(KEY_HOME, tkey_i2c->input_dev->keybit);
	} else {
		clear_bit(KEY_BACK, tkey_i2c->input_dev->keybit);
		clear_bit(KEY_MENU, tkey_i2c->input_dev->keybit);
		clear_bit(KEY_HOME, tkey_i2c->input_dev->keybit);
	}
	input_sync(tkey_i2c->input_dev);

	return count;
}

static DEVICE_ATTR(keypad_enable, S_IRUGO|S_IWUSR, sec_keypad_enable_show,
	      sec_keypad_enable_store);

static DEVICE_ATTR(recommended_version, S_IRUGO | S_IWUSR | S_IWGRP,
		   touch_version_read, touch_version_write);
static DEVICE_ATTR(updated_version, S_IRUGO | S_IWUSR | S_IWGRP,
		   touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   touchkey_led_control);
static DEVICE_ATTR(touchkey_menu, S_IRUGO | S_IWUSR | S_IWGRP,
		   touchkey_menu_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO | S_IWUSR | S_IWGRP,
		   touchkey_back_show, NULL);

#if defined(TK_USE_4KEY)
static DEVICE_ATTR(touchkey_home, S_IRUGO, touchkey_home_show, NULL);
static DEVICE_ATTR(touchkey_search, S_IRUGO, touchkey_search_show, NULL);
#endif

static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   touch_sensitivity_control);
static DEVICE_ATTR(touchkey_firm_update, S_IRUGO | S_IWUSR | S_IWGRP,
	set_touchkey_update_show, NULL);
static DEVICE_ATTR(touchkey_firm_update_status, S_IRUGO | S_IWUSR | S_IWGRP,
	set_touchkey_firm_status_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP,
	set_touchkey_firm_version_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP,
		   set_touchkey_firm_version_read_show, NULL);
#ifdef LED_LDO_WITH_REGULATOR
static DEVICE_ATTR(touchkey_brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   brightness_control);
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT)
static DEVICE_ATTR(touchkey_autocal_start, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   set_touchkey_autocal_testmode);
#endif

#if defined(TK_HAS_AUTOCAL)
static DEVICE_ATTR(touchkey_raw_data0, S_IRUGO, touchkey_raw_data0_show, NULL);
static DEVICE_ATTR(touchkey_raw_data1, S_IRUGO, touchkey_raw_data1_show, NULL);
static DEVICE_ATTR(touchkey_raw_data2, S_IRUGO, touchkey_raw_data2_show, NULL);
static DEVICE_ATTR(touchkey_raw_data3, S_IRUGO, touchkey_raw_data3_show, NULL);
static DEVICE_ATTR(touchkey_idac0, S_IRUGO, touchkey_idac0_show, NULL);
static DEVICE_ATTR(touchkey_idac1, S_IRUGO, touchkey_idac1_show, NULL);
static DEVICE_ATTR(touchkey_idac2, S_IRUGO, touchkey_idac2_show, NULL);
static DEVICE_ATTR(touchkey_idac3, S_IRUGO, touchkey_idac3_show, NULL);
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, touchkey_threshold_show, NULL);
static DEVICE_ATTR(autocal_enable, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   autocalibration_enable);
static DEVICE_ATTR(autocal_stat, S_IRUGO | S_IWUSR | S_IWGRP,
		   autocalibration_status, NULL);
#endif

static struct attribute *touchkey_attributes[] = {
	&dev_attr_recommended_version.attr,
	&dev_attr_updated_version.attr,
	&dev_attr_brightness.attr,
	&dev_attr_touchkey_menu.attr,
	&dev_attr_touchkey_back.attr,
#if defined(TK_USE_4KEY)
	&dev_attr_touchkey_home.attr,
	&dev_attr_touchkey_search.attr,
#endif
	&dev_attr_touch_sensitivity.attr,
	&dev_attr_touchkey_firm_update.attr,
	&dev_attr_touchkey_firm_update_status.attr,
	&dev_attr_touchkey_firm_version_phone.attr,
	&dev_attr_touchkey_firm_version_panel.attr,
#ifdef LED_LDO_WITH_REGULATOR
	&dev_attr_touchkey_brightness.attr,
#endif
#if defined(CONFIG_TARGET_LOCALE_NAATT)
	&dev_attr_touchkey_autocal_start.attr,
#endif
#if defined(TK_HAS_AUTOCAL)
	&dev_attr_touchkey_raw_data0.attr,
	&dev_attr_touchkey_raw_data1.attr,
	&dev_attr_touchkey_raw_data2.attr,
	&dev_attr_touchkey_raw_data3.attr,
	&dev_attr_touchkey_idac0.attr,
	&dev_attr_touchkey_idac1.attr,
	&dev_attr_touchkey_idac2.attr,
	&dev_attr_touchkey_idac3.attr,
	&dev_attr_touchkey_threshold.attr,
	&dev_attr_autocal_enable.attr,
	&dev_attr_autocal_stat.attr,
	&dev_attr_keypad_enable.attr,
#endif
	NULL,
};

static struct attribute_group touchkey_attr_group = {
	.attrs = touchkey_attributes,
};

#ifdef CONFIG_TOUCHKEY_BLN
static void touchkey_activate(void)
{
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);
	if (bln_tkey_i2c == NULL) {
		printk(KERN_ERR "%s: no bln_tkey_i2c\n", __func__);
		return;
	}

	if (bln_tkey_i2c->pdata == NULL) {
		printk(KERN_ERR "%s: no pdata\n", __func__);
		return;
	}

	if (!blnww && !wake_lock_active(&bln_wake_lock)) {
		printk(KERN_DEBUG "[TouchKey-BLN] touchkey get wake_lock\n");
		wake_lock(&bln_wake_lock);
	}
	mutex_lock(&touchkey_enable_mutex);
	printk(KERN_DEBUG "[TouchKey-BLN] touchkey activate.\n");
	bln_tkey_i2c->pdata->power_on(1);

	msleep(50);
	bln_tkey_i2c->pdata->led_power_on(1);

	touchkey_enable = 1;
	mutex_unlock(&touchkey_enable_mutex);
}

static void touchkey_deactivate(void) {
	mutex_lock(&touchkey_enable_mutex);
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);

	if (bln_tkey_i2c == NULL) {
		printk(KERN_ERR "%s: no bln_tkey_i2c\n", __func__);
		return;
	}

	if (bln_tkey_i2c->pdata == NULL) {
		printk(KERN_ERR "%s: no pdata\n", __func__);
		return;
	}

	bln_tkey_i2c->pdata->led_power_on(0);
	bln_tkey_i2c->pdata->power_on(0);

	if (wake_lock_active(&bln_wake_lock)) {
		printk(KERN_DEBUG "[TouchKey-BLN] touchkey clear wake_lock\n");
		wake_unlock(&bln_wake_lock);
	}
	touchkey_enable = 0;
	mutex_unlock(&touchkey_enable_mutex);
}

static void bln_early_suspend(struct early_suspend *h) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);
	bln_suspended = true;
}

static void bln_late_resume(struct early_suspend *h) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);

	bln_suspended = false;
	if (wake_lock_active(&bln_wake_lock)) {
		printk(KERN_DEBUG "[TouchKey-BLN] clear wake lock \n");
		wake_unlock(&bln_wake_lock);
	}
}

static struct early_suspend bln_suspend_data = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = bln_early_suspend,
	.resume = bln_late_resume,
};

static void enable_touchkey_backlights(void) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);

	if (bln_tkey_i2c == NULL) {
		printk(KERN_ERR "%s: no bln_tkey_i2c\n", __func__);
		return;
	}

	if (bln_tkey_i2c->client == NULL) {
		printk(KERN_ERR "%s: no client\n", __func__);
		return;
	}

	int status = 1;
	printk(KERN_ERR "[TouchKey-BLN] enable LED from BLN app\n");
	i2c_touchkey_write(bln_tkey_i2c->client, (u8 *)&status, 1);
}

static void disable_touchkey_backlights(void) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);

	if (bln_tkey_i2c == NULL) {
		printk(KERN_ERR "%s: no bln_tkey_i2c\n", __func__);
		return;
	}

	if (bln_tkey_i2c->client == NULL) {
		printk(KERN_ERR "%s: no client\n", __func__);
		return;
	}

	int status = 2;
	printk(KERN_ERR "[TouchKey-BLN] disable LED from BLN app\n");
	i2c_touchkey_write(bln_tkey_i2c->client, (u8 *)&status, 1);
}

static void enable_led_notification(void) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);

	if (bln_enabled) {
		if (touchkey_enable != 1) {
			if (bln_suspended) {
				touchkey_activate();
			}
		}
		if (touchkey_enable == 1) {
			printk(KERN_DEBUG "[TouchKey-BLN] bln_ongoing set to true\n");
			bln_ongoing = true;
			set_touch_constraints(true);
			enable_touchkey_backlights();
		}

		/* See if a timeout value has been set for the notification */
		if (notification_timeout > 0) {
			/* restart the timer */
			mod_timer(&notification_timer, jiffies + msecs_to_jiffies(notification_timeout));
		}
		if (breathing) mod_timer(&breathing_timer, jiffies + 4);
	}
}

static void disable_led_notification(void) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s\n", __func__);

	down(&enable_sem);
	printk(KERN_DEBUG "[TouchKey-BLN] bln_ongoing set to false\n");
	bln_blink_enabled = false;
	set_touch_constraints(false);
	bln_ongoing = false;

	if (touchkey_enable == 1) {
		disable_touchkey_backlights();
		if (bln_suspended) {
			touchkey_deactivate();
		}
		/* a notification timeout was set, disable the timer */
		if (notification_timeout > 0) {
			del_timer(&notification_timer);
		}
		if (breathing) stop_breathing();
        }
	up(&enable_sem);
}

static ssize_t bln_status_read(struct device *dev, struct device_attribute *attr, char *buf) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s: %u\n", __func__, (bln_enabled ? 1 : 0 ));

	return sprintf(buf,"%u\n", (bln_enabled ? 1 : 0 ));
}

static ssize_t bln_status_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s: %s\n", __func__, buf);

	unsigned int data;
	if (sscanf(buf,"%u\n", &data) == 1) {
		if (data == 1) {
			bln_enabled = true;
		}
		if (data == 0) {
			bln_enabled = false;
			if (bln_ongoing)
				disable_led_notification();
	        }
	} else {
		printk(KERN_ERR "[TouchKey-BLN] %s: %s %u\n", __func__, buf, data);
	}
	return size;
}

static ssize_t notification_led_status_read( struct device *dev, struct device_attribute *attr, char *buf ){
	printk(KERN_DEBUG "[TouchKey-BLN] %s: %u\n", __func__, (bln_ongoing ? 1 : 0 ));

	return sprintf(buf,"%u\n", (bln_ongoing ? 1 : 0 ));
}

static ssize_t notification_led_status_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s: %s\n", __func__, buf);

	unsigned int data;
	if (sscanf(buf,"%u\n", &data ) == 1) {
		if (data == 0 || data == 1) {
			if (data == 1)
				enable_led_notification();

			if (data == 0)
				disable_led_notification();
		} else {
			printk(KERN_ERR "[TouchKey-BLN] %s: Invalid value %s\n", __func__, buf);
		}
	} else {
		printk(KERN_ERR "[TouchKey-BLN] %s: Invalid value %s\n", __func__, buf);
	}
	return size;
}

static ssize_t blink_control_read (struct device *dev, struct device_attribute *attr, char *buf) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s: %u\n", __func__, (bln_blink_enabled ? 1 : 0));

	return sprintf( buf, "%u\n", (bln_blink_enabled ? 1 : 0));
}

static ssize_t blink_control_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	printk(KERN_DEBUG "[TouchKey-BLN] %s: %s\n", __func__, buf);

	unsigned int data;
	if (sscanf(buf, "%u\n", &data ) == 1) {
		if (data == 0 || data == 1) {
			if (data == 1) {
				bln_blink_enabled = true;
				disable_touchkey_backlights();
			}

			if (data == 0) {
				bln_blink_enabled = false;
				enable_touchkey_backlights();
			}
		}
	}

	return size;
}

static ssize_t bln_version( struct device *dev, struct device_attribute *attr, char *buf ){
        return sprintf(buf,"%u\n", BLN_VERSION);
}

static DEVICE_ATTR(blink_control, S_IRUGO | S_IWUGO, blink_control_read, blink_control_write );
static DEVICE_ATTR(enabled, S_IRUGO | S_IWUGO, bln_status_read, bln_status_write );
static DEVICE_ATTR(notification_led, S_IRUGO | S_IWUGO, notification_led_status_read,  notification_led_status_write );
static DEVICE_ATTR(version, S_IRUGO, bln_version, NULL );

static struct attribute *bln_notification_attributes[] = {
        &dev_attr_blink_control.attr,
        &dev_attr_enabled.attr,
        &dev_attr_notification_led.attr,
        &dev_attr_version.attr,
        NULL
};

static struct attribute_group bln_notification_group = {
        .attrs = bln_notification_attributes,
};

static struct miscdevice bln_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name  = "backlightnotification",
};

/*
 * Start of the main LED Notify code block
 */

static void bl_on()
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	/* Do nothing if LED is disabled */
	if (led_disabled)  {
		printk(KERN_DEBUG "[TouchKey-LED] %s: Doing nothing, LED is disabled\n", __func__);
		return;
	}

	/* Do nothing if there is an active notification */
	if (bln_ongoing == 1 || touchkey_enable != 1)
		return;

	/* Turn the LED on */
	schedule_work(&led_fadein_work);

	/* Restart the timer */
	if (led_timeout > 0) {
		mod_timer(&led_timer, jiffies + msecs_to_jiffies(led_timeout));
	}

	return;
}
static void bl_off(struct work_struct *bl_off_work)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	/* Do nothing if LED is disabled */
	if (led_disabled)  {
		printk(KERN_DEBUG "[TouchKey-LED] %s: Doing nothing, LED is disabled\n", __func__);
		return;
	}

	/* do nothing if there is an active notification */
	if (bln_ongoing == 1)
		return;

	/* Turn the lights off */
	schedule_work(&led_fadeout_work);

	return;
}

static void handle_led_timeout(unsigned long data)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	/* we cannot call the timeout directly as it causes a kernel spinlock BUG, schedule it instead */
	schedule_work(&bl_off_work);
}

static void notification_off(struct work_struct *notification_off_work)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	/* do nothing if there is no active notification */
	if (bln_ongoing != 1 || touchkey_enable != 1)
		return;

	disable_touchkey_backlights();
	set_touch_constraints(false);
	bln_ongoing = 0;
}

static void handle_notification_timeout(unsigned long data)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	/* we cannot call the timeout directly as it causes a kernel spinlock BUG, schedule it instead */
	schedule_work(&notification_off_work);
}

//to prevent two consecutive touches restarting fadein/fadeout effect -gm
static DEFINE_MUTEX(led_fadein_mutex);
static DEFINE_MUTEX(led_fadeout_mutex);

static void led_fadeout_process(struct work_struct *work)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	if (bln_tkey_i2c == NULL) {
		printk(KERN_ERR "%s: no bln_tkey_i2c\n", __func__);
		return;
	}

	int i, status = 2;
	cancel_work_sync(&led_fadein_work);
	printk(KERN_DEBUG "[TouchKey-LED] %s: Unlock led_Fadein_mutex\n", __func__);
	mutex_unlock(&led_fadein_mutex);
	printk(KERN_DEBUG "[TouchKey-LED] %s: Unlock led_Fadein_mutex, done\n", __func__);

	if (led_fadeout) {
		if (!mutex_trylock(&led_fadeout_mutex)) {
			printk(KERN_DEBUG "[TouchKey-LED] %s: Ignoring, already doing fade out\n", __func__);
			return;
		}
		for (i = touchkey_voltage; i >= BL_MIN; i -= 50) {
			change_touch_key_led_voltage(i);
			msleep(FADEIN_STEP_MS);
		}
		mutex_unlock(&led_fadeout_mutex);
	}
	i2c_touchkey_write(bln_tkey_i2c->client, (u8 *)&status, 1);

	/* disable ldo18 */
	printk(KERN_DEBUG "[TouchKey-LED] %s: LED power off\n", __func__);
	bln_tkey_i2c->pdata->led_power_on(0);


	msleep(100); // wait for LED is actually turned off before restoring voltage.
	printk(KERN_DEBUG "[TouchKey-LED] %s: Restore voltage after turning LED off\n", __func__);
	touchkey_voltage = touchkey_voltage_brightness;
	change_touch_key_led_voltage(touchkey_voltage);
}

static void led_fadein_process(struct work_struct *work)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	if (bln_tkey_i2c == NULL) {
		printk(KERN_ERR "%s: no bln_tkey_i2c\n", __func__);
		return;
	}

	int i, status = 1;
	cancel_work_sync(&led_fadeout_work);
	mutex_unlock(&led_fadeout_mutex);

	touchkey_voltage = touchkey_voltage_brightness;
	if (led_fadein) {
		if (!mutex_trylock(&led_fadein_mutex)) {
			printk(KERN_DEBUG "[TouchKey-LED] %s: Ignoring, already doing fade in\n", __func__);
			return;
		}
		change_touch_key_led_voltage(BL_MIN);

		/* enable ldo18 */
		bln_tkey_i2c->pdata->led_power_on(1);

		i2c_touchkey_write(bln_tkey_i2c->client, (u8 *)&status, 1);
		for (i = BL_MIN; i <= touchkey_voltage; i += 50) {
			change_touch_key_led_voltage(i);
			msleep(FADEIN_STEP_MS);
		}
		mutex_unlock(&led_fadein_mutex);
	} else {
		change_touch_key_led_voltage(touchkey_voltage);

		/* enable ldo18 */
		bln_tkey_i2c->pdata->led_power_on(1);

		i2c_touchkey_write(bln_tkey_i2c->client, (u8 *)&status, 1);
	}
}

static ssize_t led_timeout_read_ms( struct device *dev, struct device_attribute *attr, char *buf )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %u\n", __func__, led_timeout);

	return sprintf(buf,"%d\n", led_timeout);
}

static ssize_t led_timeout_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %f\n", __func__, led_timeout/1000);

	return sprintf(buf,"%d\n", led_timeout/1000);
}

static ssize_t led_timeout_write_ms( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);


	int i = 0;
	sscanf(buf,"%d\n", &i);

	if (i < 0) return size;

	led_timeout = i;

	if (!led_disabled) {
		if (led_timeout == 0)
			del_timer(&led_timer);
		else
			mod_timer(&led_timer, jiffies + msecs_to_jiffies(led_timeout));
	}
	return size;
}

static ssize_t led_timeout_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);


	sscanf(buf,"%d\n", &led_timeout);
	led_timeout = led_timeout * 1000;

	if (!led_disabled) {
		if (led_timeout == 0) {
			del_timer(&led_timer);
			schedule_work(&led_fadein_work);
		} else
			mod_timer(&led_timer, jiffies + msecs_to_jiffies(led_timeout));
	}
	return size;
}

static ssize_t notification_timeout_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %u\n", __func__, notification_timeout);

	return sprintf(buf,"%d\n", notification_timeout);
}

static ssize_t notification_timeout_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);


	sscanf(buf,"%d\n", &notification_timeout);
	return size;
}

static void breathe(struct work_struct *notification_off_work)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	int data;
	if (bln_ongoing != 1 || touchkey_enable != 1)
		return;

	if (breathing_steps[breathing_step_idx].start <= breathing_steps[breathing_step_idx].end) {
		data = breathing_steps[breathing_step_idx].start +
			breathing_idx++ * breathing_steps[breathing_step_idx].step;
		if (data > breathing_steps[breathing_step_idx].end) {
			breathing_idx = 0;
			breathing_step_idx++;
			if (breathing_step_idx >= breathing_step_count) breathing_step_idx = 0;
			data = breathing_steps[breathing_step_idx].start;
		}
	} else {
		data = breathing_steps[breathing_step_idx].start -
			breathing_idx++ * breathing_steps[breathing_step_idx].step;
		if (data < breathing_steps[breathing_step_idx].end) {
			breathing_idx = 0;
			breathing_step_idx++;
			if (breathing_step_idx >= breathing_step_count) breathing_step_idx = 0;
			data = breathing_steps[breathing_step_idx].start;
		}
	}

	change_touch_key_led_voltage(data);
	mod_timer(&breathing_timer, jiffies + msecs_to_jiffies(breathing_steps[breathing_step_idx].period));
}

void stop_breathing(void)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	del_timer(&breathing_timer);
	change_touch_key_led_voltage(touchkey_voltage);
}

static void handle_breathe(unsigned long data)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	schedule_work(&breathe_work);
}

static ssize_t breathing_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %u\n", __func__, breathing);

	return sprintf(buf,"%d\n", breathing);
}
static ssize_t breathing_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);

	if (!strncmp(buf, "on", 2)) breathing = 1;
	else if (!strncmp(buf, "off", 3)) breathing = 0;
	else sscanf(buf,"%d\n", &breathing);
	if (breathing != 1) stop_breathing();
	return size;
}

void reset_breathing_steps(void)
{
	printk(KERN_DEBUG "[TouchKey-LED] %s\n", __func__);

	//this will reset steps to have steady bln
	breathing_step_count = 0;
	breathing_steps[0].start = 3000;
	breathing_steps[0].end = 3000;
	breathing_steps[0].period = 1000;
	breathing_steps[0].step = 50;
}

static ssize_t breathing_steps_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	int count = (breathing_step_count == 0 ? 1 : breathing_step_count);
	int i, len = 0;
	for (i = 0; i<count; i++) {
		len += sprintf(buf + len, "%dmV %dmV %dms %dmV\n", breathing_steps[i].start, breathing_steps[i].end,
						breathing_steps[i].period, breathing_steps[i].step);
	}
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);

	return len;
}

static ssize_t breathing_steps_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	int ret;
	int bstart, bend, bperiod, bstep;

	if (!strncmp(buf, "reset", 5)) {
		reset_breathing_steps();
		return size;
	}
	if (breathing_step_count >= MAX_BREATHING_STEPS) return -EINVAL;
	ret = sscanf(buf, "%d %d %d %d", &bstart, &bend, &bperiod, &bstep);
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);

	if (ret != 4) return -EINVAL;
	breathing_steps[breathing_step_count].start = bstart;
	breathing_steps[breathing_step_count].end = bend;
	breathing_steps[breathing_step_count].period = bperiod;
	breathing_steps[breathing_step_count].step = bstep;
	breathing_step_count++;
	breathing_idx = 0;
	breathing_step_idx = 0;
	return size;
}

static ssize_t led_fadein_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %u\n", __func__, led_fadein);

	return sprintf(buf,"%d\n", led_fadein);
}
static ssize_t led_fadein_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);

	if (!strncmp(buf, "on", 2)) led_fadein = 1;
	else if (!strncmp(buf, "off", 3)) led_fadein = 0;
	else sscanf(buf,"%d\n", &led_fadein);
	return size;
}

static ssize_t led_fadeout_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, led_fadeout);

	return sprintf(buf,"%d\n", led_fadeout);
}
static ssize_t led_fadeout_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{

	if (!strncmp(buf, "on", 2)) led_fadeout = 1;
	else if (!strncmp(buf, "off", 3)) led_fadeout = 0;
	else sscanf(buf,"%d\n", &led_fadeout);

	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);

	return size;
}

static ssize_t led_on_touch_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %u\n", __func__, led_on_touch);

	return sprintf(buf,"%d\n", led_on_touch);
}
static ssize_t led_on_touch_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);

	if (!strncmp(buf, "on", 2)) led_on_touch = 1;
	else if (!strncmp(buf, "off", 3)) led_on_touch = 0;
	else sscanf(buf,"%d\n", &led_on_touch);
	return size;
}

static ssize_t dyn_brightness_read( struct device *dev, struct device_attribute *attr, char *buf ) {
	printk(KERN_DEBUG "[TouchKey-LED] %s: %u\n", __func__, (dyn_brightness ? 1 : 0));

	return sprintf(buf,"%u\n", (dyn_brightness ? 1 : 0));
}

static ssize_t dyn_brightness_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size ) {
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);

	unsigned int data;
	int old_dyn = dyn_brightness;

	if (sscanf(buf,"%u\n", &data) == 1) {
		if (data == 1) dyn_brightness = 1;
		if (data == 0) dyn_brightness = 0;
	} else {
		if (!strncmp(buf, "on", 2)) dyn_brightness = 1;
		if (!strncmp(buf, "off", 3)) dyn_brightness = 0;
	}

	return size;
}

static ssize_t blnww_status_read( struct device *dev, struct device_attribute *attr, char *buf ) {
	printk(KERN_DEBUG "[TouchKey-LED] %s: %u\n", __func__, (blnww ? 1 : 0));

	return sprintf(buf,"%u\n", (blnww ? 1 : 0));
}

static ssize_t blnww_status_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size ) {
	printk(KERN_DEBUG "[TouchKey-LED] %s: %s\n", __func__, buf);


	unsigned int data;

	if (sscanf(buf,"%u\n", &data) == 1) {
		if (data == 1) blnww = 1;
		if (data == 0) blnww = 0;
	} else {
		if (!strncmp(buf, "on", 2)) blnww = 1;
		if (!strncmp(buf, "off", 3)) blnww = 0;
	}
	return size;
}

static struct miscdevice led_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "notification",
};

static DEVICE_ATTR(led, S_IRUGO | S_IWUGO, notification_led_status_read, notification_led_status_write);
static DEVICE_ATTR(notification_timeout, S_IRUGO | S_IWUGO, notification_timeout_read, notification_timeout_write);
static DEVICE_ATTR(led_timeout_ms, S_IRUGO | S_IWUGO, led_timeout_read_ms, led_timeout_write_ms);
static DEVICE_ATTR(led_timeout, S_IRUGO | S_IWUGO, led_timeout_read, led_timeout_write);
static DEVICE_ATTR(bl_timeout, S_IRUGO | S_IWUGO, led_timeout_read, led_timeout_write);
static DEVICE_ATTR(notification_enabled, S_IRUGO | S_IWUGO, bln_status_read, bln_status_write);
static DEVICE_ATTR(breathing, S_IRUGO | S_IWUGO, breathing_read, breathing_write);
static DEVICE_ATTR(breathing_steps, S_IRUGO | S_IWUGO, breathing_steps_read, breathing_steps_write);
static DEVICE_ATTR(led_fadein, S_IRUGO | S_IWUGO, led_fadein_read, led_fadein_write);
static DEVICE_ATTR(led_fadeout, S_IRUGO | S_IWUGO, led_fadeout_read, led_fadeout_write);
static DEVICE_ATTR(led_on_touch, S_IRUGO | S_IWUGO, led_on_touch_read, led_on_touch_write);
static DEVICE_ATTR(dyn_brightness, S_IRUGO | S_IWUSR | S_IWGRP,	dyn_brightness_read, dyn_brightness_write);
static DEVICE_ATTR(blnww, S_IRUGO | S_IWUGO, blnww_status_read,  blnww_status_write);

static struct attribute *led_notification_attributes[] = {
	&dev_attr_led.attr,
	&dev_attr_led_timeout_ms.attr,
	&dev_attr_bl_timeout.attr,
	&dev_attr_notification_timeout.attr,
	&dev_attr_notification_enabled.attr,
	&dev_attr_breathing.attr,
	&dev_attr_breathing_steps.attr,
	&dev_attr_led_fadein.attr,
	&dev_attr_led_fadeout.attr,
	&dev_attr_led_on_touch.attr,
	&dev_attr_dyn_brightness.attr,
	&dev_attr_blnww.attr,
    NULL
};

static struct attribute_group led_notification_group = {
        .attrs = led_notification_attributes,
};
#endif

static int i2c_touchkey_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct touchkey_platform_data *pdata = client->dev.platform_data;
	struct touchkey_i2c *tkey_i2c;

	struct input_dev *input_dev;
	int err = 0;
	unsigned char data;
	int i;
	int ret;

	printk(KERN_DEBUG "[TouchKey] i2c_touchkey_probe\n");

	if (pdata == NULL) {
		printk(KERN_ERR "%s: no pdata\n", __func__);
		return -ENODEV;
	}

	/*Check I2C functionality */
	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (ret == 0) {
		printk(KERN_ERR "[Touchkey] No I2C functionality found\n");
		ret = -ENODEV;
		return ret;
	}

	/*Obtain kernel memory space for touchkey i2c */
	tkey_i2c = kzalloc(sizeof(struct touchkey_i2c), GFP_KERNEL);
	if (NULL == tkey_i2c) {
		printk(KERN_ERR "[Touchkey] failed to allocate tkey_i2c.\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();

	if (!input_dev) {
		printk(KERN_ERR"[Touchkey] failed to allocate input device\n");
		kfree(tkey_i2c);
		return -ENOMEM;
	}

	input_dev->name = "sec_touchkey";
	input_dev->phys = "sec_touchkey/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &client->dev;

	/*tkey_i2c*/
	tkey_i2c->pdata = pdata;
	tkey_i2c->input_dev = input_dev;
	tkey_i2c->client = client;
	tkey_i2c->irq = client->irq;
	tkey_i2c->name = "sec_touchkey";

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_LED, input_dev->evbit);
	set_bit(LED_MISC, input_dev->ledbit);
	set_bit(EV_KEY, input_dev->evbit);

	atomic_set(&tkey_i2c->keypad_enable, 1);

	for (i = 1; i < touchkey_count; i++)
		set_bit(touchkey_keycode[i], input_dev->keybit);

	input_set_drvdata(input_dev, tkey_i2c);

	ret = input_register_device(input_dev);
	if (ret) {
		printk(KERN_ERR"[Touchkey] failed to register input device\n");
		input_free_device(input_dev);
		kfree(tkey_i2c);
		return err;
	}

	INIT_WORK(&tkey_i2c->update_work, touchkey_update_func);

	tkey_i2c->pdata->power_on(1);
	msleep(50);

	touchkey_enable = 1;
	data = 1;

	/*sysfs*/
	tkey_i2c->dev = device_create(sec_class, NULL, 0, NULL, "sec_touchkey");

	if (IS_ERR(tkey_i2c->dev)) {
		printk(KERN_ERR "Failed to create device(tkey_i2c->dev)!\n");
		input_unregister_device(input_dev);
	} else {
		dev_set_drvdata(tkey_i2c->dev, tkey_i2c);
		ret = sysfs_create_group(&tkey_i2c->dev->kobj,
					&touchkey_attr_group);
		if (ret) {
			printk(KERN_ERR
				"[TouchKey]: failed to create sysfs group\n");
		}
	}

#if defined(CONFIG_MACH_M0) || defined(CONFIG_MACH_C1)
	gpio_request(GPIO_OLED_DET, "OLED_DET");
	ret = gpio_get_value(GPIO_OLED_DET);
	printk(KERN_DEBUG
	"[TouchKey] OLED_DET = %d\n", ret);

	if (ret == 0) {
		printk(KERN_DEBUG
		"[TouchKey] device wasn't connected to board\n");

		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}
#else
	ret = touchkey_i2c_check(tkey_i2c);
	if (ret < 0) {
		printk(KERN_DEBUG"[TouchKey] probe failed\n");
		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}
#endif

	ret =
		request_threaded_irq(tkey_i2c->irq, NULL, touchkey_interrupt,
				IRQF_DISABLED | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT, tkey_i2c->name, tkey_i2c);
	if (ret < 0) {
		printk(KERN_ERR
			"[Touchkey]: failed to request irq(%d) - %d\n",
			tkey_i2c->irq, ret);
		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}

	tkey_i2c->pdata->led_power_on(1);

#if defined(TK_HAS_FIRMWARE_UPDATE)
	ret = touchkey_firmware_update(tkey_i2c);
	if (ret < 0) {
		printk(KERN_ERR
			"[Touchkey]: failed firmware updating process (%d)\n",
			ret);
		input_unregister_device(input_dev);
		touchkey_probe = false;
		return -EBUSY;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	tkey_i2c->early_suspend.suspend =
		(void *)sec_touchkey_early_suspend;
	tkey_i2c->early_suspend.resume =
		(void *)sec_touchkey_late_resume;
	register_early_suspend(&tkey_i2c->early_suspend);
#endif

#if defined(TK_HAS_AUTOCAL)
	touchkey_autocalibration(tkey_i2c);
#endif
	set_touchkey_debug('K');

#ifdef CONFIG_TOUCHKEY_BLN
        bln_tkey_i2c = tkey_i2c;
        err = misc_register( &bln_device );
        if( err ){
            printk(KERN_ERR "[TouchKey-BLN] sysfs misc_register failed.\n");
        }else{
            if( sysfs_create_group( &bln_device.this_device->kobj, &bln_notification_group) < 0){
                printk(KERN_ERR "[TouchKey-BLN] sysfs create group failed.\n");
            }
        }

        /* BLN early suspend */
        register_early_suspend(&bln_suspend_data);

	err = misc_register(&led_device);
	if (err) {
		printk(KERN_ERR "[LED] sysfs misc_register failed.\n");
	} else {
		if (sysfs_create_group( &led_device.this_device->kobj, &led_notification_group) < 0) {
			printk(KERN_ERR "[LED] sysfs create group failed.\n");
		}
	}

	/* Setup the timer for the timeouts */
	setup_timer(&led_timer, handle_led_timeout, 0);
	setup_timer(&notification_timer, handle_notification_timeout, 0);
	setup_timer(&breathing_timer, handle_breathe, 0);

	led_timeout = 0;
	reset_breathing_steps();
        /* wake lock for BLN */
        wake_lock_init(&bln_wake_lock, WAKE_LOCK_SUSPEND, "bln_wake_lock");
#endif
	return 0;
}

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = "sec_touchkey_driver",
	},
	.id_table = sec_touchkey_id,
	.probe = i2c_touchkey_probe,
};

static int __init touchkey_init(void)
{
	int ret = 0;

#if defined(CONFIG_MACH_M0)
	if (system_rev < TOUCHKEY_FW_UPDATEABLE_HW_REV) {
		printk(KERN_DEBUG "[Touchkey] Doesn't support this board rev %d\n",
				system_rev);
		return 0;
	}
#elif defined(CONFIG_MACH_C1)
	if (system_rev < TOUCHKEY_FW_UPDATEABLE_HW_REV) {
		printk(KERN_DEBUG "[Touchkey] Doesn't support this board rev %d\n",
				system_rev);
		return 0;
	}
#endif

#ifdef TEST_JIG_MODE
	unsigned char get_touch = 0x40;
#endif

	ret = i2c_add_driver(&touchkey_i2c_driver);

	if (ret) {
		printk(KERN_ERR
	       "[TouchKey] registration failed, module not inserted.ret= %d\n",
	       ret);
	}
#ifdef TEST_JIG_MODE
	i2c_touchkey_write(tkey_i2c->client, &get_touch, 1);
#endif
	return ret;
}

static void __exit touchkey_exit(void)
{
	printk(KERN_DEBUG "[TouchKey] %s\n", __func__);
	i2c_del_driver(&touchkey_i2c_driver);
#ifdef CONFIG_TOUCHKEY_BLN
	bln_tkey_i2c = NULL;
	misc_deregister(&bln_device);
	wake_lock_destroy(&bln_wake_lock);
	misc_deregister(&led_device);
	del_timer(&led_timer);
	del_timer(&notification_timer);
	del_timer(&breathing_timer);
#endif
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("touch keypad");
