/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * s2mpg12.h
 *
 * Copyright (C) 2016 Samsung Electrnoics
 *
 * Driver for the s2mpg12
 */

#ifndef __S2MPG12_MFD_H__
#define __S2MPG12_MFD_H__

#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/regmap.h>

#include "s2mpg1x-register.h"
#include "s2mpg12-meter.h"

#define S2MPG12_MFD_DEV_NAME "s2mpg12"

/**
 * sec_regulator_data - regulator data
 * @id: regulator id
 * @initdata: regulator init data (constraints, supplies, ...)
 */
struct s2mpg12_regulator_data {
	int id;
	struct regulator_init_data *initdata;
	struct device_node *reg_node;
};

enum s2mpg12_irq_source {
	S2MPG12_IRQS_PMIC_INT1 = 0,
	S2MPG12_IRQS_PMIC_INT2,
	S2MPG12_IRQS_PMIC_INT3,
	S2MPG12_IRQS_PMIC_INT4,
	S2MPG12_IRQS_PMIC_INT5,
	S2MPG12_IRQS_METER_INT1,
	S2MPG12_IRQS_METER_INT2,

	S2MPG12_IRQ_GROUP_NR,
};

#define S2MPG12_NUM_IRQ_PMIC_REGS	5
#define S2MPG12_NUM_IRQ_METER_REGS	2

enum s2mpg12_device_type {
	S2MPG12X,
};

enum s2mpg12_types {
	TYPE_S2MPG12,
};

struct s2mpg12_platform_data {
	/* Device Data */
	int device_type;

	/* IRQ */
	int irq_base;
	bool wakeup;

	/* VGPIO */
	u32 *sel_vgpio;

	/* Regulator */
	int num_regulators;
	struct s2mpg12_regulator_data *regulators;
	struct sec_opmode_data *opmode;

	int smpl_warn_pin;
	unsigned int smpl_warn_lvl;
	unsigned int smpl_warn_hys;
	unsigned int smpl_warn_lbdt;

	int b2_ocp_warn_pin;
	unsigned int b2_ocp_warn_en;
	unsigned int b2_ocp_warn_cnt;
	unsigned int b2_ocp_warn_dvs_mask;
	unsigned int b2_ocp_warn_lvl;

	int b3_ocp_warn_pin;
	unsigned int b3_ocp_warn_en;
	unsigned int b3_ocp_warn_cnt;
	unsigned int b3_ocp_warn_dvs_mask;
	unsigned int b3_ocp_warn_lvl;

	int b10_ocp_warn_pin;
	unsigned int b10_ocp_warn_en;
	unsigned int b10_ocp_warn_cnt;
	unsigned int b10_ocp_warn_dvs_mask;
	unsigned int b10_ocp_warn_lvl;

	int b2_soft_ocp_warn_pin;
	unsigned int b2_soft_ocp_warn_en;
	unsigned int b2_soft_ocp_warn_cnt;
	unsigned int b2_soft_ocp_warn_dvs_mask;
	unsigned int b2_soft_ocp_warn_lvl;

	int b3_soft_ocp_warn_pin;
	unsigned int b3_soft_ocp_warn_en;
	unsigned int b3_soft_ocp_warn_cnt;
	unsigned int b3_soft_ocp_warn_dvs_mask;
	unsigned int b3_soft_ocp_warn_lvl;

	int b10_soft_ocp_warn_pin;
	unsigned int b10_soft_ocp_warn_en;
	unsigned int b10_soft_ocp_warn_cnt;
	unsigned int b10_soft_ocp_warn_dvs_mask;
	unsigned int b10_soft_ocp_warn_lvl;

	unsigned int buck_ocp_ctrl1;
	unsigned int buck_ocp_ctrl2;
	unsigned int buck_ocp_ctrl3;
	unsigned int buck_ocp_ctrl4;
	unsigned int buck_ocp_ctrl5;

	/* ---- RTC ---- */
	struct sec_wtsr_smpl *wtsr_smpl;
	struct rtc_time *init_time;
	int osc_bias_up;
	int cap_sel;
	int osc_xin;
	int osc_xout;

	void *meter;
};

struct s2mpg12_dev {
	/* Device Data */
	struct device *dev;
	struct s2mpg12_platform_data *pdata;
	struct regmap *regmap;
	int device_type;
	int type;

	/* pmic VER/REV register */
	enum S2MPG12_pmic_rev pmic_rev; /* pmic Rev */

	/* I2C Client */
	struct i2c_client *i2c;
	struct i2c_client *pmic;
	struct i2c_client *rtc;
	struct i2c_client *meter;
	struct i2c_client *wlwp;
	struct i2c_client *gpio;
	struct i2c_client *mt_trim;
	struct i2c_client *trim;
	/* mutex for i2c */
	struct mutex i2c_lock;

	/* IRQ */
	int irq;
	int irq_base;
	bool wakeup;

	/* VGPIO_RX_MONITOR */
	void __iomem *mem_base;

	/* VGPIO_INTC0_IPEND */
	void __iomem *sysreg_pending;

	/* mutex for s2mpg12 irq handling */
	struct mutex irqlock;
	int irq_masks_cur[S2MPG12_IRQ_GROUP_NR];
	int irq_masks_cache[S2MPG12_IRQ_GROUP_NR];

	/* Work queue */
	struct workqueue_struct *irq_wqueue;
	struct delayed_work irq_work;
};

struct s2mpg12_pmic {
	struct s2mpg12_dev *iodev;
	struct i2c_client *i2c;

	/* mutex for s2mpg12 regulator */
	struct mutex lock;
	struct regulator_dev **rdev;
	unsigned int *opmode;
	int num_regulators;
	int *buck_ocp_irq;
	int cpu1_ocp_warn_irq;
	int soft_cpu1_ocp_warn_irq;
	int cpu2_ocp_warn_irq;
	int soft_cpu2_ocp_warn_irq;
	int tpu_ocp_warn_irq;
	int soft_tpu_ocp_warn_irq;
#if IS_ENABLED(CONFIG_DRV_SAMSUNG_PMIC)
	struct device *dev;
	u16 read_addr;
#endif
};

int s2mpg12_irq_init(struct s2mpg12_dev *s2mpg12);
void s2mpg12_irq_exit(struct s2mpg12_dev *s2mpg12);
void s2mpg13_call_notifier(void);

/* S2MPG12 shared i2c API function */
int s2mpg12_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
int s2mpg12_bulk_read(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg12_write_reg(struct i2c_client *i2c, u8 reg, u8 value);
int s2mpg12_bulk_write(struct i2c_client *i2c, u8 reg, int count, u8 *buf);
int s2mpg12_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask);

u8 s2mpg12_get_rev_id(void);
int pmic_read_pwrkey_status(void);

#endif /* __S2MPG12_MFD_H__ */
