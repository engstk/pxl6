/* SPDX-License-Identifier: GPL-2.0 */
/*
 * machine generated DO NOT MODIFY
 * source MW_regmap_ds_v0p65_121919.csv
 * 2020-03-19
 */

#ifndef MAX77759_REG_H_
#define MAX77759_REG_H_

#include "max77759_A1_regs.h"

/*
 * b/156527175: workaround for read only MAX77759_CHG_DETAILS_03
 * MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7 is used to detect exit from fshipmode.
 * The register (MAX77759_PMIC_TOPSYS_INT_MASK) is type S and the bit is reset
 * to 1 on power loss. The reports MAX77759_CHG_DETAILS_03 when the bit
 * is 1 and report 0 when the bit is set to 0.
 */
#define MAX77759_FSHIP_EXIT_DTLS	  MAX77759_PMIC_TOPSYS_INT_MASK
#define MAX77759_FSHIP_EXIT_DTLS_RD \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7
#define MAX77759_FSHIP_EXIT_DTLS_RD_SHIFT \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_SHIFT
#define MAX77759_FSHIP_EXIT_DTLS_RD_MASK \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_MASK
#define MAX77759_FSHIP_EXIT_DTLS_RD_CLEAR \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_CLEAR

/* */
#define MAX77759_CHG_CNFG_11_OTG_VBYP_5100MV	0x2

/* */
#define MAX77759_CHG_CNFG_05_OTG_ILIM_DISABLE	0x00
#define MAX77759_CHG_CNFG_05_OTG_ILIM_500MA	0x01
#define MAX77759_CHG_CNFG_05_OTG_ILIM_1500MA	0x0b

/* b/179816224 */
#define MAX77759_CHG_CNFG_12_WCIN_REG_4_5  (0x0 << MAX77759_CHG_CNFG_12_WCIN_REG_SHIFT)
#define MAX77759_CHG_CNFG_12_WCIN_REG_4_85 (0x3 << MAX77759_CHG_CNFG_12_WCIN_REG_SHIFT)

/* mux configuration in MAX77759_PMIC_CONTROL_FG */
#define THMIO_MUX_BATT_PACK	0
#define THMIO_MUX_USB_TEMP	1
#define THMIO_MUX_BATT_ID	2

/* ----------------------------------------------------------------------------
 * Mode Register
 */

enum max77759_charger_modes {
	MAX77759_CHGR_MODE_ALL_OFF = 0x00,
	MAX77759_CHGR_MODE_BUCK_ON = 0x04,
	MAX77759_CHGR_MODE_CHGR_BUCK_ON = 0x05,
	MAX77759_CHGR_MODE_BOOST_UNO_ON = 0x08,
	MAX77759_CHGR_MODE_BOOST_ON = 0x09,
	MAX77759_CHGR_MODE_OTG_BOOST_ON = 0x0a,
	MAX77759_CHGR_MODE_BUCK_BOOST_UNO_ON = 0x0c,
	MAX77759_CHGR_MODE_CHGR_BUCK_BOOST_UNO_ON = 0x0d,
	MAX77759_CHGR_MODE_OTG_BUCK_BOOST_ON = 0x0e,
	MAX77759_CHGR_MODE_CHGR_OTG_BUCK_BOOST_ON = 0x0f,
};


#endif /* MAX77759_REG_H_ */