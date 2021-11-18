/*
 * Definitions for nl80211 vendor command/event access to host driver
 *
 * Copyright (C) 2020, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Dual:>>
 *
 */

#ifndef _brcm_nl80211_h_
#define _brcm_nl80211_h_

//#ifdef OEM_ANDROID Need proper #ifdef in the referencing code as well
#define OUI_BRCM  0x001018
#define OUI_GOOGLE  0x001A11

enum wl_vendor_subcmd {
	BRCM_VENDOR_SCMD_UNSPEC			= 0,
	BRCM_VENDOR_SCMD_PRIV_STR		= 1,
	BRCM_VENDOR_SCMD_BCM_STR		= 2,
	BRCM_VENDOR_SCMD_BCM_PSK		= 3,
	BRCM_VENDOR_SCMD_SET_PMK		= 4,
	BRCM_VENDOR_SCMD_GET_FEATURES		= 5,
	BRCM_VENDOR_SCMD_SET_MAC		= 6,
	BRCM_VENDOR_SCMD_SET_CONNECT_PARAMS	= 7,
	BRCM_VENDOR_SCMD_SET_START_AP_PARAMS	= 8,
	BRCM_VENDOR_SCMD_ACS			= 9,
	BRCM_VENDOR_SCMD_MAX			= 10
};

/* Added for HOSTAPD required ACS action */
#ifdef WL_SUPPORT_AUTO_CHANNEL
#define APCS_MAX_RETRY        10
#define APCS_DEFAULT_2G_CH    1
#define APCS_DEFAULT_5G_CH    149

enum wl_vendor_attr_acs_offload {
	BRCM_VENDOR_ATTR_ACS_CHANNEL_INVALID = 0,
	BRCM_VENDOR_ATTR_ACS_PRIMARY_FREQ,
	BRCM_VENDOR_ATTR_ACS_SECONDARY_FREQ,
	BRCM_VENDOR_ATTR_ACS_VHT_SEG0_CENTER_CHANNEL,
	BRCM_VENDOR_ATTR_ACS_VHT_SEG1_CENTER_CHANNEL,

	BRCM_VENDOR_ATTR_ACS_HW_MODE,
	BRCM_VENDOR_ATTR_ACS_HT_ENABLED,
	BRCM_VENDOR_ATTR_ACS_HT40_ENABLED,
	BRCM_VENDOR_ATTR_ACS_VHT_ENABLED,
	BRCM_VENDOR_ATTR_ACS_CHWIDTH,
	BRCM_VENDOR_ATTR_ACS_CH_LIST,
	BRCM_VENDOR_ATTR_ACS_FREQ_LIST,

	BRCM_VENDOR_ATTR_ACS_LAST
};

/* defined for hw_mode in hostapd.conf */
enum hostapd_hw_mode {
	HOSTAPD_MODE_IEEE80211B,
	HOSTAPD_MODE_IEEE80211G,
	HOSTAPD_MODE_IEEE80211A,
	HOSTAPD_MODE_IEEE80211AD,
	HOSTAPD_MODE_IEEE80211ANY,
	NUM_HOSTAPD_MODES
};

typedef struct acs_selected_channels {
	u32 pri_freq; /* save slelcted primary frequency */
	u32 sec_freq; /* save slelcted secondary frequency */
	u8 vht_seg0_center_ch;
	u8 vht_seg1_center_ch;
	u16 ch_width;
	enum hostapd_hw_mode hw_mode;
} acs_selected_channels_t;

typedef struct drv_acs_params {
	enum hostapd_hw_mode hw_mode;
	int band;

	int ht_enabled;
	int ht40_enabled;
	int vht_enabled;
	int he_enabled;

	u16 ch_width;

	unsigned int ch_list_len;
	const u8 *ch_list;

	const int *freq_list;
} drv_acs_params_t;

typedef struct acs_delay_work {
	struct delayed_work acs_delay_work;
	int init_flag;

	struct net_device *ndev;
	chanspec_t ch_chosen;
	drv_acs_params_t parameter;
} acs_delay_work_t;
#endif /* WL_SUPPORT_AUTO_CHANNEL */

struct bcm_nlmsg_hdr {
	uint cmd;	/* common ioctl definition */
	int len;	/* expected return buffer length */
	uint offset;	/* user buffer offset */
	uint set;	/* get or set request optional */
	uint magic;	/* magic number for verification */
};

enum bcmnl_attrs {
	BCM_NLATTR_UNSPEC,

	BCM_NLATTR_LEN,
	BCM_NLATTR_DATA,

	__BCM_NLATTR_AFTER_LAST,
	BCM_NLATTR_MAX = __BCM_NLATTR_AFTER_LAST - 1
};

struct nl_prv_data {
	int err;			/* return result */
	void *data;			/* ioctl return buffer pointer */
	uint len;			/* ioctl return buffer length */
	struct bcm_nlmsg_hdr *nlioc;	/* bcm_nlmsg_hdr header pointer */
};
//#endif /* OEM_ANDROID */

/* Keep common BCM netlink macros here */
#define BCM_NL_USER	31
#define BCM_NL_OXYGEN	30
#define BCM_NL_TS	29
/* ====== !! ADD NEW NL socket related defines here !! ====== */

#endif /* _brcm_nl80211_h_ */
