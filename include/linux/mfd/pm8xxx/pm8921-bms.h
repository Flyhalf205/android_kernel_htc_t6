/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PM8XXX_BMS_H
#define __PM8XXX_BMS_H

#include <linux/errno.h>

#define PM8921_BMS_DEV_NAME	"pm8921-bms"

#define FCC_CC_COLS		5
#define FCC_TEMP_COLS		8

#define PC_CC_ROWS             31
#define PC_CC_COLS             13

#define PC_TEMP_ROWS		31
#define PC_TEMP_COLS		8

#define MAX_SINGLE_LUT_COLS	20

#define OCV_UPDATE_STOP_BIT_CABLE_IN			(1)
#define OCV_UPDATE_STOP_BIT_BATT_LEVEL			(1<<1)
#define OCV_UPDATE_STOP_BIT_ATTR_FILE			(1<<2)
#define OCV_UPDATE_STOP_BIT_BOOT_UP			(1<<3)

struct single_row_lut {
	int x[MAX_SINGLE_LUT_COLS];
	int y[MAX_SINGLE_LUT_COLS];
	int cols;
};

/**
 * struct sf_lut -
 * @rows:	number of percent charge entries should be <= PC_CC_ROWS
 * @cols:	number of charge cycle entries should be <= PC_CC_COLS
 * @row_entries:	the charge cycles/temperature at which sf data
 *			is available in the table.
 *		The charge cycles must be in increasing order from 0 to rows.
 * @percent:	the percent charge at which sf data is available in the table
 *		The  percentcharge must be in decreasing order from 0 to cols.
 * @sf:		the scaling factor data
 */
struct sf_lut {
	int rows;
	int cols;
	int row_entries[PC_CC_COLS];
	int percent[PC_CC_ROWS];
	int sf[PC_CC_ROWS][PC_CC_COLS];
};

/**
 * struct pc_temp_ocv_lut -
 * @rows:	number of percent charge entries should be <= PC_TEMP_ROWS
 * @cols:	number of temperature entries should be <= PC_TEMP_COLS
 * @temp:	the temperatures at which ocv data is available in the table
 *		The temperatures must be in increasing order from 0 to rows.
 * @percent:	the percent charge at which ocv data is available in the table
 *		The  percentcharge must be in decreasing order from 0 to cols.
 * @ocv:	the open circuit voltage
 */
struct pc_temp_ocv_lut {
	int rows;
	int cols;
	int temp[PC_TEMP_COLS];
	int percent[PC_TEMP_ROWS];
	int ocv[PC_TEMP_ROWS][PC_TEMP_COLS];
};

/**
 * struct pm8921_bms_battery_data -
 * @fcc:		full charge capacity (mAmpHour)
 * @fcc_temp_lut:	table to get fcc at a given temp
 * @pc_temp_ocv_lut:	table to get percent charge given batt temp and cycles
 * @pc_sf_lut:		table to get percent charge scaling factor given cycles
 *			and percent charge
 * @rbatt_sf_lut:	table to get battery resistance scaling factor given
 *			temperature and percent charge
 * @default_rbatt_mohm:	the default value of battery resistance to use when
 *			readings from bms are not available.
 * @delta_rbatt_mohm:	the resistance to be added towards lower soc to
 *			compensate for battery capacitance.
 */
struct pm8921_bms_battery_data {
	unsigned int		fcc;
	struct single_row_lut	*fcc_temp_lut;
	struct single_row_lut	*fcc_sf_lut;
	struct pc_temp_ocv_lut	*pc_temp_ocv_lut;
	struct sf_lut		*pc_sf_lut;
	struct sf_lut		*rbatt_sf_lut;
	struct sf_lut		*rbatt_est_ocv_lut;
	int			default_rbatt_mohm;
	int			delta_rbatt_mohm;
	int			level_ocv_update_stop_begin; /* optional */
	int			level_ocv_update_stop_end; /* optional */
};

struct pm8921_bms_pj_data {
	struct single_row_lut	*pj_vth_discharge_lut;
	struct single_row_lut	*pj_dvi_discharge_lut;
	struct single_row_lut	*pj_vth_charge_lut;
	struct single_row_lut	*pj_dvi_charge_lut;
	struct single_row_lut	*pj_temp_lut;
};

struct pm8xxx_bms_core_data {
	unsigned int	batt_temp_channel;
	unsigned int	vbat_channel;
	unsigned int	ref625mv_channel;
	unsigned int	ref1p25v_channel;
	unsigned int	batt_id_channel;
};

enum battery_type {
	BATT_UNKNOWN = 0,
	BATT_PALLADIUM,
	BATT_DESAY,
};

/**
 * struct pm8921_bms_platform_data -
 * @batt_type:		allows to force chose battery calibration data
 * @r_sense:		sense resistor value in (mOhms)
 * @i_test:		current at which the unusable charger cutoff is to be
 *			calculated or the peak system current (mA)
 * @v_failure:		the voltage at which the battery is considered empty(mV)
 * @enable_fcc_learning:	if set the driver will learn full charge
 *				capacity of the battery upon end of charge
 * @ocv_update_stop_active_mask:	Limit OCV updated in certain situations.
 */
struct pm8921_bms_platform_data {
	struct pm8xxx_bms_core_data	bms_cdata;
	enum battery_type		battery_type;
	unsigned int			r_sense;
	unsigned int			i_test;
	unsigned int			v_failure;
	unsigned int			max_voltage_uv;
	unsigned int			rconn_mohm;
	int				store_batt_data_soc_thre;
	int				enable_fcc_learning;
	unsigned int			criteria_sw_est_ocv; /*estimate ocv if timer over criteria*/
	unsigned int			rconn_mohm_sw_est_ocv;
	void (*get_power_jacket_status) (int *full, int *status, int *exist);
	int				qb_mode_cc_criteria_uAh;
};

extern int batt_stored_magic_num;
extern int batt_stored_soc;
extern int batt_stored_ocv_uv;
extern int batt_stored_cc_uv;
extern unsigned long batt_stored_time_ms;

#if defined(CONFIG_PM8921_BMS) || defined(CONFIG_PM8921_BMS_MODULE)
extern struct pm8921_bms_battery_data  palladium_1500_data;
extern struct pm8921_bms_battery_data  desay_5200_data;
/**
 * pm8921_bms_get_vsense_avg - return the voltage across the sense
 *				resitor in microvolts
 * @result:	The pointer where the voltage will be updated. A -ve
 *		result means that the current is flowing in
 *		the battery - during battery charging
 *
 * RETURNS:	Error code if there was a problem reading vsense, Zero otherwise
 *		The result won't be updated in case of an error.
 *
 *
 */
int pm8921_bms_get_vsense_avg(int *result);

/**
 * pm8921_bms_get_battery_current - return the battery current based on vsense
 *				resitor in microamperes
 * @result:	The pointer where the voltage will be updated. A -ve
 *		result means that the current is flowing in
 *		the battery - during battery charging
 *
 * RETURNS:	Error code if there was a problem reading vsense, Zero otherwise
 *		The result won't be updated in case of an error.
 *
 */
int pm8921_bms_get_battery_current(int *result);

/**
 * pm8921_bms_get_percent_charge - returns the current battery charge in percent
 *
 */
int pm8921_bms_get_percent_charge(void);

/**
 * pm8921_calculate_pj_level - returns the current power jacket level
 *
 */
int pm8921_calculate_pj_level(int Vjk, int is_charging, int batt_temp);

/**
 * pm8921_bms_get_fcc - returns fcc in mAh of the battery depending on its age
 *			and temperature
 *
 */
int pm8921_bms_get_fcc(void);

/**
 * pm8921_bms_charging_began - function to notify the bms driver that charging
 *				has started. Used by the bms driver to keep
 *				track of chargecycles
 */
int pm8921_bms_charging_began(void);
/**
 * pm8921_bms_charging_end - function to notify the bms driver that charging
 *				has stopped. Used by the bms driver to keep
 *				track of chargecycles
 */
void pm8921_bms_charging_end(int is_battery_full);

void pm8921_bms_calibrate_hkadc(void);
int pm8921_bms_stop_ocv_updates(void);
int pm8921_bms_start_ocv_updates(void);
/**
 * pm8921_bms_get_simultaneous_battery_voltage_and_current
 *		- function to take simultaneous vbat and vsense readings
 *		  this puts the bms in override mode but keeps coulumb couting
 *		  on. Useful when ir compensation needs to be implemented
 */
int pm8921_bms_get_simultaneous_battery_voltage_and_current(int *ibat_ua,
								int *vbat_uv);
/**
 * pm8921_bms_get_rbatt - function to get the battery resistance in mOhm.
 */
int pm8921_bms_get_rbatt(void);
/**
 * pm8921_bms_invalidate_shutdown_soc - function to notify the bms driver that
 *					the battery was replaced between reboot
 *					and so it should not use the shutdown
 *					soc stored in a coincell backed register
 */
void pm8921_bms_invalidate_shutdown_soc(void);
/**
 * pm8921_bms_dump_all - function to dump irq, regs, params for debug
 */
int pm8921_bms_dump_all(void);
#ifdef CONFIG_HTC_BATT_8960
/********************************************/
/* htc_gauge/htc_charger abstract interface */
/********************************************/
/**
 * pm8921_bms_get_batt_current - get battery voltage in mA
 *
 */
int pm8921_bms_get_batt_current(int *result);

int pm8921_store_hw_reset_reason(int is_hw_reset);
/**
 * pm8921_bms_get_batt_soc - get battery voltage in percent
 *
 */
int pm8921_bms_get_batt_soc(int *result);
/**
 * pm8921_bms_get_batt_cc - get battery cc (uAh)
 *
 */
int pm8921_bms_get_batt_cc(int *result);
/**
 * pm8921_bms_store_battery_data_emmc - store battery data into emmc
 *
 */
int pm8921_bms_store_battery_data_emmc(void);
/**
 * pm8921_bms_store_battery_ui_soc - store ui soc into emmc
 *
 */
int pm8921_bms_store_battery_ui_soc(int soc_ui);
/**
 * pm8921_bms_get_battery_ui_soc - get ui soc from emmc
 *
 */
int pm8921_bms_get_battery_ui_soc(void);
/**
 * pm8921_bms_get_attr_text - function to get regs, params for debug
 */
int pm8921_bms_get_attr_text(char *buf, int size);
/**
 * pm8921_bms_enter_qb_mode - notify bms driver device enter qb mode
 */
int pm8921_bms_enter_qb_mode(void);
/**
 * pm8921_bms_exit_qb_mode - notify bms driver device exit qb mode
 */
int pm8921_bms_exit_qb_mode(void);
/**
 * pm8921_qb_mode_pwr_consumption_check - func to monitor qb mode cc accumulation
 */
int pm8921_qb_mode_pwr_consumption_check(unsigned long time_stamp);
#endif /* CONFIG_HTC_BATT_8960 */
#else
static inline int pm8921_bms_get_vsense_avg(int *result)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_battery_current(int *result)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_percent_charge(void)
{
	return -ENXIO;
}
static inline int pm8921_calculate_pj_level(int Vjk, int is_charging, int batt_temp)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_fcc(void)
{
	return -ENXIO;
}
static inline void pm8921_bms_charging_began(void)
{
}
static inline void pm8921_bms_charging_end(int is_battery_full)
{
}
static inline void pm8921_bms_calibrate_hkadc(void)
{
}
static inline void pm8921_bms_stop_ocv_updates(void)
{
}
static inline void pm8921_bms_start_ocv_updates(void)
{
}
static inline int pm8921_bms_get_simultaneous_battery_voltage_and_current(
						int *ibat_ua, int *vbat_uv)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_rbatt(void)
{
	return -EINVAL;
}
static inline void pm8921_bms_invalidate_shutdown_soc(void)
{
}
static inline int pm8921_bms_dump_all(void)
{
	return -ENXIO;
}
#ifdef CONFIG_HTC_BATT_8960
static inline int pm8921_bms_get_batt_current(int *result)
{
	return -ENXIO;
}

static inline int pm8921_store_hw_reset_reason(int is_hw_reset)
{
	return -ENXIO;
}

static inline int pm8921_bms_get_batt_soc(int *result)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_batt_cc(int *result)
{
	return -ENXIO;
}
static inline int pm8921_bms_store_battery_data_emmc(void)
{
	return -ENXIO;
}
static inline int pm8921_bms_store_battery_ui_soc(int soc_ui)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_battery_ui_soc(void)
{
	return -ENXIO;
}
static inline int pm8921_bms_get_attr_text(char *buf, int size)
{
	return 0;
}
static inline int pm8921_bms_enter_qb_mode(void)
{
	return 0;
}
static inline int pm8921_bms_exit_qb_mode(void)
{
	return 0;
}
static inline int pm8921_qb_mode_pwr_consumption_check(unsigned long time_stamp)
{
	return 0;
}
#endif /* CONFIG_HTC_BATT_8960 */
#endif

#endif
