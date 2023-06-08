/*
 * Copyright (c) 2023 T-Mobile
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_SX9328_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_SX9328_H_

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_attribute_sx9328 {
	/** Doze period */
	SENSOR_ATTR_SX9328_DOZE_PERIOD = SENSOR_ATTR_PRIV_START,
	/** Scan period */
	SENSOR_ATTR_SX9328_SCAN_PERIOD,
	/** Scan period multiplier for channels 2 & 3 */
	SENSOR_ATTR_SX9328_SCAN_PERIOD_23,
	/** Enabled phases */
	SENSOR_ATTR_SX9328_PHASES,
	/** Sampling frequency for phases 0 & 1 */
	SENSOR_ATTR_SX9328_PHASE_01_FREQ,
	/** Sampling frequency for phases 2 & 3 */
	SENSOR_ATTR_SX9328_PHASE_23_FREQ,
	/** Digital gain for phases 0 & 1 */
	SENSOR_ATTR_SX9328_PHASE_01_GAIN,
	/** Digital gain for phases 2 & 3 */
	SENSOR_ATTR_SX9328_PHASE_23_GAIN,
	/** Proximity threshold for phases 0 & 1 */
	SENSOR_ATTR_SX9328_PHASE_01_PRX_THD,
	/** Proximity threshold for phases 2 & 3 */
	SENSOR_ATTR_SX9328_PHASE_23_PRX_THD,
	/** High table threshold for phases 0 & 1 */
	SENSOR_ATTR_SX9328_PHASE_01_HITBL_THD,
	/** High table threshold for phases 2 & 3 */
	SENSOR_ATTR_SX9328_PHASE_23_HITBL_THD,
	/** Low table threshold for phases 0 & 1 */
	SENSOR_ATTR_SX9328_PHASE_01_LOTBL_THD,
	/** Low table threshold for phases 2 & 3 */
	SENSOR_ATTR_SX9328_PHASE_23_LOTBL_THD,
	/** Body threshold for phases 0 & 1 */
	SENSOR_ATTR_SX9328_PHASE_01_BDY_THD,
	/** Body threshold for phases 2 & 3 */
	SENSOR_ATTR_SX9328_PHASE_23_BDY_THD,
	/** Sets which generic proximity flags trigger the proximity channel */
	SENSOR_ATTR_SX9328_PRX_PHASES,
	/**
	 * Sets which phase table proximity flags trigger the table proximity
	 * channel
	 */
	SENSOR_ATTR_SX9328_TBL_PRX_PHASES,
	/**
	 * Sets which phase body proximity flags trigger the body proximity
	 * channel
	 */
	SENSOR_ATTR_SX9328_BDY_PRX_PHASES,
};

enum sensor_channel_sx9328 {
	/** Table proximity value */
	SENSOR_CHAN_SX9328_TBL_PRX = SENSOR_CHAN_PRIV_START,
	/** Body proximity value */
	SENSOR_CHAN_SX9328_BDY_PRX,
	/** Phase 0 table proximity value */
	SENSOR_CHAN_SX9328_TBL_PRX_PH0,
	/** Phase 1 table proximity value */
	SENSOR_CHAN_SX9328_TBL_PRX_PH1,
	/** Phase 2 table proximity value */
	SENSOR_CHAN_SX9328_TBL_PRX_PH2,
	/** Phase 3 table proximity value */
	SENSOR_CHAN_SX9328_TBL_PRX_PH3,

	/** Phase 0 body proximity value */
	SENSOR_CHAN_SX9328_BDY_PRX_PH0,
	/** Phase 1 body proximity value */
	SENSOR_CHAN_SX9328_BDY_PRX_PH1,
	/** Phase 2 body proximity value */
	SENSOR_CHAN_SX9328_BDY_PRX_PH2,
	/** Phase 3 body proximity value */
	SENSOR_CHAN_SX9328_BDY_PRX_PH3,

};

enum sensor_trigger_type_sx9328 {
	/**
	 * Trigger fires when a near/far event is detected for body or table
	 * sensing.
	 */
	SENSOR_TRIG_SX9328_NEAR_FAR_TBL_BDY = SENSOR_TRIG_PRIV_START,
};

#define SX9328_FREQ_250_KHZ		0b00000
#define SX9328_FREQ_200_KHZ		0b00001
#define SX9328_FREQ_166_67_KHZ	0b00010
#define SX9328_FREQ_142_86_KHZ	0b00011
#define SX9328_FREQ_125_KHZ		0b00100
#define SX9328_FREQ_111_11_KHZ	0b00101
#define SX9328_FREQ_100_KHZ		0b00110
#define SX9328_FREQ_90_91_KHZ	0b00111
#define SX9328_FREQ_83_33_KHZ	0b01000
#define SX9328_FREQ_76_92_KHZ	0b01001
#define SX9328_FREQ_71_43_KHZ	0b01010
#define SX9328_FREQ_66_67_KHZ	0b01011
#define SX9328_FREQ_62_50_KHZ	0b01100
#define SX9328_FREQ_58_82_KHZ	0b01101
#define SX9328_FREQ_55_56_KHZ	0b01110
#define SX9328_FREQ_52_63_KHZ	0b01111
#define SX9328_FREQ_50_KHZ		0b10000
#define SX9328_FREQ_45_45_KHZ	0b10001
#define SX9328_FREQ_41_67_KHZ	0b10010
#define SX9328_FREQ_38_46_KHZ	0b10011
#define SX9328_FREQ_35_71_KHZ	0b10100
#define SX9328_FREQ_31_25_KHZ	0b10101
#define SX9328_FREQ_27_78_KHZ	0b10110
#define SX9328_FREQ_25_KHZ		0b10111
#define SX9328_FREQ_20_83_KHZ	0b11000
#define SX9328_FREQ_17_86_KHZ	0b11001
#define SX9328_FREQ_13_89_KHZ	0b11010
#define SX9328_FREQ_11_36_KHZ	0b11011
#define SX9328_FREQ_8_33_KHZ	0b11100
#define SX9328_FREQ_6_58_KHZ	0b11101
#define SX9328_FREQ_5_43_KHZ	0b11110
#define SX9328_FREQ_4_63_KHZ	0b11111

#define SX9328_DOZE_OFF			0b00
#define SX9328_DOZE_X4			0b01
#define SX9328_DOZE_X8			0b10
#define SX9328_DOZE_X16			0b11

#define SX9328_SCANPERIOD23_X1	0b00
#define SX9328_SCANPERIOD23_X2	0b01
#define SX9328_SCANPERIOD23_X8	0b10
#define SX9328_SCANPERIOD23_X16	0b11

#define SX9328_PHASE0_EN		0x01
#define SX9328_PHASE1_EN		0x02
#define SX9328_PHASE2_EN		0x04
#define SX9328_PHASE3_EN		0x08

#define SX9328_DIGITAL_GAIN_X1	0b001
#define SX9328_DIGITAL_GAIN_X2	0b010
#define SX9328_DIGITAL_GAIN_X4	0b011
#define SX9328_DIGITAL_GAIN_X8	0b100

#define SX9328_THD_OFF	    0b0000
#define SX9328_THD_2048	    0b0001
#define SX9328_THD_4096	    0b0010
#define SX9328_THD_6144	    0b0011
#define SX9328_THD_8192	    0b0100
#define SX9328_THD_10240	0b0101
#define SX9328_THD_12288	0b0110
#define SX9328_THD_14336	0b0111
#define SX9328_THD_16384	0b1000
#define SX9328_THD_18432	0b1001
#define SX9328_THD_20480	0b1010
#define SX9328_THD_22528	0b1011
#define SX9328_THD_24576	0b1100
#define SX9328_THD_26624	0b1101
#define SX9328_THD_28672	0b1110
#define SX9328_THD_30720	0b1111

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_SX9328_H_ */
