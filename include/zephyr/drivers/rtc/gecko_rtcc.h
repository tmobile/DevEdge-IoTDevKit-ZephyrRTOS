/*
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * This driver is based upon Peter Bigot Consulting, LLC's
 * maxim_ds3231 driver but without an I2C interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Real-time clock control based on the gecko counter API.
 *
 * The Gecko Real Time Counter and Calendar (RTCC) is a 32-bit
 * counter ensuring timekeeping in low energy modes. The RTCC
 * also includes a calendar mode for easy time and date keeping.
 *
 * The core Zephyr API to this device is as a counter, with the
 * following limitations:
 * * many other counter APIs, such as start/stop/set_top_value are not
 *   supported as the clock is always running.
 * * the RTCC has three different Capture/Compare channels which can
 *   trigger wake-up, generate PRS signalling, or capture system events.
 *
 * Most applications for this device will need to use the extended
 * functionality exposed by this header to access the real-time-clock
 * features.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_RTC_GECKO_H_
#define ZEPHYR_INCLUDE_DRIVERS_RTC_GECKO_H_

#include <time.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/notify.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RTCC_ALARM_NUM        3

/* Constants corresponding to bits in the RTCC registers
 *
 * See the datasheet for interpretation of these bits.
 */
/** @brief ctrl bit for alarm 1 interrupt enable. */
#define GECKO_RTCC_ALARM1 RTCC_IEN_CC0
#define GECKO_RTCC_REG_CTRL_A1IE RTCC_IEN_CC0

/** @brief ctrl bit for alarm 2 interrupt enable. */
#define GECKO_RTCC_ALARM2 RTCC_IEN_CC1
#define GECKO_RTCC_REG_CTRL_A2IE RTCC_IEN_CC1

/** @brief ctrl bit for alarm 3 interrupt enable. */
#define GECKO_RTCC_ALARM3 RTCC_IEN_CC2
#define GECKO_RTCC_REG_CTRL_A3IE RTCC_IEN_CC2

/** @brief interrupt flags bit indicating alarm1 has triggered.
 *
 * If an alarm callback handler is registered this bit is
 * cleared prior to invoking the callback with the flags
 * indicating which alarms are ready.
 */
#define GECKO_RTCC_REG_CTRL_A1F RTCC_IF_CC0

/** @brief interrupt flags bit indicating alarm2 has triggered.
 *
 * If an alarm callback handler is registered this bit is
 * cleared prior to invoking the callback with the flags
 * indicating which alarms are ready.
 */
#define GECKO_RTCC_REG_CTRL_A2F RTCC_IF_CC1

/** @brief interrupt flags bit indicating alarm3 has triggered.
 *
 * If an alarm callback handler is registered this bit is
 * cleared prior to invoking the callback with the flags
 * indicating which alarms are ready.
 */
#define GECKO_RTCC_REG_CTRL_A3F RTCC_IF_CC2

/** @brief Control alarm behavior on match in seconds field.
 *
 * If clear the alarm fires only when the RTC seconds matches the
 * alarm seconds.
 *
 * If set the alarm seconds field is ignored and an alarm will be
 * triggered every second.  The bits for IGNMN, IGNHR, and IGNDA must
 * all be set.
 *
 * This bit must be clear for the second alarm instance.
 *
 * Bit maps to A1M1 and is used in
 * gecko_rtcc_alarm_configuration::alarm_flags.
 */
#define GECKO_RTCC_ALARM_FLAGS_IGNSE RTCC_IEN_CNTTICK

/** @brief Control alarm behavior on match in minutes field.
 *
 * If clear the alarm fires only when the RTC minutes matches the
 * alarm minutes.  The bit for IGNSE must be clear.
 *
 * If set the alarm minutes field is ignored and alarms will be
 * triggered based on IGNSE. The bits for IGNHR and IGNDA must both be
 * set.
 *
 * Bit maps to A1M2 or A2M2 and is used in
 * gecko_rtcc_alarm_configuration::alarm_flags.
 */
#define GECKO_RTCC_ALARM_FLAGS_IGNMN RTCC_IEN_MINTICK

/** @brief Control alarm behavior on match in hours field.
 *
 * If clear the alarm fires only when the RTC hours matches the
 * alarm hours.  The bits for IGNMN and IGNSE must be clear.
 *
 * If set the alarm hours field is ignored and alarms will be
 * triggered based on IGNMN and IGNSE.  The bit for IGNDA must be set.
 *
 * Bit maps to A1M3 or A2M3 and is used in
 * gecko_rtcc_alarm_configuration::alarm_flags.
 */
#define GECKO_RTCC_ALARM_FLAGS_IGNHR RTCC_IEN_HOURTICK

/** @brief Control alarm behavior on match in day/date field.
 *
 * If clear the alarm fires only when the RTC day/date matches the
 * alarm day/date, mediated by GECKO_RTCC_ALARM_FLAGS_DAY.  The bits
 * for IGNHR, IGNMN, and IGNSE must be clear
 *
 * If set the alarm day/date field is ignored and an alarm will be
 * triggered based on IGNHR, IGNMN, and IGNSE.
 *
 * Bit maps to A1M4 or A2M4 and is used in
 * gecko_rtcc_alarm_configuration::alarm_flags.
 */
#define GECKO_RTCC_ALARM_FLAGS_IGNDA RTCC_IEN_DAYTICK

/** @brief Control match on day of week versus day of month
 *
 * Set the flag to match on day of week; clear it to match on day of
 * month.
 *
 * Bit maps to DY/DTn in corresponding
 * gecko_rtcc_alarm_configuration::alarm_flags.
 */
#define GECKO_RTCC_ALARM_FLAGS_DOW RTCC_IEN_DAYOWOF

/**
 * @brief Gecko RTCC Driver-Specific API
 * @defgroup rtc_interface Real Time Clock interfaces
 * @ingroup io_interfaces
 * @{
 */

/** @brief Signature for RTCC alarm callbacks.
 *
 * The alarm callback is invoked from the system work queue thread.
 * At the point the callback is invoked the corresponding alarm flags
 * will have been cleared from the device status register.  The
 * callback is permitted to invoke operations on the device.
 *
 * @param dev the device from which the callback originated
 * @param id the alarm id
 * @param syncclock the value from gecko_rtcc_read_syncclock() at the
 * time the alarm interrupt was processed.
 * @param user_data the corresponding parameter from
 * gecko_rtcc_alarm::user_data.
 */
typedef void (*gecko_rtcc_alarm_callback_handler_t)(const struct device *dev,
						      uint8_t id,
						      uint32_t syncclock,
						      void *user_data);

/** @brief Signature used to notify a user of the RTCC that an
 * asynchronous operation has completed.
 *
 * Functions compatible with this type are subject to all the
 * constraints of #sys_notify_generic_callback.
 *
 * @param dev the RTCC device pointer
 *
 * @param notify the notification structure provided in the call
 *
 * @param res the result of the operation.
 */
typedef void (*gecko_rtcc_notify_callback)(const struct device *dev,
					     struct sys_notify *notify,
					     int res);

/** @brief Information defining the alarm configuration.
 *
 * RTCC alarms can be set to fire at specific times or at the
 * rollover of minute, hour, day, or day of week.
 *
 * When an alarm is configured with a handler an interrupt will be
 * generated and the handler called from the system work queue.
 *
 * When an alarm is configured without a handler, or a persisted alarm
 * is present, alarms can be read using gecko_rtcc_check_alarms().
 */
struct gecko_rtcc_alarm {
	/** @brief Time specification for an RTC alarm.
	 *
	 * Though specified as a UNIX time, the alarm parameters are
	 * determined by converting to civil time and interpreting the
	 * component hours, minutes, seconds, day-of-week, and
	 * day-of-month fields, mediated by the corresponding #flags.
	 *
	 * The year and month are ignored, but be aware that gmtime()
	 * determines day-of-week based on calendar date.  Decoded
	 * alarm times will fall within 1978-01 since 1978-01-01
	 * (first of month) was a Sunday (first of week).
	 */
	time_t time;

	/** @brief Handler to be invoked when alarms are signalled.
	 *
	 * If this is null the alarm will not be triggered by the
	 * INTn/SQW GPIO.  This is a "persisted" alarm from its role
	 * in using the RTCC to trigger a wake from deep sleep.  The
	 * application should use gecko_rtcc_check_alarms() to
	 * determine whether such an alarm has been triggered.
	 *
	 * If this is not null the driver will monitor the ISW GPIO
	 * for alarm signals and will invoke the handler with a
	 * parameter carrying the value returned by
	 * gecko_rtcc_check_alarms().  The corresponding status flags
	 * will be cleared in the device before the handler is
	 * invoked.
	 *
	 * The handler will be invoked from the system work queue.
	 */
	gecko_rtcc_alarm_callback_handler_t handler;

	/** @brief User-provided pointer passed to alarm callback. */
	void *user_data;

	/** @brief Flags controlling configuration of the alarm alarm.
	 *
	 * See RTCC_IEN_CNTTICK and related constants.
	 *
	 * Note that as described the alarm mask fields require that
	 * if a unit is not ignored, higher-precision units must also
	 * not be ignored.  For example, if match on hours is enabled,
	 * match on minutes and seconds must also be enabled.  Failure
	 * to comply with this requirement will cause
	 * gecko_rtcc_set_alarm() to return an error, leaving the
	 * alarm configuration unchanged.
	 */
	uint32_t flags;
};

/** @brief Register the RTCC clock against system clocks.
 *
 * This captures the same instant in both the RTC time scale and a
 * stable system clock scale, allowing conversion between those
 * scales.
 */
struct gecko_rtcc_syncpoint {
	/** @brief Time from the RTCC.
	 *
	 * This maybe in UTC, TAI, or local offset depending on how
	 * the RTC is maintained.
	 */
	struct timespec rtcc;

	/** @brief Value of a local clock at the same instant as #rtcc.
	 *
	 * This is captured from a stable monotonic system clock
	 * running at between 1 kHz and 1 MHz, allowing for
	 * microsecond to millisecond accuracy in synchronization.
	 */
	uint32_t syncclock;
};

/** @brief Read the local synchronization clock.
 *
 * Synchronization aligns the RTCC real-time clock with a stable
 * monotonic local clock which should have a frequency between 1 kHz
 * and 1 MHz and be itself synchronized with the primary system time
 * clock.  The accuracy of the alignment and the maximum time between
 * synchronization updates is affected by the resolution of this
 * clock.
 *
 * On some systems the hardware clock from k_cycles_get_32() is
 * suitable, but on others that clock advances too quickly.  The
 * frequency of the target-specific clock is provided by
 * gecko_rtcc_syncclock_frequency().
 *
 * At this time the value is captured from `k_uptime_get_32()`; future
 * kernel extensions may make a higher-resolution clock available.
 *
 * @note This function is *isr-ok*.
 *
 * @param dev the RTCC device pointer
 *
 * @return the current value of the synchronization clock.
 */
static inline uint32_t gecko_rtcc_read_syncclock(const struct device *dev)
{
	return k_uptime_get_32();
}

/** @brief Get the frequency of the synchronization clock.
 *
 * Provides the frequency of the clock used in gecko_rtcc_read_syncclock().
 *
 * @param dev the RTCC device pointer
 *
 * @return the frequency of the selected synchronization clock.
 */
static inline uint32_t gecko_rtcc_syncclock_frequency(const struct device *dev)
{
	return 1000U;
}

/**
 * @brief Set and clear specific bits in the control register.
 *
 * @note This function assumes the device register cache is valid.  It
 * will not read the register value, and it will write to the device
 * only if the value changes as a result of applying the set and clear
 * changes.
 *
 * @note Unlike gecko_rtcc_stat_update() the return value from this
 * function indicates the register value after changes were made.
 * That return value is cached for use in subsequent operations.
 *
 * @note This function is *supervisor*.
 *
 * @return the non-negative updated value of the register.
 */
int gecko_rtcc_ctrl_update(const struct device *dev,
			     uint32_t set_bits,
			     uint32_t clear_bits);

/**
 * @brief Read the ctrl_stat register then set and clear bits in it.
 *
 * The content of the ctrl_stat register will be read, then the set
 * and clear bits applied and the result written back to the device
 * (regardless of whether there appears to be a change in value).
 *
 * OSF, A1F, and A2F will be written with 1s if the corresponding bits
 * do not appear in either @p set_bits or @p clear_bits.  This ensures
 * that if any flag becomes set between the read and the write that
 * indicator will not be cleared.
 *
 * @note Unlike gecko_rtcc_ctrl_update() the return value from this
 * function indicates the register value before any changes were made.
 *
 * @note This function is *supervisor*.
 *
 * @param dev the RTCC device pointer
 *
 * @param set_bits bits to be set when writing back.  Setting bits
 * other than @ ref GECKO_RTCC_REG_STAT_EN32kHz will have no effect.
 *
 * @param clear_bits bits to be cleared when writing back.  Include
 * the bits for the status flags you want to clear.
 *
 * @return the non-negative register value as originally read
 * (disregarding the effect of clears and sets).
 */
int gecko_rtcc_stat_update(const struct device *dev,
			     uint32_t set_bits,
			     uint32_t clear_bits);

/** @brief Read a RTCC alarm configuration.
 *
 * The alarm configuration data is read from the device and
 * reconstructed into the output parameter.
 *
 * @note This function is *supervisor*.
 *
 * @param dev the RTCC device pointer.
 *
 * @param id the alarm index, which must be 0 (for the 1 s resolution
 * alarm) or 1 (for the 1 min resolution alarm).
 *
 * @param cfg a pointer to a structure into which the configured alarm
 * data will be stored.
 *
 * @return a non-negative value indicating successful conversion, or
 * invalid parameter.
 */
int gecko_rtcc_get_alarm(const struct device *dev,
			   uint8_t id,
			   struct gecko_rtcc_alarm *cfg);

/** @brief Configure a RTCC alarm.
 *
 * The alarm configuration is validated and stored into the device.
 *
 * To cancel an alarm use counter_cancel_channel_alarm().
 *
 * @note This function is *supervisor*.
 *
 * @param dev the RTCC device pointer.
 *
 * @param id 0 Analog to counter index.  @c ALARM1 is 0, @c ALARM2 is 1
 * and @c ALARM2 is 2.
 *
 * @param cfg a pointer to the desired alarm configuration.  Both
 * alarms are configured; if only one is to change the application
 * must supply the existing configuration for the other.
 *
 * @return a non-negative value on success, or an invalid parameter.
 */
int gecko_rtcc_set_alarm(const struct device *dev,
			   uint8_t id,
			   const struct gecko_rtcc_alarm *cfg);

/** @brief Synchronize the RTCC against the local clock.
 *
 * The RTCC advances one tick per second with no access to sub-second
 * precision.  Synchronizing clocks at sub-second resolution requires
 * enabling a 1pps signal then capturing the system clocks in a GPIO
 * callback.  This function provides that operation.
 *
 * Synchronization is performed in asynchronously, and may take as
 * long as 1 s to complete; notification of completion is provided
 * through the @p notify parameter.
 *
 * Applications should use gecko_rtcc_get_syncpoint() to retrieve the
 * synchronization data collected by this operation.
 *
 * @note This function is *supervisor*.
 *
 * @param dev the RTCC device pointer.
 *
 * @param notify pointer to the object used to specify asynchronous
 * function behavior and store completion information.
 *
 * @retval non-negative on success
 * @retval -EBUSY if a synchronization or set is currently in progress
 * @retval -EINVAL if notify is not provided
 * @retval -ENOTSUP if the required interrupt is not configured
 */
int gecko_rtcc_synchronize(const struct device *dev,
			     struct sys_notify *notify);
#if !defined(CONFIG_TIME_GECKO_RTCC)
/** @brief Request to update the synchronization point.
 *
 * This is a variant of gecko_rtcc_synchronize() for use from user
 * threads.
 *
 * @param dev the RTCC device pointer.
 *
 * @param signal pointer to a valid and ready-to-be-signalled
 * k_poll_signal.  May be NULL to request a synchronization point be
 * collected without notifying when it has been updated.
 *
 * @retval non-negative on success
 * @retval -EBUSY if a synchronization or set is currently in progress
 * @retval -ENOTSUP if the required interrupt is not configured
 */
__syscall int gecko_rtcc_req_syncpoint(const struct device *dev,
					 struct k_poll_signal *signal);
#endif
/** @brief Retrieve the most recent synchronization point.
 *
 * This function returns the synchronization data last captured using
 * gecko_rtcc_synchronize().
 *
 * @param dev the RTCC device pointer.
 *
 * @param syncpoint where to store the synchronization data.
 *
 * @retval non-negative on success
 * @retval -ENOENT if no syncpoint has been captured
 */
__syscall int gecko_rtcc_get_syncpoint(const struct device *dev,
					 struct gecko_rtcc_syncpoint *syncpoint);

/** @brief Set the RTCC to a time consistent with the provided
 * synchronization.
 *
 * The RTCC advances one tick per second with no access to sub-second
 * precision, and setting the clock resets the internal countdown
 * chain.  This function implements the magic necessary to set the
 * clock while retaining as much sub-second accuracy as possible.  It
 * requires a synchronization point that pairs sub-second resolution
 * civil time with a local synchronization clock captured at the same
 * instant.  The set operation may take as long as 1 second to
 * complete; notification of completion is provided through the @p
 * notify parameter.
 *
 * @note This function is *supervisor*.
 *
 * @param dev the RTCC device pointer.
 *
 * @param syncpoint the structure providing the synchronization point.
 *
 * @param notify pointer to the object used to specify asynchronous
 * function behavior and store completion information.
 *
 * @retval non-negative on success
 * @retval -EINVAL if syncpoint or notify are null
 * @retval -ENOTSUP if the required interrupt signal is not configured
 * @retval -EBUSY if a synchronization or set is currently in progress
 */
int gecko_rtcc_set(const struct device *dev,
		     const struct gecko_rtcc_syncpoint *syncpoint,
		     struct sys_notify *notify);

/** @brief Check for and clear flags indicating that an alarm has
 * fired.
 *
 * Returns a mask indicating alarms that are marked as having fired,
 * and clears from stat the flags that it found set.  Alarms that have
 * been configured with a callback are not represented in the return
 * value.
 *
 * This API may be used when a persistent alarm has been programmed.
 *
 * @note This function is *supervisor*.
 *
 * @param dev the RTCC device pointer.
 *
 * @return a non-negative value that may have GECKO_RTCC_ALARM0 and/or
 * GECKO_RTCC_ALARM1 and/or GECKO_RTCC_ALARM2 set, or a negative error code.
 */
int gecko_rtcc_check_alarms(const struct device *dev);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

/* @todo this should be syscalls/drivers/rtc/gecko_rtcc.h */
#include <syscalls/gecko_rtcc.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_RTC_GECKO_H_ */
