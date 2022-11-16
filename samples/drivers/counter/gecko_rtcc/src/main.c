/*
 * Copyright (c) 2019-2020 Peter Bigot Consulting, LLC
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * This sample is based upon modifying Peter Bigot Consulting, LLC's
 * maxim_ds3231 sample to work with the gecko_rtcc's driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/rtc/gecko_rtcc.h>

/* Format times as: YYYY-MM-DD HH:MM:SS DOW DOY */
static const char *format_time(time_t time,
			       long nsec)
{
	static char buf[64];
	char *bp = buf;
	char *const bpe = bp + sizeof(buf);
	struct tm tv;
	struct tm *tp = gmtime_r(&time, &tv);

	bp += strftime(bp, bpe - bp, "%Y-%m-%d %H:%M:%S", tp);
	if (nsec >= 0) {
		bp += snprintf(bp, bpe - bp, ".%09lu", nsec);
	}
	bp += strftime(bp, bpe - bp, " %a %j", tp);
	return buf;
}

static void sec_counter_callback(const struct device *dev,
				 uint8_t id,
				 uint32_t ticks,
				 void *ud)
{
	printk("Counter callback at %u ms, id %d, ticks %u, ud %p\n",
	       k_uptime_get_32(), id, ticks, ud);
}

static void sec_alarm_handler(const struct device *dev,
			      uint8_t id,
			      uint32_t syncclock,
			      void *ud)
{
	uint32_t now = gecko_rtcc_read_syncclock(dev);
	struct counter_alarm_cfg alarm = {
		.callback = sec_counter_callback,
		.ticks = 10,
		.user_data = ud,
	};

	printk("setting channel alarm\n");
	int rc = counter_set_channel_alarm(dev, id, &alarm);

	printk("Sec signaled at %u ms, param %p, delay %u; set %d\n",
	       k_uptime_get_32(), ud, now - syncclock, rc);
}


/** Calculate the normalized result of a - b.
 *
 * For both inputs and outputs tv_nsec must be in the range [0,
 * NSEC_PER_SEC).  tv_sec may be negative, zero, or positive.
 */
void timespec_subtract(struct timespec *amb,
		       const struct timespec *a,
		       const struct timespec *b)
{
	if (a->tv_nsec >= b->tv_nsec) {
		amb->tv_nsec = a->tv_nsec - b->tv_nsec;
		amb->tv_sec = a->tv_sec - b->tv_sec;
	} else {
		amb->tv_nsec = NSEC_PER_SEC + a->tv_nsec - b->tv_nsec;
		amb->tv_sec = a->tv_sec - b->tv_sec - 1;
	}
}

/** Calculate the normalized result of a + b.
 *
 * For both inputs and outputs tv_nsec must be in the range [0,
 * NSEC_PER_SEC).  tv_sec may be negative, zero, or positive.
 */
void timespec_add(struct timespec *apb,
		  const struct timespec *a,
		  const struct timespec *b)
{
	apb->tv_nsec = a->tv_nsec + b->tv_nsec;
	apb->tv_sec = a->tv_sec + b->tv_sec;
	if (apb->tv_nsec >= NSEC_PER_SEC) {
		apb->tv_sec += 1;
		apb->tv_nsec -= NSEC_PER_SEC;
	}
}

static void min_alarm_handler(const struct device *dev,
			      uint8_t id,
			      uint32_t syncclock,
			      void *ud)
{
	uint32_t time_get = 0;
	struct gecko_rtcc_syncpoint sp = { 0 };

	(void)counter_get_value(dev, &time_get);

	uint32_t uptime = k_uptime_get_32();
	uint8_t us = uptime % 1000U;

	uptime /= 1000U;
	uint8_t se = uptime % 60U;

	uptime /= 60U;
	uint8_t mn = uptime % 60U;

	uptime /= 60U;
	uint8_t hr = uptime;

	(void)gecko_rtcc_get_syncpoint(dev, &sp);

	uint32_t offset_syncclock = syncclock - sp.syncclock;
	uint32_t offset_s = time_get - (uint32_t)sp.rtc.tv_sec;
	uint32_t syncclock_Hz = gecko_rtcc_syncclock_frequency(dev);
	struct timespec adj;

	adj.tv_sec = offset_syncclock / syncclock_Hz;
	adj.tv_nsec = (offset_syncclock % syncclock_Hz)
		* (uint64_t)NSEC_PER_SEC / syncclock_Hz;

	int32_t err_ppm = (int32_t)(offset_syncclock
				- offset_s * syncclock_Hz)
			* (int64_t)1000000
			/ (int32_t)syncclock_Hz / (int32_t)offset_s;
	struct timespec *ts = &sp.rtc;

	ts->tv_sec += adj.tv_sec;
	ts->tv_nsec += adj.tv_nsec;
	if (ts->tv_nsec >= NSEC_PER_SEC) {
		ts->tv_sec += 1;
		ts->tv_nsec -= NSEC_PER_SEC;
	}

	printk("%s: adj %d.%09lu, uptime %u:%02u:%02u.%03u, clk err %d ppm\n",
	       format_time(time_get, -1),
	       (uint32_t)(ts->tv_sec - time_get), ts->tv_nsec,
	       hr, mn, se, us, err_ppm);
}

struct gecko_rtcc_alarm alarm[RTCC_ALARM_NUM];

static void show_counter(const struct device *gecko)
{
	uint32_t now = 0;

	printk("Counter at %p\n", gecko);
	printk("\tMax top value: %u (%08x)\n",
	       counter_get_max_top_value(gecko),
	       counter_get_max_top_value(gecko));
	printk("\t%u channels\n", counter_get_num_of_channels(gecko));
	printk("\t%u Hz\n", counter_get_frequency(gecko));

	printk("Top counter value: %u (%08x)\n",
	       counter_get_top_value(gecko),
	       counter_get_top_value(gecko));

	(void)counter_get_value(gecko, &now);

	printk("Now %u: %s\n", now, format_time(now, -1));
}

/* Take the currently stored RTC time and round it up to the next
 * hour.  Program the RTC as though this time had occurred at the
 * moment the application booted.
 *
 * Subsequent reads of the RTC time adjusted based on a syncpoint
 * should match the uptime relative to the programmed hour.
 */
static void set_aligned_clock(const struct device *gecko)
{
	if (!IS_ENABLED(CONFIG_APP_SET_ALIGNED_CLOCK)) {
		return;
	}

	uint32_t syncclock_Hz = gecko_rtcc_syncclock_frequency(gecko);
	uint32_t syncclock = gecko_rtcc_read_syncclock(gecko);
	uint32_t now = 0;
	int rc = counter_get_value(gecko, &now);
	uint32_t align_hour = now + 3600 - (now % 3600);

	struct gecko_rtcc_syncpoint sp = {
		.rtc = {
			.tv_sec = align_hour,
			.tv_nsec = (uint64_t)NSEC_PER_SEC * syncclock / syncclock_Hz,
		},
		.syncclock = syncclock,
	};

	struct k_poll_signal ss;
	struct sys_notify notify;
	struct k_poll_event sevt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
							    K_POLL_MODE_NOTIFY_ONLY,
							    &ss);

	k_poll_signal_init(&ss);
	sys_notify_init_signal(&notify, &ss);

	uint32_t t0 = k_uptime_get_32();

	rc = gecko_rtcc_set(gecko, &sp, &notify);

	printk("Set %s at %u ms past: %d\n", format_time(sp.rtc.tv_sec, sp.rtc.tv_nsec),
	       syncclock, rc);

	/* Wait for the set to complete */
	rc = k_poll(&sevt, 1, K_FOREVER);

	uint32_t t1 = k_uptime_get_32();

	/* Delay so log messages from sync can complete */
	k_sleep(K_MSEC(100));
	printk("Synchronize final: %d %d in %u ms\n", rc, ss.result, t1 - t0);

	rc = gecko_rtcc_get_syncpoint(gecko, &sp);
	printk("wrote sync %d: %u %u at %u\n", rc,
	       (uint32_t)sp.rtc.tv_sec, (uint32_t)sp.rtc.tv_nsec,
	       sp.syncclock);
}

void main(void)
{
	const struct device *gecko;
	const char *const dev_id = DT_LABEL(DT_INST(0, silabs_gecko_rtcc));

	gecko = device_get_binding(dev_id);
	if (!gecko) {
		printk("No device %s available\n", dev_id);
		return;
	}

	uint32_t syncclock_Hz = gecko_rtcc_syncclock_frequency(gecko);

	printk("Gecko on %s syncclock %u Hz\n", CONFIG_BOARD, syncclock_Hz);

	int rc = gecko_rtcc_stat_update(gecko, 0, RTCC_IF_OSCFAIL);

	if (rc >= 0) {
		printk("Gecko has%s experienced an oscillator fault\n",
		       (rc & RTCC_IF_OSCFAIL) ? "" : " not");
	} else {
		printk("Gecko stat fetch failed: %d\n", rc);
		return;
	}

	/* Show the Gecko counter properties */
	show_counter(gecko);

	/* Show the GECKO ctrl and ctrl_stat register values */
	printk("GECKO ctrl %03x ; ctrl_stat %03x\n",
	       gecko_rtcc_ctrl_update(gecko, 0, 0),
	       gecko_rtcc_stat_update(gecko, 0, 0));

	/* Test gecko_rtcc_set, if enabled */
	set_aligned_clock(gecko);

	struct k_poll_signal ss;
	struct sys_notify notify;
	struct gecko_rtcc_syncpoint sp = { 0 };
	struct k_poll_event sevt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
							    K_POLL_MODE_NOTIFY_ONLY,
							    &ss);

	k_poll_signal_init(&ss);
	sys_notify_init_signal(&notify, &ss);

	uint32_t t0 = k_uptime_get_32();

	rc = gecko_rtcc_synchronize(gecko, &notify);
	printk("Synchronize init: %d\n", rc);

	rc = k_poll(&sevt, 1, K_FOREVER);

	uint32_t t1 = k_uptime_get_32();

	k_sleep(K_MSEC(100));   /* wait for log messages */

	printk("Synchronize complete in %u ms: %d %d\n", t1 - t0, rc, ss.result);

	rc = gecko_rtcc_get_syncpoint(gecko, &sp);
	printk("Read sync %d: %u %u at %u\n", rc,
	       (uint32_t)sp.rtc.tv_sec, (uint32_t)sp.rtc.tv_nsec,
	       sp.syncclock);

	uint8_t id;

	for (id = 0; id < counter_get_num_of_channels(gecko); ++id) {
		rc = gecko_rtcc_get_alarm(gecko, 0, &alarm[id]);
		printk("Alarm[%d] flags %x at %u: %d\n", id, alarm[id].flags,
				(uint32_t)alarm[id].time, rc);
	}

	/* One-shot callback in 5 s.  The handler will
	 * then use the base device counter API to schedule a second
	 * alarm 10 s later.
	 */
	alarm[0].time = sp.rtc.tv_sec + 5;
	alarm[0].flags = 0;
	alarm[0].handler = sec_alarm_handler;
	alarm[0].user_data = &alarm[0];

	printk("Min base time: %s\n", format_time(alarm[0].time, -1));

	/* Repeating callback at rollover to a new minute. */
	alarm[1].time = alarm[0].time;
	alarm[1].flags = 0
			  | RTCC_IF_DAYTICK
			  | RTCC_IF_HOURTICK
			  | RTCC_IF_MINTICK
			  | RTCC_IF_CNTTICK;
	alarm[1].handler = min_alarm_handler;

	/* Repeating callback at rollover to a new minute. */
	alarm[2].time = alarm[0].time;
	alarm[2].flags = RTCC_IF_MINTICK;
	alarm[2].handler = min_alarm_handler;

	for (id = 0; id < counter_get_num_of_channels(gecko); ++id) {
		rc = gecko_rtcc_set_alarm(gecko, 0, &alarm[id]);
		printk("Set alarm[%d] flags %x at %u ~ %s: %d\n", id, alarm[id].flags,
				(uint32_t)alarm[id].time, format_time(alarm[id].time, -1), rc);
	}

	printk("%u ms in: get alarms: ", k_uptime_get_32());
	for (id = 0; id < counter_get_num_of_channels(gecko); ++id) {
		printk("%d ", gecko_rtcc_get_alarm(gecko, id, &alarm[id]));
	}
	printk("\n");

	if (rc >= 0) {
		for (id = 0; id < counter_get_num_of_channels(gecko); ++id) {
			printk("Alarm[%d] flags %x at %u ~ %s\n", id, alarm[id].flags,
					(uint32_t)alarm[id].time, format_time(alarm[id].time, -1));
		}
	}

	k_sleep(K_FOREVER);
}
