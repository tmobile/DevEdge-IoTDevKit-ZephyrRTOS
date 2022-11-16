/*
 * Copyright (c) 2019, Piotr Mienkowski
 * Copyright (c) 2022 T-Mobile USA, Inc.
 *
 * This driver is based upon merging Peter Bigot Consulting, LLC's
 * maxim_ds3231 driver with Piotr's counter_gecko_rtcc driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_gecko_rtcc

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc/gecko_rtcc.h>
#include <syscalls/gecko_rtcc.h>
#include <zephyr/posix/time.h>
#include <soc.h>
#include <em_cmu.h>
#include <em_rmu.h>
#include <em_rtcc.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/drivers/counter.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(time_gecko, CONFIG_COUNTER_LOG_LEVEL);

#define RTCC_MAX_VALUE       (_RTCC_CNT_MASK)

enum {
	SYNCSM_IDLE,
	SYNCSM_PREP_READ,
	SYNCSM_FINISH_READ,
	SYNCSM_PREP_WRITE,
	SYNCSM_FINISH_WRITE,
};

struct register_map {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dow;
	uint8_t dom;
	uint8_t month;
	uint8_t year;

	struct alarm {
		uint8_t sec;
		uint8_t min;
		uint8_t hour;
		uint8_t day;
		uint8_t month;
		uint32_t time;    /*!< RTCC_CCx_TIME - Capture/Compare Time Register */
		uint32_t date;    /*!< RTCC_CCx_DATE - Capture/Compare Date Register */
	} __packed alarms[RTCC_ALARM_NUM];

	uint32_t time;    /*!< RTCC_TIME - Time of Day Register */
	uint32_t date;    /*!< RTCC_DATE - Date Register */
	uint32_t ctrl;
	uint32_t ctrl_stat;
};

struct counter_gecko_config {
	struct counter_config_info generic;
	void (*irq_config)(void);
	uint32_t prescaler;
};

struct counter_gecko_alarm_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct counter_gecko_data {
	const struct device *gecko;

	struct counter_gecko_alarm_data alarm[RTCC_ALARM_NUM];
	struct register_map registers;

	struct k_sem lock;

	/* Timer structure used for synchronization */
	struct k_timer sync_timer;

	/* Work structures for the various cases of ISW interrupt. */
	struct k_work alarm_work;
	struct k_work sqw_work;
	struct k_work sync_work;

	/* Forward ISW interrupt to proper worker. */
	counter_alarm_callback_t isw_callback;

	/* syncclock captured in the last ISW interrupt handler */
	uint32_t isw_syncclock;

	struct gecko_rtcc_syncpoint syncpoint;
	struct gecko_rtcc_syncpoint new_sp;

	time_t rtc_registers;
	time_t rtc_base;
	uint32_t syncclock_base;

	/* Pointer to the structure used to notify when a synchronize
	 * or set operation completes.  Null when nobody's waiting for
	 * such an operation, or when doing a no-notify synchronize
	 * through the signal API.
	 */
	union {
		void *ptr;
		struct sys_notify *notify;
		struct k_poll_signal *signal;
	} sync;

	/* Handlers and state when using the counter alarm API. */
	gecko_rtcc_alarm_callback_handler_t alarm_handler[RTCC_ALARM_NUM];
	void *alarm_user_data[RTCC_ALARM_NUM];
	uint8_t alarm_flags[RTCC_ALARM_NUM];
	uint32_t counter_ticks[RTCC_ALARM_NUM];

	/* Flags recording requests for ISW monitoring. */
	uint8_t isw_mon_req;
#define ISW_MON_REQ_Alarm 0x01
#define ISW_MON_REQ_Sync 0x02

	/* Status of synchronization operations. */
	uint8_t sync_state;
	bool sync_signal;
};

#define DEV_NAME(dev) ((dev)->name)
#define DEV_CFG(dev) \
	((const struct counter_gecko_config * const)(dev)->config)
#define DEV_DATA(dev) \
	((struct counter_gecko_data *const)(dev)->data)

#ifdef CONFIG_SOC_GECKO_HAS_ERRATA_RTCC_E201
#define ERRATA_RTCC_E201_MESSAGE \
	"Errata RTCC_E201: In case RTCC prescaler != 1 the module does not " \
	"reset the counter value on CCV1 compare."
#endif

/*
 * Set and clear specific bits in the control register.
 *
 * This function assumes the device register cache is valid and will
 * update the device only if the value changes as a result of applying
 * the set and clear changes.
 *
 * Caches and returns the value with the changes applied.
 */
static int sc_ctrl(const struct device *dev,
		   uint32_t set,
		   uint32_t clear)
{
	struct counter_gecko_data *data = dev->data;
	struct register_map *rp = &data->registers;
	uint8_t ctrl = (rp->ctrl & ~clear) | set;
	int rc = ctrl;

	if (rp->ctrl != ctrl) {
		RTCC_IntEnable(set);
		RTCC_IntDisable(clear);
		if (rc >= 0) {
			rp->ctrl = ctrl;
			rc = ctrl;
		}
	}
	return rc;
}

int gecko_rtcc_ctrl_update(const struct device *dev,
			     uint32_t set_bits,
			     uint32_t clear_bits)
{
	struct counter_gecko_data *data = dev->data;

	k_sem_take(&data->lock, K_FOREVER);

	int rc = sc_ctrl(dev, set_bits, clear_bits);

	k_sem_give(&data->lock);

	return rc;
}

/*
 * Read the RTCC Interrupt Flags register then set and clear bits in it.
 *
 * Returns the value as originally read (disregarding the effect of
 * clears and sets).
 */
static inline int rsc_stat(const struct device *dev,
			   uint32_t set,
			   uint32_t clear)
{
	struct counter_gecko_data *data = dev->data;
	struct register_map *rp = &data->registers;
	int rc = 0;

	rp->ctrl_stat = RTCC_IntGet();
	if (rc >= 0) {
		uint32_t stat = rp->ctrl_stat & ~clear;

		if (rp->ctrl_stat != stat) {
			RTCC_IntSet(set);
			RTCC_IntClear(clear);
		}
		if (rc >= 0) {
			rc = rp->ctrl_stat;
		}
	}
	return rc;
}

int gecko_rtcc_stat_update(const struct device *dev,
			     uint32_t set_bits,
			     uint32_t clear_bits)
{
	struct counter_gecko_data *data = dev->data;

	k_sem_take(&data->lock, K_FOREVER);

	int rv = rsc_stat(dev, set_bits, clear_bits);

	k_sem_give(&data->lock);

	return rv;
}

/*
 * Look for current users of the interrupt/square-wave signal and
 * enable monitoring iff at least one consumer is active.
 */
static void validate_isw_monitoring(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	const struct register_map *rp = &data->registers;
	uint8_t isw_mon_req = 0;

	if (rp->ctrl & (RTCC_IEN_CC0 | RTCC_IEN_CC1 | RTCC_IEN_CC2)) {
		isw_mon_req |= ISW_MON_REQ_Alarm;
	}
	if (data->sync_state != SYNCSM_IDLE) {
		isw_mon_req |= ISW_MON_REQ_Sync;
	}
	LOG_DBG("ISW : %d ?= %d", isw_mon_req, data->isw_mon_req);
	if ((data->isw_callback != NULL)
	    && (isw_mon_req != data->isw_mon_req)) {
		int rc = 0;

		/* Disable before reconfigure */
		irq_disable(DT_INST_IRQN(0));

		if ((rc >= 0)
		    && ((isw_mon_req & ISW_MON_REQ_Sync)
			!= (data->isw_mon_req & ISW_MON_REQ_Sync))) {
			if (isw_mon_req & ISW_MON_REQ_Sync) {
				rc = sc_ctrl(dev, RTCC_IEN_CNTTICK, 0);
			} else {
				rc = sc_ctrl(dev, 0, RTCC_IEN_CNTTICK);
			}
		}

		data->isw_mon_req = isw_mon_req;

		/* Enable if any requests active */
		if ((rc >= 0) && (isw_mon_req != 0)) {
			irq_enable(DT_INST_IRQN(0));
		}

		LOG_INF("ISW reconfigure to %x: %d", isw_mon_req, rc);
	}
}

static const uint8_t *decode_time(struct tm *tp,
			       const uint8_t *rp,
			       bool with_sec)
{
	uint8_t reg;

	if (with_sec) {
		uint8_t reg = *rp++;

		tp->tm_sec = bcd2bin(reg);
	}

	reg = *rp++;
	tp->tm_min = bcd2bin(reg);

	reg = *rp++;
	tp->tm_hour = bcd2bin(reg);

	return rp;
}

static uint32_t decode_alarm(const uint8_t *ap,
			 bool with_sec,
			 time_t *tp)
{
	struct tm decode_tm = {
		/* tm_year zero is 1900 with underflows a 32-bit counter
		 * representation.  Use 1978-01, the first January after the
		 * POSIX epoch where the first day of the month is the first
		 * day of the week.
		 */
		.tm_year = 78,
	};
	const uint8_t *dp = decode_time(&decode_tm, ap, with_sec);
	uint32_t flags = RTCC_IntGetEnabled();

	decode_tm.tm_mday = bcd2bin(*dp++);
	decode_tm.tm_mon  = bcd2bin(*dp++);

	/* Convert to the reduced representation. */
	*tp = timeutil_timegm(&decode_tm);
	return flags;
}

static int encode_alarm(uint8_t *ap,
			bool with_sec,
			time_t time,
			uint8_t flags)
{
	struct tm encode_tm;
	uint8_t val;

	(void)gmtime_r(&time, &encode_tm);

	if (with_sec) {
		val = bin2bcd(encode_tm.tm_sec);
		*ap++ = val;
	}

	val = bin2bcd(encode_tm.tm_min);
	*ap++ = val;

	val = bin2bcd(encode_tm.tm_hour);
	*ap++ = val;

	val = bin2bcd(encode_tm.tm_mday);
	*ap++ = val;

	return 0;
}

static uint32_t decode_rtc(struct counter_gecko_data *data)
{
	struct tm tm = { 0 };
	const struct register_map *rp = &data->registers;

	data->registers.sec = (data->registers.time &
				(_RTCC_TIME_SECU_MASK |
				 _RTCC_TIME_SECT_MASK))
				>> _RTCC_TIME_SECU_SHIFT;
	data->registers.min = (data->registers.time &
				(_RTCC_TIME_MINU_MASK |
				 _RTCC_TIME_MINT_MASK))
				>> _RTCC_TIME_MINU_SHIFT;
	data->registers.hour = (data->registers.time &
				(_RTCC_TIME_HOURU_MASK |
				 _RTCC_TIME_HOURT_MASK))
				>> _RTCC_TIME_HOURU_SHIFT;
	data->registers.dom = (data->registers.date &
				(_RTCC_DATE_DAYOMU_MASK |
				 _RTCC_DATE_DAYOMT_MASK))
				>> _RTCC_DATE_DAYOMU_SHIFT;
	data->registers.month = (data->registers.date &
				(_RTCC_DATE_MONTHU_MASK |
				 _RTCC_DATE_MONTHT_MASK))
				>> _RTCC_DATE_MONTHU_SHIFT;
	data->registers.year = (data->registers.date &
				(_RTCC_DATE_YEARU_MASK |
				 _RTCC_DATE_YEART_MASK))
				>> _RTCC_DATE_YEARU_SHIFT;
	data->registers.dow = (data->registers.date &
				(_RTCC_DATE_DAYOW_MASK))
				>> _RTCC_DATE_DAYOW_SHIFT;

	decode_time(&tm, &rp->sec, true);
	tm.tm_wday = (rp->dow);
	tm.tm_mday = bcd2bin(rp->dom);
	tm.tm_mon = bcd2bin(rp->month);
	tm.tm_year = bcd2bin(rp->year);
	tm.tm_year += RTCC->RET[0].REG * 100;

	data->rtc_registers = timeutil_timegm(&tm);
	return data->rtc_registers;
}

static int update_registers(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	uint32_t syncclock;
	uint32_t tmp;

	data->syncclock_base = gecko_rtcc_read_syncclock(dev);

	data->registers.time = RTCC_TimeGet();
	do {
		tmp = data->registers.time;
		data->registers.date = RTCC_DateGet();
		data->registers.time = RTCC_TimeGet();
	} while (data->registers.time != tmp);

	syncclock = gecko_rtcc_read_syncclock(dev);

	data->rtc_base = decode_rtc(data);

	return 0;
}

int gecko_rtcc_get_alarm(const struct device *dev,
			   uint8_t id,
			   struct gecko_rtcc_alarm *cp)
{
	struct counter_gecko_data *data = dev->data;
	const struct counter_gecko_config *cfg = dev->config;
	int rv = 0;
	uint8_t addr;

	if (id < cfg->generic.channels) {
		addr = offsetof(struct register_map, alarms[id]);
	} else {
		rv = -EINVAL;
		goto out;
	}

	k_sem_take(&data->lock, K_FOREVER);

	/* Update alarm structure */
	uint8_t *rbp = &data->registers.sec + addr;

	data->registers.alarms[id].time = RTCC_ChannelTimeGet(id);
	data->registers.alarms[id].date = RTCC_ChannelDateGet(id);

	if (rv < 0) {
		LOG_DBG("get_config at %02x failed: %d\n", addr, rv);
		goto out_locked;
	}

	data->registers.alarms[id].sec = (data->registers.alarms[id].time &
				(_RTCC_CC_TIME_SECU_MASK |
				 _RTCC_CC_TIME_SECT_MASK))
				>> _RTCC_CC_TIME_SECU_SHIFT;
	data->registers.alarms[id].min = (data->registers.alarms[id].time &
				(_RTCC_CC_TIME_MINU_MASK |
				 _RTCC_CC_TIME_MINT_MASK))
				>> _RTCC_CC_TIME_MINU_SHIFT;
	data->registers.alarms[id].hour = (data->registers.alarms[id].time &
				(_RTCC_CC_TIME_HOURU_MASK |
				 _RTCC_CC_TIME_HOURT_MASK))
				>> _RTCC_CC_TIME_HOURU_SHIFT;
	data->registers.alarms[id].day = (data->registers.alarms[id].date &
				(_RTCC_CC_DATE_DAYU_MASK |
				 _RTCC_CC_DATE_DAYT_MASK))
				>> _RTCC_CC_DATE_DAYU_SHIFT;
	data->registers.alarms[id].month = (data->registers.alarms[id].date &
				(_RTCC_CC_DATE_MONTHU_MASK |
				 _RTCC_CC_DATE_MONTHT_MASK))
				>> _RTCC_CC_DATE_MONTHU_SHIFT;

	*cp = (struct gecko_rtcc_alarm){ 0 };
	cp->flags = decode_alarm(rbp, true, &cp->time);
	cp->handler = data->alarm_handler[id];
	cp->user_data = data->alarm_user_data[id];

out_locked:
	k_sem_give(&data->lock);

out:
	return rv;
}

static int cancel_alarm(const struct device *dev,
			uint8_t id)
{
	struct counter_gecko_data *data = dev->data;

	data->alarm_handler[id] = NULL;
	data->alarm_user_data[id] = NULL;
	data->alarm[id].callback = NULL;
	data->alarm[id].user_data = NULL;

	RTCC_ChannelCCVSet(id, 0);

	LOG_DBG("cancel alarm: channel %u", id);

	return sc_ctrl(dev, 0, RTCC_IF_CC0 << id);
}

static int counter_gecko_cancel_alarm(const struct device *dev,
				       uint8_t id)
{
	struct counter_gecko_data *data = dev->data;
	const struct counter_gecko_config *cfg = dev->config;
	int rv = 0;

	if (id >= cfg->generic.channels) {
		rv = -EINVAL;
		goto out;
	}

	k_sem_take(&data->lock, K_FOREVER);

	rv = cancel_alarm(dev, id);

	k_sem_give(&data->lock);

out:
	/* Throw away information counter API disallows */
	if (rv >= 0) {
		rv = 0;
	}

	return rv;
}

static int set_alarm(const struct device *dev,
		     uint8_t id,
		     const struct gecko_rtcc_alarm *cp)
{
	struct counter_gecko_data *data = dev->data;
	const struct counter_gecko_config *cfg = dev->config;
	uint8_t addr;
	uint8_t len;

	if (id == 0) {
		addr = offsetof(struct register_map, alarms[0]);
		len = sizeof(data->registers.alarms[0]);
	} else if (id == 1) {
		addr = offsetof(struct register_map, alarms[1]);
		len = sizeof(data->registers.alarms[1]);
	} else if (id < cfg->generic.channels) {
		addr = offsetof(struct register_map, alarms[2]);
		len = sizeof(data->registers.alarms[2]);
	} else {
		return -EINVAL;
	}

	uint8_t buf[5] = { addr };
	int rc = encode_alarm(buf + 1, true, cp->time, cp->flags);

	if (rc < 0) {
		return rc;
	}

	/* @todo resolve race condition: a previously stored alarm may
	 * trigger between clear of AxF and the write of the new alarm
	 * control.
	 */
	rc = rsc_stat(dev, 0U, (RTCC_IFC_CC0 << id));
	if (rc >= 0) {
		uint32_t set_date = 0, set_time = 0;

		set_time |= (buf[0] << _RTCC_CC_TIME_SECU_SHIFT) &
				(_RTCC_CC_TIME_SECU_MASK |
				 _RTCC_CC_TIME_SECT_MASK);
		set_time |= (buf[1] << _RTCC_CC_TIME_MINU_SHIFT) &
				(_RTCC_CC_TIME_MINU_MASK |
				 _RTCC_CC_TIME_MINT_MASK);
		set_time |= (buf[2] << _RTCC_CC_TIME_HOURU_SHIFT) &
				(_RTCC_CC_TIME_HOURU_MASK |
				 _RTCC_CC_TIME_HOURT_MASK);
		set_date |= (buf[3] << _RTCC_CC_DATE_DAYU_SHIFT) &
				(_RTCC_CC_DATE_DAYU_MASK |
				 _RTCC_CC_DATE_DAYT_MASK);
		set_date |= (buf[4] << _RTCC_CC_DATE_MONTHU_SHIFT) &
				(_RTCC_CC_DATE_MONTHU_MASK |
				 _RTCC_CC_DATE_MONTHT_MASK);

		RTCC_ChannelDateSet(id, set_date);
		RTCC_ChannelTimeSet(id, set_time);
	}
	if ((rc >= 0)
	    && (cp->handler != NULL)) {
		rc = sc_ctrl(dev, RTCC_IEN_CC0 << id, 0);
	}
	if (rc >= 0) {
		memmove(&data->registers.sec + addr, buf + 1, len);
		data->alarm_handler[id] = cp->handler;
		data->alarm_user_data[id] = cp->user_data;
		data->alarm_flags[id] = cp->flags;
		validate_isw_monitoring(dev);
	}

	return rc;
}

int gecko_rtcc_set_alarm(const struct device *dev,
			   uint8_t id,
			   const struct gecko_rtcc_alarm *cp)
{
	struct counter_gecko_data *data = dev->data;

	k_sem_take(&data->lock, K_FOREVER);

	int rc = set_alarm(dev, id, cp);

	k_sem_give(&data->lock);

	return rc;
}

int geck_rtcc_check_alarms(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	const struct register_map *rp = &data->registers;
	uint8_t mask = (RTCC_IF_CC0 | RTCC_IF_CC1 | RTCC_IF_CC2);

	k_sem_take(&data->lock, K_FOREVER);

	/* Fetch and clear only the alarm flags that are not
	 * interrupt-enabled.
	 */
	int rv = rsc_stat(dev, 0U, (rp->ctrl & mask) ^ mask);

	if (rv >= 0) {
		rv &= mask;
	}

	k_sem_give(&data->lock);

	return rv;
}

static int check_handled_alarms(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	const struct register_map *rp = &data->registers;
	uint8_t mask = (RTCC_IF_CC0 | RTCC_IF_CC1 | RTCC_IF_CC2);

	/* Fetch and clear only the alarm flags that are
	 * interrupt-enabled.  Leave any flags that are not enabled;
	 * it may be an alarm that triggered a wakeup.
	 */
	mask &= rp->ctrl;

	int rv = rsc_stat(dev, 0U, mask);

	if (rv > 0) {
		rv &= mask;
	}

	return rv;
}

static void counter_alarm_forwarder(const struct device *dev,
				    uint8_t id,
				    uint32_t syncclock,
				    void *ud)
{
	/* Dummy handler marking a counter callback. */
}

static void alarm_worker(struct k_work *work)
{
	struct counter_gecko_data *data = CONTAINER_OF(work,
						struct counter_gecko_data,
						alarm_work);
	const struct device *gecko = data->gecko;
	const struct counter_gecko_config *cfg = gecko->config;

	k_sem_take(&data->lock, K_FOREVER);

	int af = check_handled_alarms(gecko);

	while (af > 0) {
		uint8_t id;

		for (id = 0; id < cfg->generic.channels; ++id) {
			if ((af & (RTCC_IF_CC0 << id)) == 0) {
				continue;
			}


			gecko_rtcc_alarm_callback_handler_t handler
				= data->alarm_handler[id];
			void *ud = data->alarm_user_data[id];

			if (data->alarm_flags[id]) {
				int rc = cancel_alarm(gecko, id);

				LOG_DBG("autodisable %d: %d", id, rc);
				validate_isw_monitoring(gecko);
			}

			if (handler == counter_alarm_forwarder) {
				counter_alarm_callback_t cb = data->alarm_handler[id];
				uint32_t ticks = data->counter_ticks[id];

				data->alarm_handler[id] = NULL;
				data->counter_ticks[id] = 0;

				if (cb) {
					k_sem_give(&data->lock);

					cb(gecko, id, ticks, ud);

					k_sem_take(&data->lock, K_FOREVER);
				}

			} else if (handler != NULL) {
				k_sem_give(&data->lock);

				handler(gecko, id, data->isw_syncclock, ud);

				k_sem_take(&data->lock, K_FOREVER);
			}
		}
		af = check_handled_alarms(gecko);
	}

	k_sem_give(&data->lock);

	if (af < 0) {
		LOG_ERR("failed to read alarm flags");
		return;
	}

	LOG_DBG("ALARM %02x at %u latency %u", af, data->isw_syncclock,
			gecko_rtcc_read_syncclock(gecko) - data->isw_syncclock);
}

static void sqw_worker(struct k_work *work)
{
	struct counter_gecko_data *data = CONTAINER_OF(work, struct counter_gecko_data, sqw_work);
	uint32_t syncclock = gecko_rtcc_read_syncclock(data->gecko);

	/* This is a placeholder for potential application-controlled
	 * use of the square wave functionality.
	 */
	LOG_DBG("SQW %u latency %u", data->isw_syncclock,
		syncclock - data->isw_syncclock);
}

static int read_time(const struct device *dev,
		     time_t *time_read)
{
	struct counter_gecko_data *data = dev->data;
	uint32_t tmp;

	data->registers.time = RTCC_TimeGet();
	do {
		tmp = data->registers.time;
		data->registers.date = RTCC_DateGet();
		data->registers.time = RTCC_TimeGet();
	} while (data->registers.time != tmp);

	*time_read = decode_rtc(data);

	return 0;
}

static int counter_gecko_get_value(const struct device *dev, uint32_t *ticks)
{
	struct counter_gecko_data *data = dev->data;
	time_t time_get = 0;

	k_sem_take(&data->lock, K_FOREVER);

	int rc = read_time(dev, &time_get);

	k_sem_give(&data->lock);

	if (rc >= 0) {
		*ticks = time_get;
	}

	return rc;
}

static void sync_finish(const struct device *dev,
			int rc)
{
	struct counter_gecko_data *data = dev->data;
	struct sys_notify *notify = NULL;
	struct k_poll_signal *signal_rcv = NULL;

	if (data->sync_signal) {
		signal_rcv = data->sync.signal;
	} else {
		notify = data->sync.notify;
	}
	data->sync.ptr = NULL;
	data->sync_signal = false;
	data->sync_state = SYNCSM_IDLE;
	(void)validate_isw_monitoring(dev);

	LOG_DBG("sync complete, notify %d to %p or %p\n", rc, notify, signal_rcv);
	k_sem_give(&data->lock);

	if (notify != NULL) {
		gecko_rtcc_notify_callback cb =
			(gecko_rtcc_notify_callback)sys_notify_finalize(notify, rc);

		if (cb) {
			cb(dev, notify, rc);
		}
	} else if (signal_rcv != NULL) {
		k_poll_signal_raise(signal_rcv, rc);
	}
}

static void sync_prep_read(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	int rc = sc_ctrl(dev, RTCC_IEN_CNTTICK, 0U);

	if (rc < 0) {
		sync_finish(dev, rc);
		return;
	}
	data->sync_state = SYNCSM_FINISH_READ;
	validate_isw_monitoring(dev);
}

static void sync_finish_read(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	time_t time_rcv = 0;

	(void)read_time(dev, &time_rcv);
	data->syncpoint.rtcc.tv_sec = time_rcv;
	data->syncpoint.rtcc.tv_nsec = 0;
	data->syncpoint.syncclock = data->isw_syncclock;
	sync_finish(dev, 0);
}

static void sync_timer_handler(struct k_timer *tmr)
{
	struct counter_gecko_data *data = CONTAINER_OF(tmr,
						struct counter_gecko_data,
						sync_timer);

	LOG_INF("sync_timer fired");
	k_work_submit(&data->sync_work);
}

static void sync_prep_write(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	uint32_t syncclock = gecko_rtcc_read_syncclock(dev);
	uint32_t offset = syncclock - data->new_sp.syncclock;
	uint32_t syncclock_Hz = gecko_rtcc_syncclock_frequency(dev);
	uint32_t offset_s = offset / syncclock_Hz;
	uint32_t offset_ms = (offset % syncclock_Hz) * 1000U / syncclock_Hz;
	time_t when = data->new_sp.rtcc.tv_sec;

	when += offset_s;
	offset_ms += data->new_sp.rtcc.tv_nsec / NSEC_PER_USEC / USEC_PER_MSEC;
	if (offset_ms >= MSEC_PER_SEC) {
		offset_ms -= MSEC_PER_SEC;
	} else {
		when += 1;
	}

	uint32_t rem_ms = MSEC_PER_SEC - offset_ms;

	if (rem_ms < 5) {
		when += 1;
		rem_ms += MSEC_PER_SEC;
	}
	data->new_sp.rtcc.tv_sec = when;
	data->new_sp.rtcc.tv_nsec = 0;

	data->sync_state = SYNCSM_FINISH_WRITE;
	k_timer_start(&data->sync_timer, K_MSEC(rem_ms), K_NO_WAIT);
	LOG_INF("sync %u in %u ms after %u", (uint32_t)when, rem_ms, syncclock);
}

static void sync_finish_write(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	time_t when = data->new_sp.rtcc.tv_sec;
	struct tm tm_gm;

	data->registers.time = data->registers.date = 0;

	(void)gmtime_r(&when, &tm_gm);
	data->registers.sec = bin2bcd(tm_gm.tm_sec);
	data->registers.time |= data->registers.sec << _RTCC_TIME_SECU_SHIFT;

	data->registers.min = bin2bcd(tm_gm.tm_min);
	data->registers.time |= data->registers.min << _RTCC_TIME_MINU_SHIFT;

	data->registers.hour = bin2bcd(tm_gm.tm_hour);
	data->registers.time |= data->registers.hour << _RTCC_TIME_HOURU_SHIFT;

	data->registers.dow = tm_gm.tm_wday;
	data->registers.date |= data->registers.dow << _RTCC_DATE_DAYOW_SHIFT;

	data->registers.dom = bin2bcd(tm_gm.tm_mday);
	data->registers.date |= data->registers.dom << _RTCC_DATE_DAYOMU_SHIFT;

	tm_gm.tm_mon += 1;
	data->registers.month = bin2bcd(tm_gm.tm_mon);
	data->registers.date |= data->registers.month << _RTCC_DATE_MONTHU_SHIFT;

	if (tm_gm.tm_year >= 100) {
		RTCC->RET[0].REG = tm_gm.tm_year / 100;
		tm_gm.tm_year %= 100;
	}

	data->registers.year = bin2bcd(tm_gm.tm_year);
	data->registers.date |= data->registers.year << _RTCC_DATE_YEARU_SHIFT;

	uint32_t syncclock = gecko_rtcc_read_syncclock(dev);

	RTCC_TimeSet(data->registers.time);
	RTCC_DateSet(data->registers.date);

	data->syncpoint.rtcc.tv_sec = when;
	data->syncpoint.rtcc.tv_nsec = 0;
	data->syncpoint.syncclock = syncclock;
	LOG_INF("sync %u at %u", (uint32_t)when, syncclock);

	sync_finish(dev, 0);
}

static void sync_worker(struct k_work *work)
{
	struct counter_gecko_data *data = CONTAINER_OF(work, struct counter_gecko_data, sync_work);
	uint32_t syncclock = gecko_rtcc_read_syncclock(data->gecko);
	bool unlock = true;

	k_sem_take(&data->lock, K_FOREVER);

	LOG_DBG("SYNC.%u %u latency %u", data->sync_state, data->isw_syncclock,
		syncclock - data->isw_syncclock);
	switch (data->sync_state) {
	default:
	case SYNCSM_IDLE:
		break;
	case SYNCSM_PREP_READ:
		sync_prep_read(data->gecko);
		break;
	case SYNCSM_FINISH_READ:
		sync_finish_read(data->gecko);
		break;
	case SYNCSM_PREP_WRITE:
		sync_prep_write(data->gecko);
		break;
	case SYNCSM_FINISH_WRITE:
		sync_finish_write(data->gecko);
		unlock = false;
		break;
	}

	if (unlock) {
		k_sem_give(&data->lock);
	}
}

static void isw_rtcc_callback(const struct device *dev,
				uint8_t id,
				uint32_t syncclock,
				void *ud)
{
	struct counter_gecko_data *data = dev->data;

	data->isw_syncclock = gecko_rtcc_read_syncclock(data->gecko);
	if (data->registers.ctrl & RTCC_IEN_CNTTICK) {
		k_work_submit(&data->alarm_work);
	} else if (data->sync_state != SYNCSM_IDLE) {
		k_work_submit(&data->sync_work);
	} else {
		k_work_submit(&data->sqw_work);
	}
}

int z_impl_gecko_rtcc_get_syncpoint(const struct device *dev,
				      struct gecko_rtcc_syncpoint *syncpoint)
{
	struct counter_gecko_data *data = dev->data;
	int rv = 0;

	k_sem_take(&data->lock, K_FOREVER);

	if (data->syncpoint.rtcc.tv_sec == 0) {
		rv = -ENOENT;
	} else {
		__ASSERT_NO_MSG(syncpoint != NULL);
		*syncpoint = data->syncpoint;
	}

	k_sem_give(&data->lock);

	return rv;
}

int gecko_rtcc_synchronize(const struct device *dev,
			     struct sys_notify *notify)
{
	struct counter_gecko_data *data = dev->data;
	int rv = 0;

	if (notify == NULL) {
		rv = -EINVAL;
		goto out;
	}

	if (data->isw_callback == NULL) {
		rv = -ENOTSUP;
		goto out;
	}

	k_sem_take(&data->lock, K_FOREVER);

	if (data->sync_state != SYNCSM_IDLE) {
		rv = -EBUSY;
		goto out_locked;
	}

	data->sync_signal = false;
	data->sync.notify = notify;
	data->sync_state = SYNCSM_PREP_READ;

out_locked:
	k_sem_give(&data->lock);

	if (rv >= 0) {
		k_work_submit(&data->sync_work);
	}

out:
	return rv;
}

int z_impl_gecko_rtcc_req_syncpoint(const struct device *dev,
				      struct k_poll_signal *sig)
{
	struct counter_gecko_data *data = dev->data;
	int rv = 0;

	if (data->isw_callback == NULL) {
		rv = -ENOTSUP;
		goto out;
	}

	k_sem_take(&data->lock, K_FOREVER);

	if (data->sync_state != SYNCSM_IDLE) {
		rv = -EBUSY;
		goto out_locked;
	}

	data->sync_signal = true;
	data->sync.signal = sig;
	data->sync_state = SYNCSM_PREP_READ;

out_locked:
	k_sem_give(&data->lock);

	if (rv >= 0) {
		k_work_submit(&data->sync_work);
	}

out:
	return rv;
}

int gecko_rtcc_set(const struct device *dev,
		     const struct gecko_rtcc_syncpoint *syncpoint,
		     struct sys_notify *notify)
{
	struct counter_gecko_data *data = dev->data;
	int rv = 0;

	if ((syncpoint == NULL)
	    || (notify == NULL)) {
		rv = -EINVAL;
		goto out;
	}
	if (data->isw_callback == NULL) {
		rv = -ENOTSUP;
		goto out;
	}

	k_sem_take(&data->lock, K_FOREVER);

	if (data->sync_state != SYNCSM_IDLE) {
		rv = -EBUSY;
		goto out_locked;
	}

	data->new_sp = *syncpoint;
	data->sync_signal = false;
	data->sync.notify = notify;
	data->sync_state = SYNCSM_PREP_WRITE;

out_locked:
	k_sem_give(&data->lock);

	if (rv >= 0) {
		k_work_submit(&data->sync_work);
	}

out:
	return rv;
}

static int update_posix(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	struct timespec tp;

	clock_gettime(CLOCK_REALTIME, &tp);
	tp.tv_sec = data->rtc_base;

	return clock_settime(CLOCK_REALTIME, &tp);
}

/* Pin bootloader definitions. */
#define LB_CLW0           (*((volatile uint32_t *)(LOCKBITS_BASE)+122))
#define LB_CLW0_PINBOOTLOADER    (1 << 1)

static int counter_gecko_init(const struct device *dev)
{
	struct counter_gecko_data *data = dev->data;
	const struct counter_gecko_config *const dev_cfg = DEV_CFG(dev);
	int rc;

	/* Initialize and take the lock */
	k_sem_init(&data->lock, 0, 1);

	data->gecko = dev;

	RTCC_Init_TypeDef rtcc_config = {
		true,                 /* Start counting when initialization is done */
		false,                /* Disable RTC during debug halt. */
		false,                /* Don't wrap prescaler on CCV0 */
		true,                 /* Counter wrap on CCV1 */
		rtccCntPresc_32768,   /* Divide clock by 32768 = 1 sec ticks. */
		rtccCntTickPresc,     /* Count according to prescaler value */
#if defined(_RTCC_CTRL_BUMODETSEN_MASK)
		/* Don't store RTCC counter value in
		 * RTCC_CCV2 upon backup mode entry.
		 */
		false,
#endif
#if defined(_RTCC_CTRL_OSCFDETEN_MASK)
		false,                /* Don't enable LFXO fail detection */
#endif
#if defined(_RTCC_CTRL_CNTMODE_MASK)
		rtccCntModeCalendar,  /* Use RTCC in calendar mode */
#endif
#if defined(_RTCC_CTRL_LYEARCORRDIS_MASK)
		false                 /* No leap year correction. */
#endif
	};

	RTCC_CCChConf_TypeDef rtcc_channel_config = {
		rtccCapComChModeCompare,    /* Use compare mode */
		rtccCompMatchOutActionPulse,/* Don't care */
		rtccPRSCh0,                 /* PRS is not used */
		rtccInEdgeNone,             /* Capture input is not used */
		rtccCompBaseCnt,            /* Compare with base CNT register */
#if defined(_RTCC_CC_CTRL_COMPMASK_MASK)
		0,                          /* Compare mask */
#endif
#if defined(_RTCC_CC_CTRL_DAYCC_MASK)
		rtccDayCompareModeMonth,    /* Don't care */
#endif
	};

#if defined(cmuClock_CORELE)
	/* Ensure LE modules are clocked. */
	CMU_ClockEnable(cmuClock_CORELE, true);
#endif

#if defined(CMU_LFECLKEN0_RTCC)
	/* Enable LFECLK in CMU (will also enable oscillator if not enabled). */
	CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);
#elif defined(_SILICON_LABS_32B_SERIES_2)
	CMU_ClockSelectSet(cmuClock_RTCC, cmuSelect_LFXO);
#else
	/* Turn on the clock for the RTCC */
	CMU_ClockEnable(cmuClock_HFLE, true);
	CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);
#endif

	/* Turn off the bootloader enable bit to prevent RTCC changes */
	LB_CLW0 &= !LB_CLW0_PINBOOTLOADER;

	/* Turn on reset limited mode to preserve RTCC counts */
	RMU_ResetControl(rmuResetWdog, rmuResetModeLimited);
	RMU_ResetControl(rmuResetCoreLockup, rmuResetModeLimited);
	RMU_ResetControl(rmuResetSys, rmuResetModeLimited);
	RMU_ResetControl(rmuResetPin, rmuResetModeLimited);

	/* Enable RTCC module clock */
	CMU_ClockEnable(cmuClock_RTCC, true);

	/* Initialize RTCC */
	RTCC_Init(&rtcc_config);

	/* Set up compare channels */
	RTCC_ChannelInit(0, &rtcc_channel_config);
	RTCC_ChannelInit(1, &rtcc_channel_config);
	RTCC_ChannelInit(2, &rtcc_channel_config);

	/* Disable module's internal interrupt sources */
	RTCC_IntDisable(_RTCC_IF_MASK);
	RTCC_IntClear(_RTCC_IF_MASK);

	/* Configure & enable module interrupts */
	dev_cfg->irq_config();

	k_timer_init(&data->sync_timer, sync_timer_handler, NULL);
	k_work_init(&data->alarm_work, alarm_worker);
	k_work_init(&data->sqw_work, sqw_worker);
	k_work_init(&data->sync_work, sync_worker);
	data->isw_callback = isw_rtcc_callback;

	rc = update_registers(dev);
	if (rc < 0) {
		LOG_WRN("Failed to fetch registers: %d", rc);
		goto out;
	}

	rc = update_posix(dev);
	if (rc < 0) {
		LOG_WRN("Failed to set date: %d", rc);
		goto out;
	}

out:
	k_sem_give(&data->lock);

	LOG_INF("Device %s initialized %d", DEV_NAME(dev), rc);
	if (rc > 0) {
		rc = 0;
	}

	return rc;
}

static int counter_gecko_start(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -EALREADY;
}

static int counter_gecko_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int counter_gecko_set_alarm(const struct device *dev, uint8_t id,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_gecko_data *data = dev->data;
	const struct counter_gecko_config *cfg = dev->config;
	time_t when;
	int rc = 0;

	if (id >= cfg->generic.channels) {
		rc = -ENOTSUP;
		goto out;
	}

	k_sem_take(&data->lock, K_FOREVER);

	if (data->alarm[id].callback != NULL) {
		rc = -EBUSY;
		goto out_locked;
	}

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0) {
		rc = read_time(dev, &when);
		if (rc >= 0) {
			when += alarm_cfg->ticks;
		}
	} else {
		when = alarm_cfg->ticks;
	}

	RTCC_IntClear(RTCC_IF_CC0 << id);

	data->alarm[id].callback = alarm_cfg->callback;
	data->alarm[id].user_data = alarm_cfg->user_data;

	RTCC_ChannelCCVSet(id, when);

	/* Enable the compare interrupt */
	RTCC_IntEnable(RTCC_IF_CC0 << id);

	struct gecko_rtcc_alarm alarm = {
		.time = (uint32_t)when,
		.handler = counter_alarm_forwarder,
		.user_data = alarm_cfg->user_data,
		.flags = 0,
	};

	if (rc >= 0) {
		data->alarm_handler[id] = alarm_cfg->callback;
		data->counter_ticks[id] = alarm.time;
		rc = set_alarm(dev, id, &alarm);
	}

out_locked:
	k_sem_give(&data->lock);

out:
	/* Throw away information counter API disallows */
	if (rc >= 0) {
		rc = 0;
	}

	return rc;
}

static uint32_t counter_gecko_get_top_value(const struct device *dev)
{
	ARG_UNUSED(dev);

	return UINT32_MAX;
}

static uint32_t counter_gecko_get_pending_int(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int counter_gecko_set_top_value(const struct device *dev,
				       const struct counter_top_cfg *cfg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cfg);

	return -ENOTSUP;
}

static const struct counter_driver_api counter_gecko_driver_api = {
	.start = counter_gecko_start,
	.stop = counter_gecko_stop,
	.get_value = counter_gecko_get_value,
	.set_alarm = counter_gecko_set_alarm,
	.cancel_alarm = counter_gecko_cancel_alarm,
	.set_top_value = counter_gecko_set_top_value,
	.get_pending_int = counter_gecko_get_pending_int,
	.get_top_value = counter_gecko_get_top_value,
};

/* RTCC0 */

ISR_DIRECT_DECLARE(gecko_rtcc_isr_0)
{
	const struct device *dev = DEVICE_DT_INST_GET(0);
	struct counter_gecko_data *const dev_data = DEV_DATA(dev);
	counter_alarm_callback_t alarm_callback;
	uint32_t count = RTCC_CounterGet();
	uint32_t flags = RTCC_IntGetEnabled();

	RTCC_IntClear(flags);

	if (dev_data->isw_callback) {
		alarm_callback = dev_data->isw_callback;
		alarm_callback(dev, 0, count, (void *)flags);
	}

	ISR_DIRECT_PM();

	return 1;
}

BUILD_ASSERT((DT_INST_PROP(0, prescaler) > 0U) &&
	     (DT_INST_PROP(0, prescaler) <= 32768U));

static void counter_gecko_0_irq_config(void)
{
	IRQ_DIRECT_CONNECT(DT_INST_IRQN(0),
			   DT_INST_IRQ(0, priority),
			   gecko_rtcc_isr_0, 0);
	irq_enable(DT_INST_IRQN(0));
}

static const struct counter_gecko_config counter_gecko_0_config = {
	.generic = {
		.max_top_value = RTCC_MAX_VALUE,
		.freq = 1,
		.flags = COUNTER_CONFIG_INFO_COUNT_UP,
		.channels = RTCC_ALARM_NUM,
	},
	.irq_config = counter_gecko_0_irq_config,
	.prescaler = DT_INST_PROP(0, prescaler),
};

static struct counter_gecko_data counter_gecko_0_data;

DEVICE_DT_INST_DEFINE(0, counter_gecko_init, NULL,
	&counter_gecko_0_data, &counter_gecko_0_config,
	PRE_KERNEL_1, CONFIG_COUNTER_INIT_PRIORITY,
	&counter_gecko_driver_api);

#ifdef CONFIG_USERSPACE

#include <zephyr/syscall_handler.h>

int z_vrfy_gecko_rtcc_get_syncpoint(const struct device *dev,
				      struct gecko_rtcc_syncpoint *syncpoint)
{
	struct gecko_rtcc_syncpoint value;
	int rv;

	Z_OOPS(Z_SYSCALL_SPECIFIC_DRIVER(dev, K_OBJ_DRIVER_COUNTER, &counter_gecko_driver_api));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(syncpoint, sizeof(*syncpoint)));

	rv = z_impl_gecko_rtcc_get_syncpoint(dev, &value);

	if (rv >= 0) {
		Z_OOPS(z_user_to_copy(syncpoint, &value, sizeof(*syncpoint)));
	}

	return rv;
}

#include <syscalls/gecko_rtcc_get_syncpoint_mrsh.c>

int z_vrfy_gecko_rtcc_req_syncpoint(const struct device *dev,
				      struct k_poll_signal *sig)
{
	Z_OOPS(Z_SYSCALL_SPECIFIC_DRIVER(dev, K_OBJ_DRIVER_COUNTER, &counter_gecko_driver_api));
	if (sig != NULL) {
		Z_OOPS(Z_SYSCALL_OBJ(sig, K_OBJ_POLL_SIGNAL));
	}

	return z_impl_gecko_rtcc_req_syncpoint(dev, sig);
}

#include <syscalls/gecko_rtcc_req_syncpoint_mrsh.c>

#endif /* CONFIG_USERSPACE */
