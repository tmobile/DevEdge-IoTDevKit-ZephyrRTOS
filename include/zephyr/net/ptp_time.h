/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public functions for the Precision Time Protocol time specification.
 *
 */

#ifndef ZEPHYR_INCLUDE_NET_PTP_TIME_H_
#define ZEPHYR_INCLUDE_NET_PTP_TIME_H_

/**
 * @brief Precision Time Protocol time specification
 * @defgroup ptp_time PTP time
 * @ingroup networking
 * @{
 */

#include <zephyr/net/net_core.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Precision Time Protocol Timestamp format.
 *
 * This structure represents a timestamp according
 * to the Precision Time Protocol standard.
 *
 * Seconds are encoded as a 48 bits unsigned integer.
 * Nanoseconds are encoded as a 32 bits unsigned integer.
 */
struct net_ptp_time {
	/** Seconds encoded on 48 bits. */
	union {
		struct {
#ifdef CONFIG_LITTLE_ENDIAN
			uint32_t low;
			uint16_t high;
			uint16_t unused;
#else
			uint16_t unused;
			uint16_t high;
			uint32_t low;
#endif
		} _sec;
		uint64_t second;
	};

	/** Nanoseconds. */
	uint32_t nanosecond;
};

#ifdef __cplusplus
}
#endif

/**
 * @brief Precision Time Protocol Extended Timestamp format.
 *
 * This structure represents an extended timestamp according
 * to the Precision Time Protocol standard.
 *
 * Seconds are encoded as 48 bits unsigned integer.
 * Fractional nanoseconds are encoded as 48 bits, their unit
 * is 2*(-16) ns.
 */
struct net_ptp_extended_time {
	/** Seconds encoded on 48 bits. */
	union {
		struct {
#ifdef CONFIG_LITTLE_ENDIAN
			uint32_t low;
			uint16_t high;
			uint16_t unused;
#else
			uint16_t unused;
			uint16_t high;
			uint32_t low;
#endif
		} _sec;
		uint64_t second;
	};

	/** Fractional nanoseconds on 48 bits. */
	union {
		struct {
#ifdef CONFIG_LITTLE_ENDIAN
			uint32_t low;
			uint16_t high;
			uint16_t unused;
#else
			uint16_t unused;
			uint16_t high;
			uint32_t low;
#endif
		} _fns;
		uint64_t fract_nsecond;
	};
} __packed;

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_NET_PTP_TIME_H_ */
