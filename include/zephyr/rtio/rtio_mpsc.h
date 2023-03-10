/*
 * Copyright (c) 2010-2011 Dmitry Vyukov
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_RTIO_MPSC_H_
#define ZEPHYR_RTIO_MPSC_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>

/**
 * @brief RTIO Multiple Producer Single Consumer (MPSC) Queue API
 * @defgroup rtio_mpsc RTIO MPSC API
 * @ingroup rtio
 * @{
 */

/**
 * @file rtio_mpsc.h
 *
 * @brief A wait-free intrusive multi producer single consumer (MPSC) queue using
 * a singly linked list. Ordering is First-In-First-Out.
 *
 * Based on the well known and widely used wait-free MPSC queue described by
 * Dmitry Vyukov with some slight changes to account for needs of an
 * RTOS on a variety of archs. Both consumer and producer are wait free. No CAS
 * loop or lock is needed.
 *
 * An MPSC queue is safe to produce or consume in an ISR with O(1) push/pop.
 *
 * @warning MPSC is *not* safe to consume in multiple execution contexts.
 */

/**
 * @brief Queue member
 */
struct rtio_mpsc_node {
	atomic_ptr_t next;
};

/**
 * @brief MPSC Queue
 */
struct rtio_mpsc {
	atomic_ptr_t head;
	struct rtio_mpsc_node *tail;
	struct rtio_mpsc_node stub;
};


/**
 * @brief Static initializer for a mpsc queue
 *
 * Since the queue is
 *
 * @param symbol name of the queue
 */
#define RTIO_MPSC_INIT(symbol)                                 \
	{                                                      \
		.head = (struct rtio_mpsc_node *)&symbol.stub, \
		.tail = (struct rtio_mpsc_node *)&symbol.stub, \
		.stub.next = NULL,                             \
	}

/**
 * @brief Initialize queue
 *
 * @param q Queue to initialize or reset
 */
static inline void rtio_mpsc_init(struct rtio_mpsc *q)
{
	atomic_ptr_set(&q->head, &q->stub);
	q->tail = &q->stub;
	atomic_ptr_set(&q->stub.next, NULL);
}

/**
 * @brief Push a node
 *
 * @param q Queue to push the node to
 * @param n Node to push into the queue
 */
static inline void rtio_mpsc_push(struct rtio_mpsc *q, struct rtio_mpsc_node *n)
{
	struct rtio_mpsc_node *prev;
	int key;

	atomic_ptr_set(&n->next, NULL);

	key = arch_irq_lock();
	prev = atomic_ptr_set(&q->head, n);
	atomic_ptr_set(&prev->next, n);
	arch_irq_unlock(key);
}

/**
 * @brief Pop a node off of the list
 *
 * @retval NULL When no node is available
 * @retval node When node is available
 */
static inline struct rtio_mpsc_node *rtio_mpsc_pop(struct rtio_mpsc *q)
{
	struct rtio_mpsc_node *head;
	struct rtio_mpsc_node *tail = q->tail;
	struct rtio_mpsc_node *next = atomic_ptr_get(&tail->next);

	/* Skip over the stub/sentinel */
	if (tail == &q->stub) {
		if (next == NULL) {
			return NULL;
		}

		q->tail = next;
		tail = next;
		next = atomic_ptr_get(&next->next);
	}

	/* If next is non-NULL then a valid node is found, return it */
	if (next != NULL) {
		q->tail = next;
		return tail;
	}

	head = atomic_ptr_get(&q->head);

	/* If next is NULL, and the tail != HEAD then the queue has pending
	 * updates that can't yet be accessed.
	 */
	if (tail != head) {
		return NULL;
	}

	rtio_mpsc_push(q, &q->stub);

	next = atomic_ptr_get(&tail->next);

	if (next != NULL) {
		q->tail = next;
		return tail;
	}

	return NULL;
}

/**
 * @}
 */

#endif /* ZEPHYR_RTIO_MPSC_H_ */
