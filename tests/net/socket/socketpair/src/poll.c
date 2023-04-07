/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "_main.h"

struct ctx {
	bool should_write;
	int fd;
	k_timeout_t delay;
};
static ZTEST_BMEM struct ctx ctx;
static ZTEST_BMEM struct k_work work;

/*
 * Timeout should work the same for blocking & non-blocking threads
 *
 *   - no bytes available to read after timeout, r: 0 (timeout)
 *   - no bytes available to write after timeout, r: 0 (timeout)
 */

static void test_socketpair_poll_timeout_common(int sv[2])
{
	int res;

	struct pollfd fds[1];

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLIN;
	res = poll(fds, 1, 1);
	zassert_equal(res, 0, "poll: expected: 0 actual: %d", res);

	for (size_t i = 0; i < CONFIG_NET_SOCKETPAIR_BUFFER_SIZE; ++i) {
		res = send(sv[0], "x", 1, 0);
		zassert_equal(res, 1, "send() failed: %d", res);
	}

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;
	res = poll(fds, 1, 1);
	zassert_equal(res, 0, "poll: expected: 0 actual: %d", res);

	close(sv[0]);
	close(sv[1]);
}

ZTEST_USER(net_socketpair, test_poll_timeout)
{
	int sv[2] = {-1, -1};
	int res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);

	zassert_not_equal(res, -1, "socketpair failed: %d", errno);

	test_socketpair_poll_timeout_common(sv);
}

/* O_NONBLOCK should have no affect on poll(2) */
ZTEST_USER(net_socketpair, test_poll_timeout_nonblocking)
{
	int sv[2] = {-1, -1};
	int res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);

	zassert_not_equal(res, -1, "socketpair failed: %d", errno);

	res = fcntl(sv[0], F_GETFL, 0);
	zassert_not_equal(res, -1, "fcntl failed: %d", errno);

	int flags = res;

	res = fcntl(sv[0], F_SETFL, O_NONBLOCK | flags);
	zassert_not_equal(res, -1, "fcntl failed: %d", errno);

	res = fcntl(sv[1], F_SETFL, O_NONBLOCK | flags);
	zassert_not_equal(res, -1, "fcntl failed: %d", errno);

	test_socketpair_poll_timeout_common(sv);
}

static void close_fun(struct k_work *work)
{
	(void)work;

	if (!(K_TIMEOUT_EQ(ctx.delay, K_NO_WAIT)
		|| K_TIMEOUT_EQ(ctx.delay, K_FOREVER))) {
		k_sleep(ctx.delay);
	}

	LOG_DBG("about to close fd %d", ctx.fd);
	close(ctx.fd);
}

/*
 * Hangup should cause the following behaviour
 *   - close remote fd while the local fd is blocking in poll. r: 1,
 *     POLLIN, read -> r: 0, errno: 0 -> EOF
 *   - close remote fd while the local fd is blocking in poll. r: 1,
 *     POLLOUT, write -> r: -1, errno: EPIPE.
 */
ZTEST(net_socketpair, test_poll_close_remote_end_POLLIN)
{
	int res;
	char c;
	struct pollfd fds[1];

	int sv[2] = {-1, -1};

	res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
	zassert_not_equal(res, -1, "socketpair() failed: %d", errno);

	/*
	 * poll until there are bytes to read.
	 * But rather than writing, close the other end of the channel
	 */

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLIN;

	memset(&ctx, 0, sizeof(ctx));
	ctx.fd = sv[1];
	ctx.delay = K_MSEC(1000);

	LOG_DBG("scheduling work");
	k_work_init(&work, close_fun);
	k_work_submit(&work);

	res = poll(fds, 1, -1);
	zassert_equal(res, 1, "poll() failed: %d", res);
	zassert_equal(fds[0].revents & POLLIN, POLLIN, "POLLIN not set");

	res = recv(sv[0], &c, 1, 0);
	zassert_equal(res, 0, "read did not return EOF");

	close(sv[0]);
}

ZTEST(net_socketpair, test_poll_close_remote_end_POLLOUT)
{
	int res;
	struct pollfd fds[1];

	int sv[2] = {-1, -1};

	/*
	 * Fill up the remote q and then poll until write space is available.
	 * But rather than reading, close the other end of the channel
	 */

	res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
	zassert_not_equal(res, -1, "socketpair() failed: %d", errno);

	for (size_t i = 0; i < CONFIG_NET_SOCKETPAIR_BUFFER_SIZE; ++i) {
		res = send(sv[0], "x", 1, 0);
		zassert_equal(res, 1, "send failed: %d", res);
	}

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;

	memset(&ctx, 0, sizeof(ctx));
	ctx.fd = sv[1];
	ctx.delay = K_MSEC(1000);

	LOG_DBG("scheduling work");
	k_work_init(&work, close_fun);
	k_work_submit(&work);

	res = poll(fds, 1, -1);
	zassert_equal(res, 1, "poll() failed: %d", res);
	zassert_equal(fds[0].revents & POLLHUP, POLLHUP, "POLLHUP not set");

	res = send(sv[0], "x", 1, 0);
	zassert_equal(res, -1, "send(): expected: -1 actual: %d", res);
	zassert_equal(errno, EPIPE, "errno: expected: EPIPE actual: %d", errno);

	close(sv[0]);
}

/*
 * Data available immediately
 *   - even with a timeout value of 0 us, poll should return immediately with
 *     a value of 1 (for either read or write cases)
 *   - even with a timeout value of 0us, poll should return immediately with
 *     a value of 2 if both read and write are available
 */
ZTEST_USER(net_socketpair, test_poll_immediate_data)
{
	int sv[2] = {-1, -1};
	int res;

	struct pollfd fds[2];

	res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
	zassert_not_equal(res, -1, "socketpair() failed: %d", errno);

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;
	res = poll(fds, 1, 0);
	zassert_not_equal(res, -1, "poll() failed: %d", errno);
	zassert_equal(res, 1, "poll(): expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLOUT, 0, "POLLOUT not set");

	res = send(sv[0], "x", 1, 0);
	zassert_not_equal(res, -1, "send() failed: %d", errno);
	zassert_equal(res, 1, "write(): expected: 1 actual: %d", res);

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[1];
	fds[0].events |= POLLIN;
	res = poll(fds, 1, 0);
	zassert_not_equal(res, -1, "poll() failed: %d", errno);
	zassert_equal(res, 1, "poll(): expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLIN, 0, "POLLIN not set");

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;
	fds[1].fd = sv[1];
	fds[1].events |= POLLIN;
	res = poll(fds, 2, 0);
	zassert_not_equal(res, -1, "poll() failed: %d", errno);
	zassert_equal(res, 2, "poll(): expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLOUT, 0, "POLLOUT not set");
	zassert_not_equal(fds[1].revents & POLLIN, 0, "POLLIN not set");

	close(sv[0]);
	close(sv[1]);
}

static void rw_fun(struct k_work *work)
{
	(void)work;

	int res;
	char c;

	if (!(K_TIMEOUT_EQ(ctx.delay, K_NO_WAIT)
		|| K_TIMEOUT_EQ(ctx.delay, K_FOREVER))) {
		k_sleep(ctx.delay);
	}

	if (ctx.should_write) {
		LOG_DBG("about to write 1 byte");
		res = send(ctx.fd, "x", 1, 0);
		if (-1 == res) {
			LOG_DBG("send() failed: %d", errno);
		} else {
			LOG_DBG("wrote 1 byte");
		}
	} else {
		LOG_DBG("about to read 1 byte");
		res = recv(ctx.fd, &c, 1, 0);
		if (-1 == res) {
			LOG_DBG("recv() failed: %d", errno);
		} else {
			LOG_DBG("read 1 byte");
		}
	}
}

/*
 * Data only available but after some short period
 *   - say there is a timeout value of 5 s, poll should return immediately
 *     with the a value of 1 (for either read or write cases)
 */
ZTEST(net_socketpair, test_poll_delayed_data)
{
	int sv[2] = {-1, -1};
	int res;

	struct pollfd fds[1];

	res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
	zassert_not_equal(res, -1, "socketpair() failed: %d", errno);

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLIN;

	memset(&ctx, 0, sizeof(ctx));
	ctx.fd = sv[1];
	ctx.should_write = true;
	ctx.delay = K_MSEC(100);

	LOG_DBG("scheduling work");
	k_work_init(&work, rw_fun);
	k_work_submit(&work);

	res = poll(fds, 1, 5000);
	zassert_not_equal(res, -1, "poll() failed: %d", errno);
	zassert_equal(res, 1, "poll(): expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLIN, 0, "POLLIN not set");

	for (size_t i = 0; i < CONFIG_NET_SOCKETPAIR_BUFFER_SIZE; ++i) {
		res = send(sv[0], "x", 1, 0);
		zassert_equal(res, 1, "send() failed: %d", res);
	}

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;

	memset(&ctx, 0, sizeof(ctx));
	ctx.fd = sv[1];
	ctx.should_write = false;
	ctx.delay = K_MSEC(100);

	LOG_DBG("scheduling work");
	k_work_init(&work, rw_fun);
	k_work_submit(&work);

	res = poll(fds, 1, 5000);
	zassert_not_equal(res, -1, "poll() failed: %d", errno);
	zassert_equal(res, 1, "poll(): expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLOUT, 0, "POLLOUT was not set");

	close(sv[0]);
	close(sv[1]);
}

/*
 * Verify that POLLIN is correctly signalled
 *   - right after socket creation, POLLIN should not be reported
 *   - after data is written to a remote socket, POLLIN should be reported, even
 *     if the poll was called after the data was written
 *   - after reading data from a remote socket, POLLIN shouldn't be reported
 */
ZTEST_USER(net_socketpair, test_poll_signalling_POLLIN)
{
	int sv[2] = {-1, -1};
	int res;
	char c;
	int64_t timestamp, delta;

	struct pollfd fds[1];

	res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
	zassert_not_equal(res, -1, "socketpair failed: %d", errno);

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[1];
	fds[0].events |= POLLIN;
	res = poll(fds, 1, 0);
	zassert_not_equal(res, -1, "poll failed: %d", errno);
	zassert_equal(res, 0, "poll: expected: 0 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLIN, POLLIN, "POLLIN set");

	res = send(sv[0], "x", 1, 0);
	zassert_equal(res, 1, "send failed: %d", res);

	timestamp = k_uptime_get();

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[1];
	fds[0].events |= POLLIN;
	res = poll(fds, 1, 1000);
	zassert_not_equal(res, -1, "poll failed: %d", errno);
	zassert_equal(res, 1, "poll: expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLIN, 0, "POLLIN not set");

	delta = k_uptime_delta(&timestamp);
	zassert_true(delta < 100, "poll did not exit immediately");

	res = recv(sv[1], &c, 1, 0);
	zassert_equal(res, 1, "recv failed: %d", res);

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[1];
	fds[0].events |= POLLIN;
	res = poll(fds, 1, 0);
	zassert_not_equal(res, -1, "poll failed: %d", errno);
	zassert_equal(res, 0, "poll: expected: 0 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLIN, POLLIN, "POLLIN set");

	close(sv[0]);
	close(sv[1]);
}

/*
 * Verify that POLLOUT is correctly signalled
 *   - right after socket creation, POLLOUT should be reported
 *   - after remote buffer is filled up, POLLOUT shouldn't be reported
 *   - after reading data from a remote socket, POLLOUT should be reported
 *     again
 */
ZTEST_USER(net_socketpair, test_poll_signalling_POLLOUT)
{
	int sv[2] = {-1, -1};
	int res;
	char c;
	int64_t timestamp, delta;

	struct pollfd fds[1];

	res = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
	zassert_not_equal(res, -1, "socketpair failed: %d", errno);

	timestamp = k_uptime_get();

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;
	res = poll(fds, 1, 1000);
	zassert_not_equal(res, -1, "poll failed: %d", errno);
	zassert_equal(res, 1, "poll: expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLOUT, 0, "POLLOUT not set");

	delta = k_uptime_delta(&timestamp);
	zassert_true(delta < 100, "poll did not exit immediately");

	/* Fill up the remote buffer */
	for (size_t i = 0; i < CONFIG_NET_SOCKETPAIR_BUFFER_SIZE; ++i) {
		res = send(sv[0], "x", 1, 0);
		zassert_equal(res, 1, "send() failed: %d", res);
	}

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;
	res = poll(fds, 1, 0);
	zassert_not_equal(res, -1, "poll failed: %d", errno);
	zassert_equal(res, 0, "poll: expected: 0 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLOUT, POLLOUT, "POLLOUT is set");

	res = recv(sv[1], &c, 1, 0);
	zassert_equal(res, 1, "recv() failed: %d", res);

	timestamp = k_uptime_get();

	memset(fds, 0, sizeof(fds));
	fds[0].fd = sv[0];
	fds[0].events |= POLLOUT;
	res = poll(fds, 1, 1000);
	zassert_not_equal(res, -1, "poll failed: %d", errno);
	zassert_equal(res, 1, "poll: expected: 1 actual: %d", res);
	zassert_not_equal(fds[0].revents & POLLOUT, 0, "POLLOUT not set");

	delta = k_uptime_delta(&timestamp);
	zassert_true(delta < 100, "poll did not exit immediately");

	close(sv[0]);
	close(sv[1]);
}
