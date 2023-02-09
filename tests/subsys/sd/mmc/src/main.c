/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/sd/mmc.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define SECTOR_COUNT 32
#define SECTOR_SIZE  512 /* subsystem should set all cards to 512 byte blocks */
#define BUF_SIZE     (SECTOR_SIZE * SECTOR_COUNT)
static const struct device *const sdhc_dev = DEVICE_DT_GET(DT_ALIAS(sdhc0));
static struct sd_card card;
static uint8_t buf[BUF_SIZE] __aligned(CONFIG_SDHC_BUFFER_ALIGNMENT);
static uint8_t check_buf[BUF_SIZE] __aligned(CONFIG_SDHC_BUFFER_ALIGNMENT);
static uint32_t sector_size;
static uint32_t sector_count;

#define MMC_UNALIGN_OFFSET 1

/*
 * Verify that SD stack can initialize an MMC card
 * This test must run first, to ensure the card is initialized.
 */
ZTEST(sd_stack, test_0_init)
{
	int ret;

	zassert_true(device_is_ready(sdhc_dev), "SDHC device is not ready");

	ret = sd_init(sdhc_dev, &card);

	zassert_equal(ret, 0, "Card initialization failed");
}

/* Verify that MMC stack returns valid IOCTL values */
ZTEST(sd_stack, test_ioctl)
{
	int ret;

	ret = mmc_ioctl(&card, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	zassert_equal(ret, 0, "IOCTL sector count read failed");
	TC_PRINT("SD card reports sector count of %d\n", sector_count);

	ret = mmc_ioctl(&card, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	zassert_equal(ret, 0, "IOCTL sector size read failed");
	TC_PRINT("SD card reports sector size of %d\n", sector_size);
}

/* Verify that SD stack can read from an SD card */
ZTEST(sd_stack, test_read)
{
	int ret;
	int block_addr = 0;

	/* Try simple reads from start of SD card */

	ret = mmc_read_blocks(&card, buf, block_addr, 1);
	zassert_equal(ret, 0, "Single block card read failed");

	ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Multiple block card read failed");

	/* Try a series of reads from the same block */
	block_addr = sector_count / 2;
	for (int i = 0; i < 10; i++) {
		ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT);
		zassert_equal(ret, 0, "Multiple reads from same addr failed");
	}
	/* Verify that out of bounds read fails */
	block_addr = sector_count;
	ret = mmc_read_blocks(&card, buf, block_addr, 1);
	zassert_not_equal(ret, 0, "Out of bounds read should fail");

	block_addr = sector_count - 2;
	ret = mmc_read_blocks(&card, buf, block_addr, 2);
	zassert_equal(ret, 0, "Read from end of card failed");

	/* Verify that unaligned reads work */
	block_addr = 3;
	ret = mmc_read_blocks(&card, buf + MMC_UNALIGN_OFFSET, block_addr, SECTOR_COUNT - 1);
	zassert_equal(ret, 0, "Unaligned read failed");
}

/* Verify that SD stack can write to an SD card */
ZTEST(sd_stack, test_write)
{
	int ret;
	int block_addr = 0;

	/* Try simple writes from start of SD card */

	ret = mmc_write_blocks(&card, buf, block_addr, 1);
	zassert_equal(ret, 0, "Single block card write failed");

	ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Multiple block card write failed");

	/* Try a series of reads from the same block */
	block_addr = sector_count / 2;
	for (int i = 0; i < 10; i++) {
		ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT);
		zassert_equal(ret, 0, "Multiple writes to same addr failed");
	}
	/* Verify that out of bounds write fails */
	block_addr = sector_count;
	ret = mmc_write_blocks(&card, buf, block_addr, 1);
	zassert_not_equal(ret, 0, "Out of bounds write should fail");

	block_addr = sector_count - 2;
	ret = mmc_write_blocks(&card, buf, block_addr, 2);
	zassert_equal(ret, 0, "Write to end of card failed");

	/* Verify that unaligned writes work */
	block_addr = 3;
	ret = mmc_write_blocks(&card, buf + MMC_UNALIGN_OFFSET, block_addr, SECTOR_COUNT - 1);
	zassert_equal(ret, 0, "Unaligned write failed");
}

/* Test reads and writes interleaved, to verify data is making it on disk */
ZTEST(sd_stack, test_rw)
{
	int ret;
	int block_addr = 0;

	/* Zero the write buffer */
	memset(buf, 0, BUF_SIZE);
	memset(check_buf, 0, BUF_SIZE);
	ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Write to card failed");
	/* Verify that a read from this area is empty */
	ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT / 2);
	zassert_equal(ret, 0, "Read from card failed");
	zassert_mem_equal(buf, check_buf, BUF_SIZE, "Read of erased area was not zero");

	/* Now write nonzero data block */
	for (int i = 0; i < sizeof(buf); i++) {
		check_buf[i] = buf[i] = (uint8_t)i;
	}

	ret = mmc_write_blocks(&card, buf, block_addr, SECTOR_COUNT);
	zassert_equal(ret, 0, "Write to card failed");
	/* Clear the read buffer, then write to it again */
	memset(buf, 0, BUF_SIZE);
	ret = mmc_read_blocks(&card, buf, block_addr, SECTOR_COUNT);
	zassert_equal(ret, 0, "Read from card failed");
	zassert_mem_equal(buf, check_buf, BUF_SIZE, "Read of written area was not correct");

	block_addr = (sector_count / 3);
	for (int i = 0; i < 10; i++) {
		/* Verify that unaligned writes work */
		ret = mmc_write_blocks(&card, buf + MMC_UNALIGN_OFFSET, block_addr,
				       SECTOR_COUNT - 1);
		zassert_equal(ret, 0, "Write to card failed");
		/* Zero check buffer and read into it */
		memset(check_buf + MMC_UNALIGN_OFFSET, 0, (SECTOR_COUNT - 1) * sector_size);
		ret = mmc_read_blocks(&card, check_buf + MMC_UNALIGN_OFFSET, block_addr,
				      (SECTOR_COUNT - 1));
		zassert_equal(ret, 0, "Read from card failed");
		zassert_mem_equal(buf + MMC_UNALIGN_OFFSET, check_buf + MMC_UNALIGN_OFFSET,
				  (SECTOR_COUNT - 1) * sector_size,
				  "Unaligned read of written area was not correct");
	}
}

/* Simply dump the card configuration. */
ZTEST(sd_stack, test_card_config)
{
	switch (card.card_voltage) {
	case SD_VOL_1_2_V:
		TC_PRINT("Card voltage: 1.2V\n");
		break;
	case SD_VOL_1_8_V:
		TC_PRINT("Card voltage: 1.8V\n");
		break;
	case SD_VOL_3_0_V:
		TC_PRINT("Card voltage: 3.0V\n");
		break;
	case SD_VOL_3_3_V:
		TC_PRINT("Card voltage: 3.3V\n");
		break;
	default:
		zassert_unreachable("Card voltage is not known value");
	}
	zassert_equal(card.status, CARD_INITIALIZED, "Card status is not OK");
	switch (card.card_speed) {
	case MMC_LEGACY_TIMING:
		TC_PRINT("Card timing: Legacy MMC\n");
		break;
	case MMC_HS_TIMING:
		TC_PRINT("Card timing: High Speed MMC\n");
		break;
	case MMC_HS200_TIMING:
		TC_PRINT("Card timing: MMC HS200\n");
		break;
	case MMC_HS400_TIMING:
		TC_PRINT("Card timing: MMC HS400\n");
		break;
	default:
		zassert_unreachable("Card timing is not known value");
	}
	switch (card.type) {
	case CARD_SDIO:
		TC_PRINT("Card type: SDIO\n");
		break;
	case CARD_SDMMC:
		TC_PRINT("Card type: SDMMC\n");
		break;
	case CARD_COMBO:
		TC_PRINT("Card type: combo card\n");
		break;
	case CARD_MMC:
		TC_PRINT("Card type: MMC\n");
		break;
	default:
		zassert_unreachable("Card type is not known value");
	}
}

ZTEST_SUITE(sd_stack, NULL, NULL, NULL, NULL, NULL);
