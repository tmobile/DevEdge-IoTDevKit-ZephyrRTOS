/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MMC disk driver using zephyr SD subsystem
 */
#define DT_DRV_COMPAT zephyr_mmc_disk

#include <zephyr/sd/mmc.h>
#include <zephyr/drivers/disk.h>


enum sd_status {
	SD_UNINIT,
	SD_ERROR,
	SD_OK,
};

struct mmc_config {
	const struct device *host_controller;
};

struct mmc_data {
	struct sd_card card;
	enum sd_status status;
	char *name;
};


static int disk_mmc_access_init(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	const struct mmc_config *cfg = dev->config;
	struct mmc_data *data = dev->data;
	int ret;

	if (data->status == SD_OK) {
		/* Called twice, don't reinit */
		return 0;
	}

	ret = sd_init(cfg->host_controller, &data->card);
	if (ret) {
		data->status = SD_ERROR;
		return ret;
	}
	data->status = SD_OK;
	return 0;
}

static int disk_mmc_access_status(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct mmc_data *data = dev->data;

	if (data->status == SD_OK) {
		return DISK_STATUS_OK;
	} else {
		return DISK_STATUS_UNINIT;
	}
}

static int disk_mmc_access_read(struct disk_info *disk, uint8_t *buf,
				 uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct mmc_data *data = dev->data;

	return mmc_read_blocks(&data->card, buf, sector, count);
}

static int disk_mmc_access_write(struct disk_info *disk, const uint8_t *buf,
				 uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct mmc_data *data = dev->data;

	return mmc_write_blocks(&data->card, buf, sector, count);
}

static int disk_mmc_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buf)
{
	const struct device *dev = disk->dev;
	struct mmc_data *data = dev->data;

	return mmc_ioctl(&data->card, cmd, buf);
}

static const struct disk_operations mmc_disk_ops = {
	.init = disk_mmc_access_init,
	.status = disk_mmc_access_status,
	.read = disk_mmc_access_read,
	.write = disk_mmc_access_write,
	.ioctl = disk_mmc_access_ioctl,
};

static struct disk_info mmc_disk = {
	.ops = &mmc_disk_ops,
};

static int disk_mmc_init(const struct device *dev)
{
	struct mmc_data *data = dev->data;

	data->status = SD_UNINIT;
	mmc_disk.dev = dev;
	mmc_disk.name = data->name;

	return disk_access_register(&mmc_disk);
}

#define DISK_ACCESS_MMC_INIT(n)						\
	static const struct mmc_config mmc_config_##n = {			\
		.host_controller = DEVICE_DT_GET(DT_INST_PARENT(n)),		\
	};									\
										\
	static struct mmc_data mmc_data_##n = {				\
		.name = CONFIG_MMC_VOLUME_NAME,				\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n,						\
			&disk_mmc_init,					\
			NULL,							\
			&mmc_data_##n,					\
			&mmc_config_##n,					\
			POST_KERNEL,						\
			CONFIG_SD_INIT_PRIORITY,				\
			NULL);

DT_INST_FOREACH_STATUS_OKAY(DISK_ACCESS_MMC_INIT)
