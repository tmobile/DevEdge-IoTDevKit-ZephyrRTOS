# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(sx9328.c)
zephyr_library_sources_ifdef(CONFIG_SX9328_TRIGGER sx9328_trigger.c)
