# SPDX-License-Identifier: Apache-2.0

# Once we have more than 10 devices we should consider splitting them into
# subdirectories to match the drivers/ structure.
zephyr_library_sources_ifdef(CONFIG_EMUL_EEPROM_AT2X	emul_atmel_at24.c)

zephyr_include_directories_ifdef(CONFIG_EMUL_SBS_GAUGE ${ZEPHYR_BASE}/drivers/sensor/sbs_gauge)
zephyr_library_sources_ifdef(CONFIG_EMUL_SBS_GAUGE		emul_sbs_gauge.c)

zephyr_include_directories_ifdef(CONFIG_EMUL_BC12_PI3USB9201 ${ZEPHYR_BASE}/drivers/usb/bc12)
zephyr_library_sources_ifdef(CONFIG_EMUL_BC12_PI3USB9201	emul_bc12_pi3usb9201.c)
