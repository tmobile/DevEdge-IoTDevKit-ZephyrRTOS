# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2019 Nordic Semiconductor ASA

add_subdirectory(../../src settings_test_bindir)
add_subdirectory(../../file/src settings_test_file_bindir)

zephyr_include_directories(
	${ZEPHYR_BASE}/subsys/settings/include
	${ZEPHYR_BASE}/subsys/settings/src
	${ZEPHYR_BASE}/tests/subsys/settings/file/include
	${ZEPHYR_BASE}/tests/subsys/settings/file_littlefs/src
	)


FILE(GLOB mysources *.c)
target_sources(app PRIVATE ${mysources})
