# Copyright (c) 2022 openEuler Embedded
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd "--use-elf" "--config=${BOARD_DIR}/support/rpi4_cortex_a72.cfg")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
