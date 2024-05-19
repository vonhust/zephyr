# Copyright (c) 2024 openEuler Embedded
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd "--use-elf" "--config=${BOARD_DIR}/support/rpi_4b.cfg")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
