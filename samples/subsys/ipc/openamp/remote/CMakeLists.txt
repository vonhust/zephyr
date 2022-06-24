cmake_minimum_required(VERSION 3.20.0)
# Copyright (c) 2018 Nordic Semiconductor ASA
# Copyright (c) 2019 Linaro Limited
# Copyright 2022 NXP
#
# SPDX-License-Identifier: Apache-2.0
#


if("${BOARD}" STREQUAL "qemu_cortex_a53")
	set(CONF_FILE "prj_qemu_cortex_a53.conf")
	set(DTC_OVERLAY_FILE CACHE STRING "boards/qemu_cortex_a53.overlay" FORCE)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(openamp_remote)
message(STATUS "${BOARD} compiles as remote in this sample")

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
