# Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ARM_MMU mmu_regions.c)

set(SOC_LINKER_SCRIPT  ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
if(CONFIG_OPENAMP_RSC_TABLE)
  zephyr_linker_section(NAME .resource_table GROUP ROM_REGION NOINPUT)
  zephyr_linker_section_configure(SECTION .resource_table KEEP INPUT ".resource_table*")
endif()
