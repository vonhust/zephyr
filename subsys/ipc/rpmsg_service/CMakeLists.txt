# SPDX-License-Identifier: Apache-2.0

if(CONFIG_OPENAMP_RSC_TABLE)
# METAL_MAX_DEVICE_REGIONS is used to give the number of memory regions shared
# between processors. By default only one region is defined for the vrings
# and rpmsg buffers. The METAL_MAX_DEVICE_REGIONS has to be redefined to add a
# second region for the resource table.
zephyr_compile_definitions(METAL_MAX_DEVICE_REGIONS=2)
zephyr_sources(rpmsg_backend_rsc_table.c)
else()
zephyr_sources(rpmsg_backend.c)
endif()
zephyr_sources(rpmsg_service.c)
zephyr_sources(rpmsg_rpc_service.c)
