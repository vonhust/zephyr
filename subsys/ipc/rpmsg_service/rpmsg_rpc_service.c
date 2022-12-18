/*
 * Copyright (c) 2022, openEuler Embedded
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rpmsg_rpc_service.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/ipc/rpmsg_service.h>
#include <zephyr/drivers/pm_cpu_ops.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME rpmsg_rpc_service
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_RPMSG_SERVICE_LOG_LEVEL);

static const struct rpmsg_rpc_service *find_rpc_service(struct rpmsg_rpc_instance *inst,
					unsigned int id)
{
	const struct rpmsg_rpc_service *service;

	for (unsigned int i = 0; i < inst->n_services; i++) {
		service = &inst->services[i];

		if (service->id == id) {
			return service;
		}
	}

	return NULL;
}

static int endpoint_cb_rpmsg_rpc(struct rpmsg_endpoint *ept, void *data,
					size_t len, uint32_t src, void *priv)
{
	uint32_t id;

	struct rpmsg_rpc_instance *inst;
	const struct rpmsg_rpc_service *service;

	if (len < RPC_ID_LEN) {
		return 0;
	}

	inst = (struct rpmsg_rpc_instance *)priv;

	/* skip rpc id */
	id = *(uint32_t *)data;
	data = (char *)data + RPC_ID_LEN;
	len -= RPC_ID_LEN;

	service = find_rpc_service(inst, id);

	if (service) {
		if (service->cb_function(data, len) < 0) {
			LOG_ERR("call back %d error\n", id);
		}
	} else {
		LOG_ERR("no service found\n");
	}

	return 0;
}

int rpmsg_rpc_service_init(struct rpmsg_rpc_instance *inst,
					const struct rpmsg_rpc_service *services,
					unsigned int n_services)
{
	int ret;

	/* parameter check */
	if (inst == NULL || services == NULL || n_services == 0) {
		return -1;
	}

	inst->services = services;
	inst->n_services = n_services;

	ret = rpmsg_service_register_endpoint(RPMSG_RPC_SERVICE_NAME,
							endpoint_cb_rpmsg_rpc, NULL, inst);

	if (ret >= 0) {
		inst->ep_id = ret;
		return 0;
	} else {
		return ret;
	}
}

int rpmsg_rpc_send(struct rpmsg_rpc_instance *inst, uint32_t rpc_id, void *params, size_t len)
{
	int ret;
	struct rpmsg_rpc_data data;

	if (inst == NULL || params == NULL || len == 0) {
		return -1;
	}

	data.id = rpc_id;
	memcpy(data.params, params, len);

	ret = rpmsg_service_send(inst->ep_id, &data, RPC_ID_LEN + len);

	if (ret > 0) {
		ret -= RPC_ID_LEN;
	}

	return ret;
}

/* this call back is called when the response of
 * sys_service_power_off arrives (in fact maybe never arrives)
 */
static int sys_service_power_off_cb(void *params, size_t len)
{
	/* before cpu_off, some clean ops need to be done:
	 * - turn off tasks, device, etc.
	 * - clear cache, memory etc.
	 */
	sys_cache_data_flush_all();
	sys_cache_instr_flush_all();

	/* from arm64 supporting psci, call pm_cpu_off to turn off */
	pm_cpu_off();

	return 0;
}

static struct rpmsg_rpc_instance sys_service_inst;
static struct rpmsg_rpc_service sys_service_table[] = {
	{RPMSG_SYS_SERVICE_POWER_OFF, sys_service_power_off_cb}
};

static int rpmsg_sys_service_init(void)
{
	unsigned int n_services = sizeof(sys_service_table) / sizeof(struct rpmsg_rpc_service);

	LOG_INF("number of services: %d\n", n_services);

	rpmsg_rpc_service_init(&sys_service_inst, sys_service_table, n_services);

	return 0;
}

SYS_INIT(rpmsg_sys_service_init, POST_KERNEL,  CONFIG_RPMSG_SERVICE_EP_REG_PRIORITY);
