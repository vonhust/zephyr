/*
 * Copyright (c) 2022, openEuler Embedded
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_IPC_RPMSG_RPC_SERVICE_H
#define ZEPHYR_SUBSYS_IPC_RPMSG_RPC_SERVICE_H

#include <zephyr/toolchain.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RPMSG_SYS_SERVICE_POWER_OFF 1

#define RPMSG_RPC_SERVICE_NAME "rpmsg-rpc"

/* RPMSG_BUFFER_SIZE = 512
 * sizeof(struct rpmsg_hdr) = 16
 * RPMSG_BUFFER_SIZE - sizeof(struct rpmsg_hdr) - 4 = 492
 * Aligning to 64 bits -> 488UL
 */
#define MAX_BUF_LEN	488UL
#define RPC_ID_LEN sizeof(uint32_t)
/*
 * rpc service call back
 */
typedef int (*rpmsg_rpc_cb_t)(void *params, size_t len);

struct rpmsg_rpc_service {
	uint32_t id;
	rpmsg_rpc_cb_t cb_function;
};

struct rpmsg_rpc_data {
	uint32_t id;    /* rpc id */
	unsigned char params[MAX_BUF_LEN];
} __packed;

struct rpmsg_rpc_instance {
	unsigned int ep_id; /* endpoint id */
	const struct rpmsg_rpc_service *services; /* service table */
	unsigned int n_services; /* number of services */
};

int rpmsg_rpc_service_init(struct rpmsg_rpc_instance *inst,
					const struct rpmsg_rpc_service *services,
					unsigned int n_services);

int rpmsg_rpc_send(struct rpmsg_rpc_instance *inst, uint32_t rpc_id, void *params, size_t len);


#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_IPC_RPMSG_RPC_SERVICE_H */
