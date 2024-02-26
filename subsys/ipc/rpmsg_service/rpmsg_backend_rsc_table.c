/*
 * Copyright (c) 2023, openEuler Embedded
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rpmsg_backend.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pm_cpu_ops.h>
#include <zephyr/drivers/cache.h>
#include <zephyr/logging/log.h>

#include <openamp/open_amp.h>
#include <metal/device.h>
#include <metal/cache.h>
#include <resource_table.h>

#define LOG_MODULE_NAME rpmsg_backend
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_RPMSG_SERVICE_LOG_LEVEL);

/* Configuration defines */
#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Module requires definition of shared memory for rpmsg"
#endif

#define MASTER IS_ENABLED(CONFIG_RPMSG_SERVICE_MODE_MASTER)

#if	IS_ENABLED(CONFIG_RPMSG_SERVICE_MODE_MASTER)
#error "resource table implementation is only for device"
#endif


/* use resource tables's  reserved[0] to carry some extra information
 *  the following IDs come from PSCI definition
 */
#define CPU_OFF_FUNCID		0x84000002
#define CPU_SUSPEND_FUNCID 	0xc4000001
#define CPU_ON_FUNCID		0xC4000003
#define SYSTEM_RESET		0x84000009

#define IPM_WORK_QUEUE_STACK_SIZE CONFIG_RPMSG_SERVICE_WORK_QUEUE_STACK_SIZE
#define IPM_WORK_QUEUE_PRIORITY   K_HIGHEST_APPLICATION_THREAD_PRIO

K_THREAD_STACK_DEFINE(ipm_stack_area, IPM_WORK_QUEUE_STACK_SIZE);

struct k_work_q ipm_work_q;

/* End of configuration defines */

static const struct device *const ipm_handle =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.num_regions = 2,
	.regions = {
	/* shared memory io, only the addr in [share mem start + share mem size]
	 * can be accessed and guaranteed by metal_io_read/write
	 */
		{.virt = NULL},
	/* resource table io, only the addr in [resource table start + table size]
	 * can be accessed and guaranteed by metal_io_read/write
	 */
		{.virt = NULL},
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

static struct virtio_device *cur_vdev;
static struct k_work ipm_work_vring_rx;
static struct k_work ipm_work_cpu_off;

static int virtio_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);
	int status;

	status = ipm_send(ipm_handle, 0, id, NULL, 0);

	if (status != 0) {
		LOG_ERR("ipm_send failed to notify: %d", status);
		return status;
	}

	return status;
}

static void ipm_work_handle_vring_rx(struct k_work *work)
{
	/* as remote device, VRING1_ID is for RX (from host to devive)*/
	rproc_virtio_notified(cur_vdev, VRING1_ID);
}

static void ipm_work_handle_cpu_off(struct k_work *work)
{
	LOG_DBG("zephyr: cpu off");
		/* before cpu_off, some clean ops need to be done:
		 * - turn off tasks, device, etc.
		 * - clear cache, memory etc.
		 */
	cache_data_invd_all();
	cache_instr_invd_all();

	/* \todo: extra work to notify other tasks that cpu is going to
		turn off
	*/

	/* from arm64 supporting psci, call pm_cpu_off to turn off */
	pm_cpu_off();
}

static void reset_vq(void)
{
	if (rvdev.svq != NULL) {
		/*
		 * For svq:
		 * vq_free_cnt: Set to vq_nentries, all descriptors in the svq are available.
		 * vq_queued_cnt: Set to 0, no descriptors waiting to be processed in the svq.
		 * vq_desc_head_idx: Set to 0, the next available descriptor is at the beginning
		 *                   of the descriptor table.
		 * vq_available_idx: Set to 0, No descriptors have been added to the available ring.
		 * vq_used_cons_idx: No descriptors have been added to the used ring.
		 * vq_ring.avail->idx and vq_ring.used->idx will be set at host.
		 */
		rvdev.svq->vq_free_cnt = rvdev.svq->vq_nentries;
		rvdev.svq->vq_queued_cnt = 0;
		rvdev.svq->vq_desc_head_idx = 0;
		rvdev.svq->vq_available_idx = 0;
		rvdev.svq->vq_used_cons_idx = 0;
	}

	if (rvdev.rvq != NULL) {
		/*
		 * For rvq:
		 * Because host resets its tx vq, on the remote side,
		 * it also needs to reset the rx rq.
		 */
		rvdev.rvq->vq_available_idx = 0;
		rvdev.rvq->vq_used_cons_idx = 0;
		rvdev.rvq->vq_ring.used->idx = 0;
		rvdev.rvq->vq_ring.avail->idx = 0;
		metal_cache_flush(&(rvdev.rvq->vq_ring.used->idx),
				  sizeof(rvdev.rvq->vq_ring.used->idx));
		metal_cache_flush(&(rvdev.rvq->vq_ring.avail->idx),
				  sizeof(rvdev.rvq->vq_ring.avail->idx));
	}
}

static void ipm_callback(const struct device *dev,
			 void *context, uint32_t id,
			 volatile void *data)
{
	struct fw_resource_table *rsc;
	uint32_t status;

	rsc = (struct fw_resource_table *)context;
	status = rsc->reserved[0];

	(void)dev;

	LOG_DBG("Got callback of id %u", id);

	/* use resource table's reserved bits for extra work */
	if (status == 0)
		return;

	LOG_DBG("rsc reserved[0]:%x", status);

	if (status == CPU_OFF_FUNCID) {
		/* cpu off work */
		k_work_submit_to_queue(&ipm_work_q, &ipm_work_cpu_off);
	} else if (status == CPU_ON_FUNCID) {
		/* normal work */
		k_work_submit_to_queue(&ipm_work_q, &ipm_work_vring_rx);
	} else if (status == SYSTEM_RESET) {
		/* attach work: reset virtqueue */
		reset_vq();
	}
}

struct virtio_device *
platform_create_vdev(void *rsc_table, struct metal_io_region *rsc_io)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, virtio_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);


	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)(uintptr_t)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)(uintptr_t)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}

	cur_vdev = vdev;
	return vdev;

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

int rpmsg_backend_init(struct metal_io_region **io, struct virtio_device **vdev)
{
	void *rsc_table;
	struct metal_io_region *rsc_io;
	int rsc_size;
	int32_t                  err;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	struct metal_device     *device;

	/* Libmetal setup */
	err = metal_init(&metal_params);
	if (err) {
		LOG_ERR("metal_init: failed - error code %d", err);
		return err;
	}

	err = metal_register_generic_device(&shm_device);
	if (err) {
		LOG_ERR("Couldn't register shared memory device: %d", err);
		return err;
	}

	err = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (err) {
		LOG_ERR("metal_device_open failed: %d", err);
		return err;
	}

	metal_io_init(&device->regions[0], (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	/* shared mem io should be return to the caller */
	*io = metal_device_io_region(device, 0);
	if (!*io) {
		LOG_ERR("metal_device_io_region failed to get region");
		return err;
	}

	rsc_table_get(&rsc_table, &rsc_size);
	metal_io_init(&device->regions[1], rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);
	rsc_io = metal_device_io_region(device, 1);
	if (!rsc_io) {
		LOG_ERR("Failed to get rsc_io region");
		return -1;
	}

	/* IPM setup */

	/* Start IPM workqueue */
	k_work_queue_start(&ipm_work_q, ipm_stack_area,
			   K_THREAD_STACK_SIZEOF(ipm_stack_area),
			   IPM_WORK_QUEUE_PRIORITY, NULL);
	k_thread_name_set(&ipm_work_q.thread, "ipm_work_q");

	/* Setup IPM workqueue item */
	k_work_init(&ipm_work_vring_rx, ipm_work_handle_vring_rx);
	k_work_init(&ipm_work_cpu_off, ipm_work_handle_cpu_off);

	if (!device_is_ready(ipm_handle)) {
		LOG_ERR("IPM device is not ready");
		return -ENODEV;
	}

	ipm_register_callback(ipm_handle, ipm_callback, rsc_table);

	err = ipm_set_enabled(ipm_handle, 1);
	if (err != 0) {
		LOG_ERR("Could not enable IPM interrupts and callbacks");
		return err;
	}

	/* virtio device setup */
	*vdev = platform_create_vdev(rsc_table, rsc_io);

	if (*vdev == NULL) {
		return -1;
	}

	return 0;
}
