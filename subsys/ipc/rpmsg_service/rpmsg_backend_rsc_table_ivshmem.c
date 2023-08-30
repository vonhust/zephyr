/*
 * Copyright (c) 2023, openEuler Embedded
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rpmsg_backend.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/virtualization/ivshmem.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <kernel_arch_interface.h>

#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>

#define LOG_MODULE_NAME rpmsg_backend
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_RPMSG_SERVICE_LOG_LEVEL);

/* Configuration defines */
#define MASTER IS_ENABLED(CONFIG_RPMSG_SERVICE_MODE_MASTER)

#if	IS_ENABLED(CONFIG_RPMSG_SERVICE_MODE_MASTER)
#error "resource table implementation is only for device"
#endif


#define IVSHMEM_EVENT_POLL_STACK_SIZE	8192
#define IVSHMEM_EVENT_POLL_PRIO		K_HIGHEST_APPLICATION_THREAD_PRIO

/* End of configuration defines */

K_THREAD_STACK_DEFINE(ivshmem_event_poll_stack, IVSHMEM_EVENT_POLL_STACK_SIZE);
static struct k_thread ivshmem_event_poll_thread;

static const struct device *ivshmem_dev =
		DEVICE_DT_GET(DT_NODELABEL(ivshmem0));

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

static int virtio_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);
	int status;
	uint16_t peer_dest_id = ivshmem_get_id(ivshmem_dev);

	peer_dest_id -= 1;

	status = ivshmem_int_peer(ivshmem_dev, peer_dest_id, 0);

	if (status != 0) {
		LOG_ERR("ivshmem_int_peer failed to notify: %d", status);
		return status;
	}

	return status;
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
	LOG_DBG("vring_rsc info: num:%d, da:0x%x", vring_rsc->num, vring_rsc->da);
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

static void ivshmem_event_poll_thread_entry(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	/* k_poll was signaled or not */
	unsigned int poll_signaled;
	/* vector received */
	int ivshmem_vector_rx;
	int ret;

	struct k_poll_signal sig;

	struct k_poll_event events[] = {
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
					 K_POLL_MODE_NOTIFY_ONLY,
					 &sig),
	};

	k_poll_signal_init(&sig);

	ret = ivshmem_register_handler(ivshmem_dev, &sig, 0);

	if (ret < 0) {
		LOG_ERR("registering handlers must be supported: %d\n", ret);
		k_panic();
	}

	while (1) {
		LOG_DBG("%s: waiting interrupt from remote peers...\n", __func__);
		ret = k_poll(events, ARRAY_SIZE(events), K_FOREVER);

		k_poll_signal_check(&sig, &poll_signaled, &ivshmem_vector_rx);
		/* get ready for next signal */
		k_poll_signal_reset(&sig);

		/* as remote device, VRING1_ID is for RX (from host to device)*/
		rproc_virtio_notified(cur_vdev, VRING1_ID);
	}
}

int rpmsg_backend_init(struct metal_io_region **io, struct virtio_device **vdev)
{
	uintptr_t	rsc_table;
	struct metal_io_region *rsc_io;
	int rsc_size;
	int32_t                  err;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	struct metal_device     *device;
	size_t shmem_size;
	uintptr_t shmem_phy_addr;
	static metal_phys_addr_t shm_physmap;

	k_thread_create(&ivshmem_event_poll_thread,
			ivshmem_event_poll_stack,
			IVSHMEM_EVENT_POLL_STACK_SIZE,
			(k_thread_entry_t)ivshmem_event_poll_thread_entry,
			NULL, NULL, NULL, IVSHMEM_EVENT_POLL_PRIO, 0, K_NO_WAIT);	
	
	/* get ivshmem info, shmem is already mapped in virt_ivshmem. c*/
	shmem_size = ivshmem_get_rw_mem_section(ivshmem_dev, &shmem_phy_addr);

	LOG_DBG("ivshmem addr is 0x%lx, size is 0x%lx", shmem_phy_addr, shmem_size);

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

	/* part1 resource table 
	 * here resource table will be parsed by the other side and
	 * copied to the 1st page of ivshmem read&write section. 
	 * we call rst_table_get to get the correct size and init meta
	 * io region 
	 *
	 */
	rsc_table_get((void **)&rsc_table, &rsc_size);
	rsc_table = shmem_phy_addr;

	LOG_DBG("rsc_table phys addr is %lx", (uintptr_t)rsc_table);

	metal_io_init(&device->regions[1], (void *)rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);
	rsc_io = metal_device_io_region(device, 1);
	if (!rsc_io) {
		LOG_ERR("Failed to get rsc_io region");
		return -1;
	}

	/* part 2: shared mem used for virtio, skip rsc table */
	shm_physmap = rsc_table + CONFIG_MMU_PAGE_SIZE;
	metal_io_init(&device->regions[0], (void *)(shm_physmap), &shm_physmap,
		      shmem_size - CONFIG_MMU_PAGE_SIZE , -1, 0, NULL);

	/* shared mem io should be return to the caller */
	*io = metal_device_io_region(device, 0);
	if (!*io) {
		LOG_ERR("metal_device_io_region failed to get region");
		return err;
	}
	
	/* virtio device setup */
	*vdev = platform_create_vdev((void *)rsc_table, rsc_io);

	if (*vdev == NULL) {
		return -1;
	}

	/* enable ivshmem device interrupt */
	ivshmem_enable_interrupts(ivshmem_dev, true);

	return 0;
}
