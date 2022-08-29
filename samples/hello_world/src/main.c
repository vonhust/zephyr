/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>

volatile unsigned int g_counter = 0;

void main(void)
{
	debug_phase[3]=10;

	while(1) {
		k_msleep(1000);
		g_counter++;
		printk("Hello World! %d\n", g_counter);
	}
}
