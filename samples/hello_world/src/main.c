/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>


int main(void)
{
	while(1){
		/* Sleep for 1 second */
		k_sleep(K_SECONDS(1));
		printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
	}

	return 0;
}
