/*
 * Copyright (c) 2023 Efinix Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT efinix_sapphire_uart0

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/types.h>

#define BSP_UART_DATA          0x00
#define BSP_UART_STATUS        0x04
#define BSP_UART_CLOCK_DIVIDER 0x08
#define BSP_UART_FRAME_CONFIG  0x0C

#define BSP_UART_WRITE_AVAILABILITY_MASK GENMASK(23, 16)
#define BSP_UART_READ_OCCUPANCY_MASK     GENMASK(31, 24)

#define UART0_DATA_MASK       GENMASK(7, 0)
#define UART0_SAMPLE_PER_BAUD 8
#define UART0_PARITY          0 /* Off */
#define UART0_STOP            0 /* 1 stop bit */


struct uart_efinix_sapphire_config {
	uintptr_t base;
	uint32_t baudrate;
};

static inline uintptr_t uart_reg(const struct uart_efinix_sapphire_config *cfg, uintptr_t off)
{
	return cfg->base + off;
}

static void uart_efinix_sapphire_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_efinix_sapphire_config *cfg = dev->config;

	/* uart_writeAvailability */
	while ((sys_read32(uart_reg(cfg, BSP_UART_STATUS)) & BSP_UART_WRITE_AVAILABILITY_MASK) == 0) {
	}

	/* Sapphire UART data register is accessed as 32-bit in bare-metal BSP. */
	sys_write32((uint32_t)c, uart_reg(cfg, BSP_UART_DATA));
}

static int uart_efinix_sapphire_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_efinix_sapphire_config *cfg = dev->config;

	if ((sys_read32(uart_reg(cfg, BSP_UART_STATUS)) & BSP_UART_READ_OCCUPANCY_MASK) != 0) {
		*c = (unsigned char)(sys_read32(uart_reg(cfg, BSP_UART_DATA)) & UART0_DATA_MASK);
		return 0;
	}

	return -1;
}

static DEVICE_API(uart, uart_efinix_sapphire_api) = {
	.poll_in = uart_efinix_sapphire_poll_in,
	.poll_out = uart_efinix_sapphire_poll_out,
	.err_check = NULL,
};

static int uart_efinix_sapphire_init(const struct device *dev)
{
	const struct uart_efinix_sapphire_config *cfg = dev->config;

	uint32_t prescaler = ((CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC /
			       (cfg->baudrate * UART0_SAMPLE_PER_BAUD)) -
			      1) &
			     0xFFFFF;
	sys_write32(prescaler, uart_reg(cfg, BSP_UART_CLOCK_DIVIDER));

	/* 8 data bits, no parity, 1 stop bit */
	uint32_t frame_config = (UART0_SAMPLE_PER_BAUD - 1) | UART0_PARITY << 8 | UART0_STOP << 16;

	sys_write32(frame_config, uart_reg(cfg, BSP_UART_FRAME_CONFIG));

	return 0;
}

#define UART_EFINIX_SAPPHIRE_INIT(n)                                                            \
	static const struct uart_efinix_sapphire_config uart_efinix_sapphire_cfg_##n = {       \
		.base = DT_INST_REG_ADDR(n),                                                     \
		.baudrate = DT_INST_PROP(n, current_speed),                                      \
	};                                                                                       \
	DEVICE_DT_INST_DEFINE(n, uart_efinix_sapphire_init, NULL, NULL,                          \
			      &uart_efinix_sapphire_cfg_##n, PRE_KERNEL_1,                      \
			      CONFIG_SERIAL_INIT_PRIORITY, (void *)&uart_efinix_sapphire_api)

DT_INST_FOREACH_STATUS_OKAY(UART_EFINIX_SAPPHIRE_INIT)
