/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT efinix_sapphire_spi

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(spi_efinix_sapphire, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

/* Register offsets */
#define EFX_SPI_DATA           0x00U
#define EFX_SPI_BUFFER         0x04U
#define EFX_SPI_CONFIG         0x08U
#define EFX_SPI_CLK_DIVIDER    0x20U
#define EFX_SPI_SS_SETUP       0x24U
#define EFX_SPI_SS_HOLD        0x28U
#define EFX_SPI_SS_DISABLE     0x2CU

/* Command bits written to DATA register */
#define EFX_SPI_CMD_WRITE      BIT(8)
#define EFX_SPI_CMD_READ       BIT(9)
#define EFX_SPI_CMD_SS         BIT(11)

/* BUFFER register fields */
#define EFX_SPI_CMD_AVAIL_MASK GENMASK(15, 0)
#define EFX_SPI_RSP_OCC_MASK   GENMASK(31, 16)

#define EFX_SPI_WAIT_LOOPS     1000000U

/* Tuned values from bare-metal reference config */
#define EFX_SPI_SS_SETUP_CYC   5U
#define EFX_SPI_SS_HOLD_CYC    2U
#define EFX_SPI_SS_DISABLE_CYC 7U

struct spi_efinix_sapphire_cfg {
	mem_addr_t base;
};

struct spi_efinix_sapphire_data {
	struct spi_context ctx;
};

static inline uint32_t spi_reg_read(const struct spi_efinix_sapphire_cfg *cfg, uint32_t reg)
{
	return sys_read32(cfg->base + reg);
}

static inline void spi_reg_write(const struct spi_efinix_sapphire_cfg *cfg, uint32_t reg, uint32_t val)
{
	sys_write32(val, cfg->base + reg);
}

static int spi_wait_cmd_avail(const struct spi_efinix_sapphire_cfg *cfg)
{
	for (uint32_t i = 0; i < EFX_SPI_WAIT_LOOPS; i++) {
		if ((spi_reg_read(cfg, EFX_SPI_BUFFER) & EFX_SPI_CMD_AVAIL_MASK) != 0U) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int spi_wait_rsp_ready(const struct spi_efinix_sapphire_cfg *cfg)
{
	for (uint32_t i = 0; i < EFX_SPI_WAIT_LOOPS; i++) {
		if ((spi_reg_read(cfg, EFX_SPI_BUFFER) & EFX_SPI_RSP_OCC_MASK) != 0U) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int spi_hw_select(const struct spi_efinix_sapphire_cfg *cfg, uint16_t slave)
{
	int rc = spi_wait_cmd_avail(cfg);

	if (rc < 0) {
		return rc;
	}

	spi_reg_write(cfg, EFX_SPI_DATA, (uint32_t)slave | 0x80U | EFX_SPI_CMD_SS);
	return 0;
}

static int spi_hw_deselect(const struct spi_efinix_sapphire_cfg *cfg, uint16_t slave)
{
	int rc = spi_wait_cmd_avail(cfg);

	if (rc < 0) {
		return rc;
	}

	spi_reg_write(cfg, EFX_SPI_DATA, (uint32_t)slave | EFX_SPI_CMD_SS);
	return 0;
}

static int spi_efinix_sapphire_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_efinix_sapphire_cfg *cfg = dev->config;
	struct spi_efinix_sapphire_data *data = dev->data;
	uint32_t op = config->operation;
	uint32_t ctrl = 0U;
	uint32_t divider;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(op) != SPI_OP_MODE_MASTER) {
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(op) != 8U) {
		return -ENOTSUP;
	}

	if ((op & SPI_HALF_DUPLEX) != 0U) {
		return -ENOTSUP;
	}

	if ((op & SPI_TRANSFER_LSB) != 0U) {
		return -ENOTSUP;
	}

	if ((op & SPI_MODE_LOOP) != 0U) {
		return -ENOTSUP;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    ((op & SPI_LINES_MASK) != SPI_LINES_SINGLE)) {
		return -ENOTSUP;
	}

	if (config->frequency == 0U) {
		return -EINVAL;
	}

	/*
	 * Hardware divider in bare-metal is programmed directly. Use a standard
	 * f_out ~= f_sys / (2 * (divider + 1)) mapping.
	 */
	divider = DIV_ROUND_UP(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, 2U * config->frequency);
	if (divider > 0U) {
		divider -= 1U;
	}

	if ((op & SPI_MODE_CPOL) != 0U) {
		ctrl |= BIT(0);
	}

	if ((op & SPI_MODE_CPHA) != 0U) {
		ctrl |= BIT(1);
	}

	spi_reg_write(cfg, EFX_SPI_CONFIG, ctrl);
	spi_reg_write(cfg, EFX_SPI_CLK_DIVIDER, divider);
	spi_reg_write(cfg, EFX_SPI_SS_SETUP, EFX_SPI_SS_SETUP_CYC);
	spi_reg_write(cfg, EFX_SPI_SS_HOLD, EFX_SPI_SS_HOLD_CYC);
	spi_reg_write(cfg, EFX_SPI_SS_DISABLE, EFX_SPI_SS_DISABLE_CYC);

	data->ctx.config = config;
	return 0;
}

static int spi_efinix_sapphire_xfer(const struct device *dev)
{
	const struct spi_efinix_sapphire_cfg *cfg = dev->config;
	struct spi_efinix_sapphire_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	while (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
		size_t len = spi_context_longest_current_buf(ctx);

		for (size_t i = 0; i < len; i++) {
			bool tx_on = spi_context_tx_buf_on(ctx);
			bool rx_on = spi_context_rx_buf_on(ctx);
			uint32_t cmd = 0U;
			int rc;

			if (tx_on) {
				cmd |= *ctx->tx_buf;
				cmd |= EFX_SPI_CMD_WRITE;
				spi_context_update_tx(ctx, 1, 1);
			}

			if (rx_on) {
				cmd |= EFX_SPI_CMD_READ;
			}

			rc = spi_wait_cmd_avail(cfg);
			if (rc < 0) {
				return rc;
			}

			spi_reg_write(cfg, EFX_SPI_DATA, cmd);

			if (rx_on) {
				rc = spi_wait_rsp_ready(cfg);
				if (rc < 0) {
					return rc;
				}

				*ctx->rx_buf = (uint8_t)spi_reg_read(cfg, EFX_SPI_DATA);
				spi_context_update_rx(ctx, 1, 1);
			}
		}
	}

	return 0;
}

static int spi_efinix_sapphire_transceive(const struct device *dev,
					  const struct spi_config *config,
					  const struct spi_buf_set *tx_bufs,
					  const struct spi_buf_set *rx_bufs)
{
	const struct spi_efinix_sapphire_cfg *cfg = dev->config;
	struct spi_efinix_sapphire_data *data = dev->data;
	int rc;

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	rc = spi_efinix_sapphire_configure(dev, config);
	if (rc < 0) {
		spi_context_release(&data->ctx, rc);
		return rc;
	}

	if (config->slave > UINT8_MAX) {
		spi_context_release(&data->ctx, -EINVAL);
		return -EINVAL;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (spi_cs_is_gpio(config)) {
		spi_context_cs_control(&data->ctx, true);
	}

	rc = spi_hw_select(cfg, config->slave);
	if (rc < 0) {
		goto out;
	}

	rc = spi_efinix_sapphire_xfer(dev);

	/* Always deassert hardware CS after this transfer. */
	(void)spi_hw_deselect(cfg, config->slave);

out:
	if (spi_cs_is_gpio(config)) {
		spi_context_cs_control(&data->ctx, false);
	}

	spi_context_complete(&data->ctx, dev, rc);
	rc = spi_context_wait_for_completion(&data->ctx);
	spi_context_release(&data->ctx, rc);

	return rc;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_efinix_sapphire_transceive_async(const struct device *dev,
						const struct spi_config *config,
						const struct spi_buf_set *tx_bufs,
						const struct spi_buf_set *rx_bufs,
						spi_callback_t cb,
						void *userdata)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);
	ARG_UNUSED(tx_bufs);
	ARG_UNUSED(rx_bufs);
	ARG_UNUSED(cb);
	ARG_UNUSED(userdata);

	return -ENOTSUP;
}
#endif

static int spi_efinix_sapphire_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_efinix_sapphire_data *data = dev->data;

	ARG_UNUSED(config);
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static int spi_efinix_sapphire_init(const struct device *dev)
{
	struct spi_efinix_sapphire_data *data = dev->data;
	int rc;

	rc = spi_context_cs_configure_all(&data->ctx);
	if (rc < 0) {
		return rc;
	}

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static DEVICE_API(spi, spi_efinix_sapphire_api) = {
	.transceive = spi_efinix_sapphire_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_efinix_sapphire_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_efinix_sapphire_release,
};

#define SPI_EFINIX_SAPPHIRE_INIT(n)                                                             \
	static const struct spi_efinix_sapphire_cfg spi_efinix_sapphire_cfg_##n = {            \
		.base = DT_INST_REG_ADDR(n),                                                     \
	};                                                                                     \
	static struct spi_efinix_sapphire_data spi_efinix_sapphire_data_##n = {                \
		SPI_CONTEXT_INIT_LOCK(spi_efinix_sapphire_data_##n, ctx),                       \
		SPI_CONTEXT_INIT_SYNC(spi_efinix_sapphire_data_##n, ctx),                       \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)                            \
	};                                                                                     \
	SPI_DEVICE_DT_INST_DEFINE(n,                                                             \
				  spi_efinix_sapphire_init,                                    \
				  NULL,                                                        \
				  &spi_efinix_sapphire_data_##n,                              \
				  &spi_efinix_sapphire_cfg_##n,                               \
				  POST_KERNEL,                                                 \
				  CONFIG_SPI_INIT_PRIORITY,                                    \
				  &spi_efinix_sapphire_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_EFINIX_SAPPHIRE_INIT)
