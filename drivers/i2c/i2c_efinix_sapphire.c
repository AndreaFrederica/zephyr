/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT efinix_sapphire_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include "i2c-priv.h"

LOG_MODULE_REGISTER(i2c_efinix_sapphire, CONFIG_I2C_LOG_LEVEL);

/* Register offsets */
#define EFX_I2C_TX_DATA                     0x00U
#define EFX_I2C_TX_ACK                      0x04U
#define EFX_I2C_RX_DATA                     0x08U
#define EFX_I2C_RX_ACK                      0x0CU
#define EFX_I2C_SAMPLING_CLOCK_DIVIDER      0x28U
#define EFX_I2C_TIMEOUT                     0x2CU
#define EFX_I2C_TSUDAT                      0x30U
#define EFX_I2C_MASTER_STATUS               0x40U
#define EFX_I2C_TLOW                        0x50U
#define EFX_I2C_THIGH                       0x54U
#define EFX_I2C_TBUF                        0x58U

/* TX/RX control fields */
#define EFX_I2C_VALUE_MASK                  GENMASK(7, 0)
#define EFX_I2C_TX_VALID                    BIT(8)
#define EFX_I2C_TX_ENABLE                   BIT(9)
#define EFX_I2C_TX_DISABLE_ON_DATA_CONFLICT BIT(11)
#define EFX_I2C_RX_VALID                    BIT(8)

/* Master status fields */
#define EFX_I2C_MASTER_BUSY                 BIT(0)
#define EFX_I2C_MASTER_START                BIT(4)
#define EFX_I2C_MASTER_STOP                 BIT(5)
#define EFX_I2C_MASTER_RECOVER              BIT(7)
#define EFX_I2C_MASTER_START_DROPPED        BIT(9)
#define EFX_I2C_MASTER_STOP_DROPPED         BIT(10)
#define EFX_I2C_MASTER_RECOVER_DROPPED      BIT(11)

#define EFX_I2C_WAIT_LOOPS                  1000000U
#define EFX_I2C_DEFAULT_SAMPLING_DIV        3U

struct i2c_efinix_sapphire_cfg {
	mem_addr_t base;
	uint32_t core_hz;
	uint32_t bus_hz;
};

struct i2c_efinix_sapphire_data {
	uint32_t dev_config;
};

static inline uint32_t i2c_reg_read(const struct i2c_efinix_sapphire_cfg *cfg, uint32_t reg)
{
	return sys_read32(cfg->base + reg);
}

static inline void i2c_reg_write(const struct i2c_efinix_sapphire_cfg *cfg, uint32_t reg, uint32_t val)
{
	sys_write32(val, cfg->base + reg);
}

static int i2c_wait_clear(const struct i2c_efinix_sapphire_cfg *cfg, uint32_t reg, uint32_t mask)
{
	for (uint32_t i = 0; i < EFX_I2C_WAIT_LOOPS; i++) {
		if ((i2c_reg_read(cfg, reg) & mask) == 0U) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int i2c_wait_set(const struct i2c_efinix_sapphire_cfg *cfg, uint32_t reg, uint32_t mask)
{
	for (uint32_t i = 0; i < EFX_I2C_WAIT_LOOPS; i++) {
		if ((i2c_reg_read(cfg, reg) & mask) != 0U) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int i2c_master_start(const struct i2c_efinix_sapphire_cfg *cfg)
{
	i2c_reg_write(cfg, EFX_I2C_MASTER_STATUS, EFX_I2C_MASTER_START | EFX_I2C_MASTER_START_DROPPED);
	return i2c_wait_clear(cfg, EFX_I2C_MASTER_STATUS, EFX_I2C_MASTER_START);
}

static int i2c_master_stop(const struct i2c_efinix_sapphire_cfg *cfg)
{
	i2c_reg_write(cfg, EFX_I2C_MASTER_STATUS, EFX_I2C_MASTER_STOP | EFX_I2C_MASTER_STOP_DROPPED);
	return i2c_wait_clear(cfg, EFX_I2C_MASTER_STATUS, EFX_I2C_MASTER_BUSY);
}

static int i2c_send_ack_phase(const struct i2c_efinix_sapphire_cfg *cfg, bool nack)
{
	uint32_t ack = (nack ? 1U : 0U) | EFX_I2C_TX_VALID | EFX_I2C_TX_ENABLE;

	i2c_reg_write(cfg, EFX_I2C_TX_ACK, ack);
	return i2c_wait_clear(cfg, EFX_I2C_TX_ACK, EFX_I2C_TX_VALID);
}

static int i2c_send_byte(const struct i2c_efinix_sapphire_cfg *cfg, uint8_t byte)
{
	uint32_t tx = (uint32_t)byte | EFX_I2C_TX_VALID | EFX_I2C_TX_ENABLE |
		      EFX_I2C_TX_DISABLE_ON_DATA_CONFLICT;
	int rc;

	i2c_reg_write(cfg, EFX_I2C_TX_DATA, tx);

	/*
	 * Match the bare-metal sequence: after every sent byte, drive the ACK
	 * phase via TX_ACK with value 1.
	 */
	rc = i2c_send_ack_phase(cfg, true);
	if (rc < 0) {
		return rc;
	}

	/* RX_ACK bit 0 low means ACK received. */
	if ((i2c_reg_read(cfg, EFX_I2C_RX_ACK) & EFX_I2C_VALUE_MASK) != 0U) {
		return -EIO;
	}

	return 0;
}

static int i2c_read_byte(const struct i2c_efinix_sapphire_cfg *cfg, bool last, uint8_t *val)
{
	uint32_t tx = 0xFFU | EFX_I2C_TX_VALID | EFX_I2C_TX_ENABLE |
		      EFX_I2C_TX_DISABLE_ON_DATA_CONFLICT;
	int rc;

	i2c_reg_write(cfg, EFX_I2C_TX_DATA, tx);
	rc = i2c_send_ack_phase(cfg, last);
	if (rc < 0) {
		return rc;
	}

	/*
	 * RX data is produced after the dummy TX and ACK/NACK phase complete.
	 * Ensure it became valid at least once before reading.
	 */
	rc = i2c_wait_set(cfg, EFX_I2C_RX_DATA, EFX_I2C_RX_VALID);
	if (rc < 0) {
		return rc;
	}

	*val = (uint8_t)(i2c_reg_read(cfg, EFX_I2C_RX_DATA) & EFX_I2C_VALUE_MASK);
	return 0;
}

static void i2c_apply_timing(const struct i2c_efinix_sapphire_cfg *cfg, uint32_t bus_hz)
{
	uint32_t half_period = MAX(1U, cfg->core_hz / (2U * bus_hz));
	uint32_t full_period = MAX(1U, cfg->core_hz / bus_hz);

	i2c_reg_write(cfg, EFX_I2C_SAMPLING_CLOCK_DIVIDER, EFX_I2C_DEFAULT_SAMPLING_DIV);
	i2c_reg_write(cfg, EFX_I2C_TIMEOUT, MAX(1U, cfg->core_hz / 1000U));
	i2c_reg_write(cfg, EFX_I2C_TSUDAT, MAX(1U, cfg->core_hz / 2000000U));
	i2c_reg_write(cfg, EFX_I2C_TLOW, half_period);
	i2c_reg_write(cfg, EFX_I2C_THIGH, half_period);
	i2c_reg_write(cfg, EFX_I2C_TBUF, full_period);
}

static int i2c_efinix_sapphire_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_efinix_sapphire_cfg *cfg = dev->config;
	struct i2c_efinix_sapphire_data *data = dev->data;
	uint32_t speed_hz;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0U) {
		return -ENOTSUP;
	}

	if ((dev_config & I2C_ADDR_10_BITS) != 0U) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		speed_hz = 100000U;
		break;
	case I2C_SPEED_FAST:
		speed_hz = 400000U;
		break;
	default:
		return -ENOTSUP;
	}

	i2c_apply_timing(cfg, speed_hz);
	data->dev_config = (dev_config & ~(I2C_SPEED_MASK)) | I2C_SPEED_SET(I2C_SPEED_GET(dev_config));

	return 0;
}

static int i2c_efinix_sapphire_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_efinix_sapphire_data *data = dev->data;

	if (dev_config == NULL) {
		return -EINVAL;
	}

	*dev_config = data->dev_config;
	return 0;
}

static int i2c_efinix_sapphire_transfer(const struct device *dev, struct i2c_msg *msgs,
					uint8_t num_msgs, uint16_t addr)
{
	const struct i2c_efinix_sapphire_cfg *cfg = dev->config;
	int rc = 0;

	if ((msgs == NULL) || (num_msgs == 0U)) {
		return -EINVAL;
	}

	for (uint8_t i = 0; i < num_msgs; i++) {
		struct i2c_msg *msg = &msgs[i];
		bool read = (msg->flags & I2C_MSG_READ) != 0U;
		bool need_start = (i == 0U) || ((msg->flags & I2C_MSG_RESTART) != 0U);

		if ((msg->flags & I2C_MSG_ADDR_10_BITS) != 0U) {
			return -ENOTSUP;
		}

		if (need_start) {
			rc = i2c_master_start(cfg);
			if (rc < 0) {
				goto out_stop;
			}

			rc = i2c_send_byte(cfg, (uint8_t)((addr << 1) | (read ? 1U : 0U)));
			if (rc < 0) {
				goto out_stop;
			}
		}

		if (read) {
			for (uint32_t b = 0; b < msg->len; b++) {
				rc = i2c_read_byte(cfg, b == (msg->len - 1U), &msg->buf[b]);
				if (rc < 0) {
					goto out_stop;
				}
			}
		} else {
			for (uint32_t b = 0; b < msg->len; b++) {
				rc = i2c_send_byte(cfg, msg->buf[b]);
				if (rc < 0) {
					goto out_stop;
				}
			}
		}

		if ((msg->flags & I2C_MSG_STOP) != 0U) {
			rc = i2c_master_stop(cfg);
			if (rc < 0) {
				return rc;
			}
		}
	}

	return 0;

out_stop:
	(void)i2c_master_stop(cfg);
	return rc;
}

static int i2c_efinix_sapphire_recover_bus(const struct device *dev)
{
	const struct i2c_efinix_sapphire_cfg *cfg = dev->config;

	for (int i = 0; i < 3; i++) {
		i2c_reg_write(cfg, EFX_I2C_MASTER_STATUS,
			      EFX_I2C_MASTER_RECOVER | EFX_I2C_MASTER_RECOVER_DROPPED);
		if (i2c_wait_clear(cfg, EFX_I2C_MASTER_STATUS, EFX_I2C_MASTER_RECOVER) < 0) {
			return -ETIMEDOUT;
		}

		if ((i2c_reg_read(cfg, EFX_I2C_MASTER_STATUS) & EFX_I2C_MASTER_RECOVER_DROPPED) == 0U) {
			return 0;
		}
	}

	return -EIO;
}

static int i2c_efinix_sapphire_init(const struct device *dev)
{
	const struct i2c_efinix_sapphire_cfg *cfg = dev->config;
	uint32_t dev_config = I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bus_hz);

	return i2c_efinix_sapphire_configure(dev, dev_config);
}

static DEVICE_API(i2c, i2c_efinix_sapphire_api) = {
	.configure = i2c_efinix_sapphire_configure,
	.get_config = i2c_efinix_sapphire_get_config,
	.transfer = i2c_efinix_sapphire_transfer,
	.recover_bus = i2c_efinix_sapphire_recover_bus,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

#define I2C_EFINIX_SAPPHIRE_INIT(n)                                                            \
	static const struct i2c_efinix_sapphire_cfg i2c_efinix_sapphire_cfg_##n = {            \
		.base = DT_INST_REG_ADDR(n),                                                     \
		.core_hz = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,                                   \
		.bus_hz = DT_INST_PROP_OR(n, clock_frequency, 100000U),                          \
	};                                                                                     \
	static struct i2c_efinix_sapphire_data i2c_efinix_sapphire_data_##n;                  \
	I2C_DEVICE_DT_INST_DEFINE(n,                                                            \
				  i2c_efinix_sapphire_init,                                    \
				  NULL,                                                       \
				  &i2c_efinix_sapphire_data_##n,                              \
				  &i2c_efinix_sapphire_cfg_##n,                               \
				  POST_KERNEL,                                                \
				  CONFIG_I2C_INIT_PRIORITY,                                   \
				  &i2c_efinix_sapphire_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_EFINIX_SAPPHIRE_INIT)
