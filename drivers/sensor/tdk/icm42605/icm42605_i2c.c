/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include "icm42605_i2c.h"

LOG_MODULE_DECLARE(ICM42605, CONFIG_SENSOR_LOG_LEVEL);

int inv_single_write(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *data)
{
	return i2c_reg_write_byte_dt(spec, reg, *data);
}

int inv_read(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *data, size_t len)
{
	return i2c_write_read(spec->bus, spec->addr, &reg, 1, data, len);
}
