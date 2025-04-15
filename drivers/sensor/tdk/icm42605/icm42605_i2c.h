/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM42605_ICM42605_SPI_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM42605_ICM42605_SPI_H_

#define DT_DRV_COMPAT invensense_icm42605_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

int inv_single_write(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *data);
int inv_read(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *data, size_t len);

#endif /* __SENSOR_ICM42605_ICM42605_SPI__ */
