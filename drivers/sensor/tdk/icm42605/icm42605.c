/*
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_ICM42605_SPI
#include <zephyr/drivers/spi.h>
#else
#include <zephyr/drivers/i2c.h>
#endif
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "icm42605.h"
#include "icm42605_reg.h"
#include "icm42605_setup.h"
#ifdef CONFIG_ICM42605_SPI
#include "icm42605_spi.h"
#else
#include "icm42605_i2c.h"
#endif

LOG_MODULE_REGISTER(ICM42605, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t icm42605_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};

/* see "Accelerometer Measurements" section from register map description */
static void icm42605_convert_accel(struct sensor_value *val,
				   int16_t raw_val,
				   uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void icm42605_convert_gyro(struct sensor_value *val,
				  int16_t raw_val,
				  uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void icm42605_convert_temp(struct sensor_value *val,
					 int16_t raw_val)
{
	val->val1 = (((int64_t)raw_val * 100) / 207) + 25;
	val->val2 = ((((int64_t)raw_val * 100) % 207) * 1000000) / 207;

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int icm42605_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	const struct icm42605_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm42605_convert_accel(val, drv_data->accel_x,
				       drv_data->accel_sensitivity_shift);
		icm42605_convert_accel(val + 1, drv_data->accel_y,
				       drv_data->accel_sensitivity_shift);
		icm42605_convert_accel(val + 2, drv_data->accel_z,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm42605_convert_accel(val, drv_data->accel_x,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm42605_convert_accel(val, drv_data->accel_y,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm42605_convert_accel(val, drv_data->accel_z,
				       drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm42605_convert_gyro(val, drv_data->gyro_x,
				      drv_data->gyro_sensitivity_x10);
		icm42605_convert_gyro(val + 1, drv_data->gyro_y,
				      drv_data->gyro_sensitivity_x10);
		icm42605_convert_gyro(val + 2, drv_data->gyro_z,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm42605_convert_gyro(val, drv_data->gyro_x,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm42605_convert_gyro(val, drv_data->gyro_y,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm42605_convert_gyro(val, drv_data->gyro_z,
				      drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm42605_convert_temp(val, drv_data->temp);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

int icm42605_tap_fetch(const struct device *dev)
{
	int result = 0;
	struct icm42605_data *drv_data = dev->data;
	const struct icm42605_config *cfg = dev->config;

	if (drv_data->tap_en &&
	    (drv_data->tap_handler || drv_data->double_tap_handler)) {
		result = inv_read(&cfg->spec, REG_INT_STATUS3, drv_data->fifo_data, 1);
		if (drv_data->fifo_data[0] & BIT_INT_STATUS_TAP_DET) {
			result = inv_read(&cfg->spec, REG_APEX_DATA4,
					      drv_data->fifo_data, 1);
			if (drv_data->fifo_data[0] & APEX_TAP) {
				if (drv_data->tap_trigger->type ==
				    SENSOR_TRIG_TAP) {
					if (drv_data->tap_handler) {
						LOG_DBG("Single Tap detected");
						drv_data->tap_handler(dev
						      , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else if (drv_data->fifo_data[0] & APEX_DOUBLE_TAP) {
				if (drv_data->double_tap_trigger->type ==
				    SENSOR_TRIG_DOUBLE_TAP) {
					if (drv_data->double_tap_handler) {
						LOG_DBG("Double Tap detected");
						drv_data->double_tap_handler(dev
						     , drv_data->tap_trigger);
					}
				} else {
					LOG_ERR("Trigger type is mismatched");
				}
			} else {
				LOG_DBG("Not supported tap event");
			}
		}
	}

	return 0;
}

static int icm42605_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	int result = 0;
	uint16_t fifo_count = 0;
	struct icm42605_data *drv_data = dev->data;
	const struct icm42605_config *cfg = dev->config;

	/* Read INT_STATUS (0x45) and FIFO_COUNTH(0x46), FIFO_COUNTL(0x47) */
	result = inv_read(&cfg->spec, REG_INT_STATUS, drv_data->fifo_data, 3);

	if (drv_data->fifo_data[0] & BIT_INT_STATUS_DRDY) {
		fifo_count = (drv_data->fifo_data[1] << 8)
			+ (drv_data->fifo_data[2]);
		result = inv_read(&cfg->spec, REG_FIFO_DATA, drv_data->fifo_data,
				      fifo_count);

		/* FIFO Data structure
		 * Packet 1 : FIFO Header(1), AccelX(2), AccelY(2),
		 *            AccelZ(2), Temperature(1)
		 * Packet 2 : FIFO Header(1), GyroX(2), GyroY(2),
		 *            GyroZ(2), Temperature(1)
		 * Packet 3 : FIFO Header(1), AccelX(2), AccelY(2), AccelZ(2),
		 *            GyroX(2), GyroY(2), GyroZ(2), Temperature(1)
		 */
		if (drv_data->fifo_data[0] & BIT_FIFO_HEAD_ACCEL) {
			/* Check empty values */
			if (!(drv_data->fifo_data[1] == FIFO_ACCEL0_RESET_VALUE
			      && drv_data->fifo_data[2] ==
			      FIFO_ACCEL1_RESET_VALUE)) {
				drv_data->accel_x =
					(drv_data->fifo_data[1] << 8)
					+ (drv_data->fifo_data[2]);
				drv_data->accel_y =
					(drv_data->fifo_data[3] << 8)
					+ (drv_data->fifo_data[4]);
				drv_data->accel_z =
					(drv_data->fifo_data[5] << 8)
					+ (drv_data->fifo_data[6]);
			}
			if (!(drv_data->fifo_data[0] & BIT_FIFO_HEAD_GYRO)) {
				drv_data->temp =
					(int16_t)(drv_data->fifo_data[7]);
			} else {
				if (!(drv_data->fifo_data[7] ==
				      FIFO_GYRO0_RESET_VALUE &&
				      drv_data->fifo_data[8] ==
				      FIFO_GYRO1_RESET_VALUE)) {
					drv_data->gyro_x =
						(drv_data->fifo_data[7] << 8)
						+ (drv_data->fifo_data[8]);
					drv_data->gyro_y =
						(drv_data->fifo_data[9] << 8)
						+ (drv_data->fifo_data[10]);
					drv_data->gyro_z =
						(drv_data->fifo_data[11] << 8)
						+ (drv_data->fifo_data[12]);
				}
				drv_data->temp =
					(int16_t)(drv_data->fifo_data[13]);
			}
		} else {
			if (drv_data->fifo_data[0] & BIT_FIFO_HEAD_GYRO) {
				if (!(drv_data->fifo_data[1] ==
				      FIFO_GYRO0_RESET_VALUE &&
				      drv_data->fifo_data[2] ==
				      FIFO_GYRO1_RESET_VALUE)) {
					drv_data->gyro_x =
						(drv_data->fifo_data[1] << 8)
						+ (drv_data->fifo_data[2]);
					drv_data->gyro_y =
						(drv_data->fifo_data[3] << 8)
						+ (drv_data->fifo_data[4]);
					drv_data->gyro_z =
						(drv_data->fifo_data[5] << 8)
						+ (drv_data->fifo_data[6]);
				}
				drv_data->temp =
					(int16_t)(drv_data->fifo_data[7]);
			}
		}
	}

	return 0;
}

static int icm42605_attr_set(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	struct icm42605_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 8000 || val->val1 < 1) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				drv_data->accel_hz = val->val1;
			}
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 < ACCEL_FS_16G ||
			    val->val1 > ACCEL_FS_2G) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				drv_data->accel_sf = val->val1;
			}
		} else {
			LOG_ERR("Not supported ATTR");
			return -ENOTSUP;
		}

		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			if (val->val1 > 8000 || val->val1 < 12) {
				LOG_ERR("Incorrect sampling value");
				return -EINVAL;
			} else {
				drv_data->gyro_hz = val->val1;
			}
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			if (val->val1 < GYRO_FS_2000DPS ||
			    val->val1 > GYRO_FS_15DPS) {
				LOG_ERR("Incorrect fullscale value");
				return -EINVAL;
			} else {
				drv_data->gyro_sf = val->val1;
			}
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}
		break;
	case SENSOR_CHAN_ALL:
		if (attr == SENSOR_ATTR_ALERT /* message handler */) {
			switch (val->val1) {
				case MSG_SLEEP:
					icm42605_set_odr(dev, 25, 0);
					/**
					 * Set Accel to Low Power mode (Register 0x4Eh in Bank 0)
					 * ACCEL_MODE = 2 and (Register 0x4Dh in Bank 0), ACCEL_LP_CLK_SEL = 0, for low power mode

					 * Set DMP ODR (Register 0x56h in Bank 0)
					 * DMP_ODR = 0 for 25 Hz, 2 for 50 Hz

					 * Wait 1 millisecond

						 1. Set DMP_MEM_RESET_EN to 1 (Register 0x4Bh in Bank 0)
						2. Wait 1 millisecond
						3. Set SLEEP_TIME_OUT (Register 0x43h in Bank 4) if default value does not meet needs
						4. Wait 1 millisecond
						5. Set MOUNTING_MATRIX (Register 0x44h in Bank 4) if default value does not meet needs
						6. Wait 1 millisecond
						7. Set SLEEP_GESTURE_DELAY (Register 0x45h in Bank 4) if default value does not meet needs
						8. Wait 1 millisecond
						9. Set DMP_INIT_EN to 1 (Register 0x4Bh in Bank 0)
						10. Enable Raise to Wake/Sleep, source for INT1 by setting bit 2,1 in register INT_SOURCE6 (Register 0x4Dh in Bank 4)
						to 1. Or if INT2 is selected for Raise to Wake/Sleep, enable Raise to Wake/Sleep source by setting bit 2,1 in register
						INT_SOURCE7 (Register 0x4Eh in Bank 4) to 1.
						11. Wait 50 milliseconds
						12. Turn on Raise to Wake/Sleep feature by setting R2W_EN to 1 (Register 0x56h in Bank 0)

						1. Read interrupt register (Register 0x38h in Bank 0) for Wake and Sleep event
					 */
					break;
				default:
					LOG_ERR("Alert msg not support");
					return -EINVAL;
			}
		}
	default:
		LOG_ERR("Not support");
		return -EINVAL;
	}

	return 0;
}

static int icm42605_attr_get(const struct device *dev,
			     enum sensor_channel chan,
			     enum sensor_attribute attr,
			     struct sensor_value *val)
{
	const struct icm42605_data *drv_data = dev->data;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = drv_data->accel_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			val->val1 = drv_data->accel_sf;
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}

		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			val->val1 = drv_data->gyro_hz;
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			val->val1 = drv_data->gyro_sf;
		} else {
			LOG_ERR("Not supported ATTR");
			return -EINVAL;
		}

		break;

	default:
		LOG_ERR("Not support");
		return -EINVAL;
	}

	return 0;
}

static int icm42605_data_init(struct icm42605_data *data,
			      const struct icm42605_config *cfg)
{
	data->accel_x = 0;
	data->accel_y = 0;
	data->accel_z = 0;
	data->temp = 0;
	data->gyro_x = 0;
	data->gyro_y = 0;
	data->gyro_z = 0;
	data->accel_hz = cfg->accel_hz;
	data->gyro_hz = cfg->gyro_hz;

	data->accel_sf = cfg->accel_fs;
	data->gyro_sf = cfg->gyro_fs;

	data->tap_en = false;
	data->sensor_started = false;

	return 0;
}


static int icm42605_init(const struct device *dev)
{
	struct icm42605_data *drv_data = dev->data;
	const struct icm42605_config *cfg = dev->config;

#ifdef CONFIG_ICM42605_SPI
	if (!spi_is_ready_dt(&cfg->spec)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}
#else
	if (!i2c_is_ready_dt(&cfg->spec)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}
#endif

	icm42605_data_init(drv_data, cfg);
	icm42605_sensor_init(dev);

	drv_data->accel_sensitivity_shift = 14 - 3;
	drv_data->gyro_sensitivity_x10 = icm42605_gyro_sensitivity_x10[3];

#ifdef CONFIG_ICM42605_TRIGGER
	if (icm42605_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return -EIO;
	}
#endif

	LOG_DBG("Initialize interrupt done");

	return 0;
}

static const struct sensor_driver_api icm42605_driver_api = {
#ifdef CONFIG_ICM42605_TRIGGER
	.trigger_set = icm42605_trigger_set,
#endif
	.sample_fetch = icm42605_sample_fetch,
	.channel_get = icm42605_channel_get,
	.attr_set = icm42605_attr_set,
	.attr_get = icm42605_attr_get,
};

#if defined(CONFIG_ICM42605_SPI)
#define DT_SPEC_CONFIG(index) \
	SPI_DT_SPEC_INST_GET(index,                          \
				    SPI_OP_MODE_MASTER |           \
				    SPI_MODE_CPOL |                \
				    SPI_MODE_CPHA |                \
				    SPI_WORD_SET(8) |              \
				    SPI_TRANSFER_MSB,              \
				    0U)
#else
#define DT_SPEC_CONFIG(index) \
	I2C_DT_SPEC_INST_GET(index)
#endif

#define ICM42605_DEFINE_CONFIG(index)					\
	static const struct icm42605_config icm42605_cfg_##index = {	\
		.spec = DT_SPEC_CONFIG(index),		\
		.gpio_int = GPIO_DT_SPEC_INST_GET(index, int_gpios),    \
		.accel_hz = DT_INST_PROP(index, accel_hz),		\
		.gyro_hz = DT_INST_PROP(index, gyro_hz),		\
		.accel_fs = DT_INST_ENUM_IDX(index, accel_fs),		\
		.gyro_fs = DT_INST_ENUM_IDX(index, gyro_fs),		\
	}

#define ICM42605_INIT(index)						\
	ICM42605_DEFINE_CONFIG(index);					\
	static struct icm42605_data icm42605_driver_##index;		\
	SENSOR_DEVICE_DT_INST_DEFINE(index, icm42605_init,		\
			    NULL,					\
			    &icm42605_driver_##index,			\
			    &icm42605_cfg_##index, POST_KERNEL,		\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &icm42605_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ICM42605_INIT)
