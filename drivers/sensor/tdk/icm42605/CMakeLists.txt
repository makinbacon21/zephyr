# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(icm42605.c)
zephyr_library_sources(icm42605_setup.c)
zephyr_library_sources_ifdef(CONFIG_ICM42605_SPI icm42605_spi.c)
zephyr_library_sources_ifdef(CONFIG_ICM42605_I2C icm42605_i2c.c)
zephyr_library_sources_ifdef(CONFIG_ICM42605_TRIGGER icm42605_trigger.c)
