# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(vibration_monitor)

target_sources(app PRIVATE
    src/main.c
    src/ble.c
    src/sensor_adxl343.c
    src/sensor_bme280.c
    src/fsm.c
    # src/sensor_piezo.c    # uncomment when hardware ready
    # src/analysis.c        # uncomment when on-device analysis needed
)
