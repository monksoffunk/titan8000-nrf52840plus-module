# Copyright (c) 2025 monksoffunk
# SPDX-License-Identifier: MIT

# Amend the existing kscan drivers library to add an extra driver.
zephyr_library_amend()

# Ensure we can include ZMK headers like <zmk/debounce.h>.
zephyr_include_directories(${CMAKE_SOURCE_DIR}/include)

zephyr_library_sources(kscan_gpio_charlieplex_titan8000.c)
