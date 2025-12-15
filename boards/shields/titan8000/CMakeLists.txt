zephyr_library()

zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

# Buzzer toggle behavior is always compiled (no-op on boards without buzzer)
zephyr_library_sources(behavior_buzzer_toggle.c)

# Only build buzzer driver for boards with PWM capability
if(CONFIG_BOARD_SEEEDUINO_XIAO_BLE)
  zephyr_library_sources(buzzer.c)
endif()