zephyr_library()

zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

# Custom charlieplex kscan driver
zephyr_library_sources(kscan_gpio_charlieplex_custom.c)

# Buzzer components - only compile when CONFIG_TITAN8000_BUZZER is enabled
if(CONFIG_TITAN8000_BUZZER)
  zephyr_library_sources(buzzer.c)
endif()

# Buzzer toggle behavior - always compile (no-op when buzzer disabled)
zephyr_library_sources(behavior_buzzer_toggle.c)