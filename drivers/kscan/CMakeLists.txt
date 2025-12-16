# Only compile custom charlieplex driver if devicetree node exists
if(CONFIG_KSCAN)
  zephyr_library()
  zephyr_library_sources_ifdef(DT_HAS_TITAN8000_KSCAN_GPIO_CHARLIEPLEX_ENABLED kscan_gpio_charlieplex_custom.c)
endif()
