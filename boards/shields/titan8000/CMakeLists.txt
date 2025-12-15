zephyr_library()

zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

# Buzzer components (conditional compilation via DT_NODE_EXISTS in source files)
zephyr_library_sources(
  buzzer.c
  behavior_buzzer_toggle.c
)