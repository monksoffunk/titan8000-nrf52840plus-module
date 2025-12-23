/*
 * Copyright (c) 2025 The ZMK Contributors
 * Copyright (c) 2025 monksoffunk
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_buzzer

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_TITAN8000_BUZZER
#include "buzzer.h"
#include "behavior_buzzer.h"
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
#ifdef CONFIG_TITAN8000_BUZZER
    int key = binding->param1;

    switch (key) {
        case TOG_KEYPRESS_BUZZER:
            buzzer_toggle_keypress_beep();
            break;
        case TOG_ALL_BUZZER:
            buzzer_toggle_enable();
            break;
        default:
            LOG_ERR("Invalid key: %d", key);
            return -EINVAL;
    }
#endif
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_parameter_value_metadata buzzer_param_values[] = {
	{
        .display_name = "Toggle keypress buzzer",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
		.value = TOG_KEYPRESS_BUZZER,
    },
	{
        .display_name = "Toggle Buzzer",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
		.value = TOG_ALL_BUZZER,
    },
};

static const struct behavior_parameter_metadata_set buzzer_param_metadata_set[] = {{
    .param1_values = buzzer_param_values,
    .param1_values_len = ARRAY_SIZE(buzzer_param_values),
}};

static const struct behavior_parameter_metadata buzzer_metadata = {
    .sets_len = ARRAY_SIZE(buzzer_param_metadata_set),
    .sets = buzzer_param_metadata_set,
};

static const struct behavior_driver_api behavior_buzzer_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
	.parameter_metadata = &buzzer_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_buzzer_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
