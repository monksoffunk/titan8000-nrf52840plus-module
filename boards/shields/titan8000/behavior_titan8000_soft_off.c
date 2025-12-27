/*
 * Custom soft-off request behavior for Titan8000
 *
 * This behavior does NOT power off the device directly.
 * It only requests soft-off, which will be executed
 * by the custom charlieplex kscan driver at a safe time.
 */
#define DT_DRV_COMPAT zmk_behavior_titan8000_soft_off

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

//#include <zmk/event_manager.h>
#include <zmk/pm.h>
#include <zmk/behavior.h>

LOG_MODULE_REGISTER(titan8000_soft_off, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

void titan8000_charlieplex_request_soft_off(void);

struct behavior_titan8000_soft_off_config {
    bool split_peripheral_turn_off_on_press;
    uint32_t hold_time_ms;
};

struct behavior_titan8000_soft_off_data {
    bool pressed;
//    uint32_t press_start;
    struct k_work_delayable hold_work;
};

#define IS_SPLIT_PERIPHERAL                                                                        \
    (IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))

static void titan8000_soft_off_hold_handler(struct k_work *work) {
    struct k_work_delayable *dwork =
        CONTAINER_OF(work, struct k_work_delayable, work);
    struct behavior_titan8000_soft_off_data *data =
        CONTAINER_OF(dwork, struct behavior_titan8000_soft_off_data, hold_work);

    if (!data->pressed) {
        return; // released before timeout → cancel
    }

    titan8000_charlieplex_request_soft_off();
}

static int behavior_titan8000_soft_off_binding_pressed(struct zmk_behavior_binding *binding,
                                                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_titan8000_soft_off_data *data = dev->data;
    const struct behavior_titan8000_soft_off_config *config = dev->config;

    if (IS_SPLIT_PERIPHERAL && config->split_peripheral_turn_off_on_press) {
        titan8000_charlieplex_request_soft_off();
    } else {
        data->pressed = true;
//        data->press_start = k_uptime_get();
        if (config->hold_time_ms > 0) {
            k_work_schedule(&data->hold_work, K_MSEC(config->hold_time_ms));
        }
    }

    return ZMK_BEHAVIOR_OPAQUE; /* prevent further processing */
}

static int behavior_titan8000_soft_off_binding_released(struct zmk_behavior_binding *binding,
                                                        struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_titan8000_soft_off_data *data = dev->data;

    data->pressed = false;
    k_work_cancel_delayable(&data->hold_work);
    /*
    const struct behavior_titan8000_soft_off_config *config = dev->config;

    if (config->hold_time_ms == 0) {
        LOG_DBG("No hold time set, triggering soft off");
        titan8000_charlieplex_request_soft_off();
    } else {
        uint32_t hold_time = k_uptime_get() - data->press_start;

        if (hold_time > config->hold_time_ms) {
            if (IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)) {
                k_sleep(K_MSEC(100));
            }
            titan8000_charlieplex_request_soft_off();
        } else {
            LOG_INF("Not triggering soft off: held for %d and hold time is %d", hold_time,
                    config->hold_time_ms);
        }
    }
    */

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_titan8000_soft_off_driver_api = {
    .binding_pressed = behavior_titan8000_soft_off_binding_pressed,
    .binding_released = behavior_titan8000_soft_off_binding_released,
    .locality = BEHAVIOR_LOCALITY_GLOBAL,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

struct behavior_titan8000_soft_off_data data;
struct behavior_titan8000_soft_off_config config = {
    .hold_time_ms = DT_INST_PROP_OR(0, hold_time_ms, 0),                                       \
    .split_peripheral_turn_off_on_press =                                                      \
        DT_INST_PROP_OR(n, split_peripheral_off_on_press, false),                              \
};

static int behavior_titan8000_soft_off_init(const struct device *dev) {
    struct behavior_titan8000_soft_off_data *data = dev->data;

    data->pressed = false;
    k_work_init_delayable(&data->hold_work, titan8000_soft_off_hold_handler);

    return 0;
}

BEHAVIOR_DT_INST_DEFINE(0,
		behavior_titan8000_soft_off_init,
        NULL,
		&data,
        &config,
		POST_KERNEL,
        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	
		&behavior_titan8000_soft_off_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
