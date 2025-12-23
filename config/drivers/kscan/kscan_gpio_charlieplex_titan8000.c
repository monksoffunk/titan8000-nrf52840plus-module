/*
 * Copyright (c) 2020-2023 The ZMK Contributors
 * Copyright (c) 2025 monksoffunk
 *
 * SPDX-License-Identifier: MIT
 */

#include <zmk/debounce.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/poweroff.h>

#if IS_ENABLED(CONFIG_ZMK_PM_SOFT_OFF)
//#if IS_ENABLED(CONFIG_PM_DEVICE)
#include <zephyr/sys/atomic.h>
static atomic_t soft_off_pending;
static const struct device *titan8000_charlieplex_dev;
void titan8000_play_soft_off_tone(void);
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT titan8000_kscan_gpio_charlieplex

#define INST_LEN(n) DT_INST_PROP_LEN(n, gpios)
#define INST_CHARLIEPLEX_LEN(n) (INST_LEN(n) * INST_LEN(n))

#if CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS >= 0
#define INST_DEBOUNCE_PRESS_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_PRESS_MS
#else
#define INST_DEBOUNCE_PRESS_MS(n)                                                                  \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_press_ms))
#endif

#if CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS >= 0
#define INST_DEBOUNCE_RELEASE_MS(n) CONFIG_ZMK_KSCAN_DEBOUNCE_RELEASE_MS
#else
#define INST_DEBOUNCE_RELEASE_MS(n)                                                                \
    DT_INST_PROP_OR(n, debounce_period, DT_INST_PROP(n, debounce_release_ms))
#endif

#define INST_DISCHARGE_US(n) DT_INST_PROP_OR(n, discharge_before_inputs_us, 0)

/* Match ZMK v0.3-branch upstream: LISTIFY() expects fn(idx, inst_idx). */
#define KSCAN_GPIO_CFG_INIT(idx, inst_idx)                                                         \
    GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst_idx), gpios, idx)

#define INST_INTR_DEFINED(n) DT_INST_NODE_HAS_PROP(n, interrupt_gpios)

#define WITH_INTR(n) COND_CODE_1(INST_INTR_DEFINED(n), (+1), (+0))
#define WITHOUT_INTR(n) COND_CODE_0(INST_INTR_DEFINED(n), (+1), (+0))

#define USES_POLLING DT_INST_FOREACH_STATUS_OKAY(WITHOUT_INTR) > 0
#define USES_INTERRUPT DT_INST_FOREACH_STATUS_OKAY(WITH_INTR) > 0

#define COND_ANY_POLLING(code) COND_CODE_1(USES_POLLING, code, ())
#define COND_THIS_INTERRUPT(n, code) COND_CODE_1(INST_INTR_DEFINED(n), code, ())

#define KSCAN_INTR_CFG_INIT(inst_idx) GPIO_DT_SPEC_GET(DT_DRV_INST(inst_idx), interrupt_gpios)

struct kscan_charlieplex_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    int64_t scan_time; /* Timestamp of the current or scheduled scan. */
    struct gpio_callback irq_callback;
    /**
     * Current state of the matrix as a flattened 2D array of length
     * (config->cells.length ^2)
     */
    struct zmk_debounce_state *charlieplex_state;
};

struct kscan_gpio_list {
    const struct gpio_dt_spec *gpios;
    size_t len;
};

/** Define a kscan_gpio_list from a compile-time GPIO array. */
#define KSCAN_GPIO_LIST(gpio_array)                                                                \
    ((struct kscan_gpio_list){.gpios = gpio_array, .len = ARRAY_SIZE(gpio_array)})

struct kscan_charlieplex_config {
    struct kscan_gpio_list cells;
    struct zmk_debounce_config debounce_config;
    int32_t debounce_scan_period_ms;
    int32_t poll_period_ms;
    bool use_interrupt;
    const struct gpio_dt_spec interrupt;
    int32_t discharge_before_inputs_us;
};

/**
 * Get the index into a matrix state array from a row and column.
 * There are effectively (n) cols and (n-1) rows, but we use the full col x row space
 * as a safety measure against someone accidentally defining a transform RC at (p,p)
 */
static int state_index(const struct kscan_charlieplex_config *config, const int row,
                       const int col) {
    __ASSERT(row < config->cells.len, "Invalid row %i", row);
    __ASSERT(col < config->cells.len, "Invalid column %i", col);
    __ASSERT(col != row, "Invalid column row pair %i, %i", col, row);

    return (col * config->cells.len) + row;
}

static int kscan_charlieplex_set_as_input(const struct gpio_dt_spec *gpio) {
    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->port->name);
        return -ENODEV;
    }

    const gpio_flags_t pull_flag =
        ((gpio->dt_flags & GPIO_ACTIVE_LOW) == GPIO_ACTIVE_LOW) ? GPIO_PULL_UP : GPIO_PULL_DOWN;
    int err = gpio_pin_configure_dt(gpio, GPIO_INPUT | pull_flag);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for input", gpio->pin, gpio->port->name);
        return err;
    }
    return 0;
}

static int kscan_charlieplex_set_as_output(const struct gpio_dt_spec *gpio) {
    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(gpio, GPIO_OUTPUT);
    if (err) {
        LOG_ERR("Unable to configure pin %u on %s for output", gpio->pin, gpio->port->name);
        return err;
    }

    err = gpio_pin_set_dt(gpio, 1);
    if (err) {
        LOG_ERR("Failed to set output pin %u active: %i", gpio->pin, err);
    }
    return err;
}

static int kscan_charlieplex_set_all_as_input(const struct device *dev) {
    const struct kscan_charlieplex_config *config = dev->config;

    for (int i = 0; i < config->cells.len; i++) {
        int err = kscan_charlieplex_set_as_input(&config->cells.gpios[i]);
        if (err) {
            return err;
        }
    }
    return 0;
}

static int kscan_charlieplex_set_all_outputs(const struct device *dev, const int value) {
    const struct kscan_charlieplex_config *config = dev->config;

    for (int i = 0; i < config->cells.len; i++) {
        const struct gpio_dt_spec *gpio = &config->cells.gpios[i];

        int err = gpio_pin_configure_dt(gpio, GPIO_OUTPUT);
        if (err) {
            LOG_ERR("Unable to configure pin %u on %s for output", gpio->pin, gpio->port->name);
            return err;
        }

        err = gpio_pin_set_dt(gpio, value);
        if (err) {
            LOG_ERR("Failed to set output %i to %i: %i", i, value, err);
            return err;
        }
    }

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_charlieplex_disconnect_all(const struct device *dev) {
    const struct kscan_charlieplex_config *config = dev->config;

    for (int i = 0; i < config->cells.len; i++) {
        const struct gpio_dt_spec *gpio = &config->cells.gpios[i];

        int err = gpio_pin_configure_dt(gpio, GPIO_DISCONNECTED);
        if (err) {
            LOG_ERR("Unable to configure pin %u on %s for input", gpio->pin, gpio->port->name);
            return err;
        }
    }

    return 0;
}

#endif

static int kscan_charlieplex_interrupt_configure(const struct device *dev,
                                                const gpio_flags_t flags) {
    const struct kscan_charlieplex_config *config = dev->config;
    const struct gpio_dt_spec *gpio = &config->interrupt;

    int err = gpio_pin_interrupt_configure_dt(gpio, flags);
    if (err) {
        LOG_ERR("Unable to configure interrupt for pin %u on %s", gpio->pin, gpio->port->name);
        return err;
    }

    return 0;
}

static int kscan_charlieplex_interrupt_line_drive_high(const struct device *dev) {
    const struct kscan_charlieplex_config *config = dev->config;
    const struct gpio_dt_spec *gpio = &config->interrupt;

    if (!device_is_ready(gpio->port)) {
        LOG_ERR("GPIO is not ready: %s", gpio->port->name);
        return -ENODEV;
    }

    int err = gpio_pin_configure(gpio->port, gpio->pin, GPIO_OUTPUT_HIGH);
    if (err) {
        LOG_ERR("Unable to configure interrupt pin %u on %s for output high", gpio->pin,
                gpio->port->name);
        return err;
    }

    return 0;
}

static int kscan_charlieplex_interrupt_line_input_pulldown(const struct device *dev) {
    const struct kscan_charlieplex_config *config = dev->config;
    return kscan_charlieplex_set_as_input(&config->interrupt);
}

static int kscan_charlieplex_interrupt_enable(const struct device *dev) {
    /* Restore the interrupt line (D10) to input w/ pulldown before enabling the level IRQ. */
    int err = kscan_charlieplex_interrupt_line_input_pulldown(dev);
    if (err) {
        return err;
    }

    err = kscan_charlieplex_interrupt_configure(dev, GPIO_INT_LEVEL_ACTIVE);
    if (err) {
        return err;
    }

    /*
     * While waiting for interrupts, drive all matrix lines to physical HIGH.
     * For ACTIVE_LOW pins, gpio_pin_set_dt(..., 0) drives the line HIGH.
     * This matches the "D0-D9 high, IRQ on D10 active high" wiring model.
     */
    return kscan_charlieplex_set_all_outputs(dev, 0);
}

static void kscan_charlieplex_irq_callback(const struct device *port, struct gpio_callback *cb,
                                          const gpio_port_pins_t _pin) {
    struct kscan_charlieplex_data *data =
        CONTAINER_OF(cb, struct kscan_charlieplex_data, irq_callback);

    /* Disable our interrupt to avoid re-entry while we scan. */
    kscan_charlieplex_interrupt_configure(data->dev, GPIO_INT_DISABLE);

    /*
     * Once the IRQ fires, drive the interrupt line high so its pulldown does not load the
     * charlieplex network during scanning (can cause ghost/false readings).
     */
    (void)kscan_charlieplex_interrupt_line_drive_high(data->dev);

    /* Release the idle drive as soon as we get the IRQ to minimize transients. */
    (void)kscan_charlieplex_set_all_as_input(data->dev);

    data->scan_time = k_uptime_get();
    k_work_reschedule(&data->work, K_NO_WAIT);
}

static void kscan_charlieplex_read_continue(const struct device *dev) {
    const struct kscan_charlieplex_config *config = dev->config;
    struct kscan_charlieplex_data *data = dev->data;

    data->scan_time += config->debounce_scan_period_ms;
    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static void kscan_charlieplex_read_end(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;
    const struct kscan_charlieplex_config *config = dev->config;

    if (config->use_interrupt) {
        /* Return to waiting for an interrupt. */
        kscan_charlieplex_interrupt_enable(dev);

#if IS_ENABLED(CONFIG_ZMK_PM_SOFT_OFF)
//#if IS_ENABLED(CONFIG_PM_DEVICE)
        if (atomic_cas(&soft_off_pending, 1, 0)) {
            sys_poweroff();
        }
#endif
    } else {
        data->scan_time += config->poll_period_ms;

        /* Return to polling slowly. */
        k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
    }
}

static int kscan_charlieplex_read(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;
    const struct kscan_charlieplex_config *config = dev->config;
    bool continue_scan = false;

    /*
     * RR vs MATRIX: set all pins as input, in case there was a failure on a
     * previous scan, and one of the pins is still set as output
     */
    int err = kscan_charlieplex_set_all_as_input(dev);
    if (err) {
        return err;
    }

    /*
     * Inverted scan (custom): choose one C (column) pin, drive it ACTIVE (expected LOW via
     * GPIO_ACTIVE_LOW), read all other pins as inputs w/ pull-ups.
     */
    for (int col = 0; col < config->cells.len; col++) {
        if (config->discharge_before_inputs_us > 0) {
            /* Drive all pins active (LOW) briefly to discharge floating capacitances. */
            err = kscan_charlieplex_set_all_outputs(dev, 1);
            if (err) {
                return err;
            }

            k_busy_wait(config->discharge_before_inputs_us);

            err = kscan_charlieplex_set_all_as_input(dev);
            if (err) {
                return err;
            }
        }

        const struct gpio_dt_spec *out_gpio = &config->cells.gpios[col];
        err = kscan_charlieplex_set_as_output(out_gpio);
        if (err) {
            return err;
        }

#if CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BEFORE_INPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BEFORE_INPUTS);
#endif

        for (int row = 0; row < config->cells.len; row++) {
            if (row == col) {
                continue;
            }

            const struct gpio_dt_spec *in_gpio = &config->cells.gpios[row];
            const int index = state_index(config, row, col);

            struct zmk_debounce_state *state = &data->charlieplex_state[index];
            /* gpio_pin_get_dt() returns 1 when pin is in its ACTIVE state. */
            zmk_debounce_update(state, gpio_pin_get_dt(in_gpio), config->debounce_scan_period_ms,
                                &config->debounce_config);

            /*
             * RR vs MATRIX: because we don't need an input/output => row/column
             * setup, we can update in the same loop.
             */
            if (zmk_debounce_get_changed(state)) {
                const bool pressed = zmk_debounce_is_pressed(state);

                LOG_DBG("Sending event at %i,%i state %s", row, col, pressed ? "on" : "off");

                data->callback(dev, row, col, pressed);
            }

            continue_scan = continue_scan || zmk_debounce_is_active(state);
        }

        err = kscan_charlieplex_set_as_input(out_gpio);
        if (err) {
            return err;
        }

#if CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BETWEEN_OUTPUTS > 0
        k_busy_wait(CONFIG_ZMK_KSCAN_CHARLIEPLEX_WAIT_BETWEEN_OUTPUTS);
#endif
    }

    if (continue_scan) {
        kscan_charlieplex_read_continue(dev);
    } else {
        kscan_charlieplex_read_end(dev);
    }

    return 0;
}

static void kscan_charlieplex_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct kscan_charlieplex_data *data = CONTAINER_OF(dwork, struct kscan_charlieplex_data, work);

    kscan_charlieplex_read(data->dev);
}

static int kscan_charlieplex_configure(const struct device *dev, const kscan_callback_t callback) {
    if (!callback) {
        return -EINVAL;
    }

    struct kscan_charlieplex_data *data = dev->data;
    data->callback = callback;
    return 0;
}

static int kscan_charlieplex_enable(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;

    data->scan_time = k_uptime_get();

    /* Read will automatically start interrupts/polling once done. */
    return kscan_charlieplex_read(dev);
}

static int kscan_charlieplex_disable(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;

    k_work_cancel_delayable(&data->work);

    const struct kscan_charlieplex_config *config = dev->config;
    if (config->use_interrupt) {
        return kscan_charlieplex_interrupt_configure(dev, GPIO_INT_DISABLE);
    }

    return 0;
}

static const struct kscan_driver_api kscan_charlieplex_api = {
    .config = kscan_charlieplex_configure,
    .enable_callback = kscan_charlieplex_enable,
    .disable_callback = kscan_charlieplex_disable,
};

static int kscan_charlieplex_init_inputs(const struct device *dev) {
    const struct kscan_charlieplex_config *config = dev->config;

    for (int i = 0; i < config->cells.len; i++) {
        int err = kscan_charlieplex_set_as_input(&config->cells.gpios[i]);
        if (err) {
            return err;
        }
    }

    return 0;
}

static int kscan_charlieplex_init_interrupt(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;

    const struct kscan_charlieplex_config *config = dev->config;
    const struct gpio_dt_spec *gpio = &config->interrupt;
    int err = kscan_charlieplex_set_as_input(gpio);
    if (err) {
        return err;
    }

    gpio_init_callback(&data->irq_callback, kscan_charlieplex_irq_callback, BIT(gpio->pin));
    err = gpio_add_callback(gpio->port, &data->irq_callback);
    if (err) {
        LOG_ERR("Error adding the callback to the input device: %i", err);
    }
    return err;
}

static void kscan_charlieplex_setup_pins(const struct device *dev) {
    kscan_charlieplex_init_inputs(dev);
    kscan_charlieplex_set_all_outputs(dev, 0);

    const struct kscan_charlieplex_config *config = dev->config;
    if (config->use_interrupt) {
        kscan_charlieplex_init_interrupt(dev);
    }
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
#include <zephyr/sys/printk.h> // added

static int kscan_charlieplex_pm_action(const struct device *dev, enum pm_device_action action) {
    LOG_ERR("pm_action action=%d wake=%d", action, pm_device_wakeup_is_enabled(dev)); // changedLOG_ERR("pm_action action=%d wake=%d", action, pm_device_wakeup_is_enabled(dev)); // changed

        printk("kscan_charlieplex_pm_action: action=%d wake=%d\n",               // added
           action, pm_device_wakeup_is_enabled(dev));    
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        k_work_cancel_delayable(&((struct kscan_charlieplex_data *)dev->data)->work);           // added: stop scan work deterministically
        (void)kscan_charlieplex_set_all_as_input(dev);                                           // added: release any output drive before poweroff
        (void)kscan_charlieplex_interrupt_line_input_pulldown(dev);                              // added: ensure IRQ line is in input+pull state

        if (pm_device_wakeup_is_enabled(dev)) {                                                  // added: prepare wake for System OFF here
            (void)kscan_charlieplex_interrupt_configure(dev, GPIO_INT_LEVEL_ACTIVE);             // added: arm wake (sense/level) before sys_poweroff
        } else {
            (void)kscan_charlieplex_interrupt_configure(dev, GPIO_INT_DISABLE);                  // changed: keep explicit disable for non-wake suspend
        }

        (void)kscan_charlieplex_disconnect_all(dev);                                             // moved/kept: cells can be disconnected for low power
        return 0; 
        
        kscan_charlieplex_interrupt_configure(dev, GPIO_INT_DISABLE);
        kscan_charlieplex_disconnect_all(dev);

        return kscan_charlieplex_disable(dev);
    case PM_DEVICE_ACTION_RESUME:
        kscan_charlieplex_setup_pins(dev);

        /*
         * When entering soft off, ZMK resumes the devices listed in `soft_off_wakers`
         * and then quickly calls `sys_poweroff()`. In that path we must configure the
         * wake interrupt immediately (without starting a scan cycle) so the device can
         * wake the system from soft off.
         */
        if (pm_device_wakeup_is_enabled(dev)) {
            return kscan_charlieplex_interrupt_enable(dev);
        }

        return kscan_charlieplex_enable(dev);
    default:
        return -ENOTSUP;
    }
}

void titan8000_charlieplex_request_soft_off(void) {
    #if IS_ENABLED(CONFIG_ZMK_PM_SOFT_OFF)
    titan8000_play_soft_off_tone();
    atomic_set(&soft_off_pending, 1);

    if (titan8000_charlieplex_dev) {
        struct kscan_charlieplex_data *data = titan8000_charlieplex_dev->data;
        k_work_reschedule(&data->work, K_NO_WAIT);
    }
    #endif
}

#endif /* IS_ENABLED(CONFIG_PM_DEVICE) */

static int kscan_charlieplex_init(const struct device *dev) {
    struct kscan_charlieplex_data *data = dev->data;

    data->dev = dev;

    k_work_init_delayable(&data->work, kscan_charlieplex_work_handler);

#if IS_ENABLED(CONFIG_ZMK_PM_SOFT_OFF)
    titan8000_charlieplex_dev = dev;
#endif

#if IS_ENABLED(CONFIG_PM_DEVICE)
    pm_device_init_suspended(dev);

#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    pm_device_runtime_enable(dev);
#endif

#else
    kscan_charlieplex_setup_pins(dev);
#endif

    return 0;
}

#define KSCAN_CHARLIEPLEX_INIT(n)                                                                  \
    BUILD_ASSERT(INST_DEBOUNCE_PRESS_MS(n) <= DEBOUNCE_COUNTER_MAX,                                 \
                 "ZMK_KSCAN_DEBOUNCE_PRESS_MS or debounce-press-ms is too large");                 \
    BUILD_ASSERT(INST_DEBOUNCE_RELEASE_MS(n) <= DEBOUNCE_COUNTER_MAX,                               \
                 "ZMK_KSCAN_DEBOUNCE_RELEASE_MS or debounce-release-ms is too large");             \
    static struct zmk_debounce_state kscan_charlieplex_state_##n[INST_CHARLIEPLEX_LEN(n)];          \
    static const struct gpio_dt_spec kscan_charlieplex_cells_##n[] = {                              \
        LISTIFY(INST_LEN(n), KSCAN_GPIO_CFG_INIT, (, ), n)};                                        \
    static struct kscan_charlieplex_data kscan_charlieplex_data_##n = {                             \
        .charlieplex_state = kscan_charlieplex_state_##n,                                           \
    };                                                                                              \
    static const struct kscan_charlieplex_config kscan_charlieplex_config_##n = {                  \
        .cells = KSCAN_GPIO_LIST(kscan_charlieplex_cells_##n),                                      \
        .debounce_config =                                                                          \
            {                                                                                       \
                .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                                     \
                .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                                 \
            },                                                                                      \
        .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms),                        \
        COND_ANY_POLLING((.poll_period_ms = DT_INST_PROP(n, poll_period_ms), ))                     \
            COND_THIS_INTERRUPT(n, (.use_interrupt = INST_INTR_DEFINED(n), ))                       \
                COND_THIS_INTERRUPT(n, (.interrupt = KSCAN_INTR_CFG_INIT(n), ))                     \
        .discharge_before_inputs_us = INST_DISCHARGE_US(n),                                         \
    };                                                                                              \
    PM_DEVICE_DT_INST_DEFINE(n, kscan_charlieplex_pm_action);                                       \
    DEVICE_DT_INST_DEFINE(n, &kscan_charlieplex_init, PM_DEVICE_DT_INST_GET(n),                     \
                          &kscan_charlieplex_data_##n, &kscan_charlieplex_config_##n, POST_KERNEL,  \
                          CONFIG_KSCAN_INIT_PRIORITY, &kscan_charlieplex_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_CHARLIEPLEX_INIT);
