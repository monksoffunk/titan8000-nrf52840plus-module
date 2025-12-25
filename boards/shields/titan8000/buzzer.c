/*
 * Copyright (c) 2025 monksoffunk
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "buzzer.h"

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/ble.h>

LOG_MODULE_REGISTER(buzzer, CONFIG_ZMK_LOG_LEVEL);

#ifdef CONFIG_TITAN8000_BUZZER

#define BUZZER_NODE DT_CHILD(DT_PATH(buzzers), buzzer)
#define BUZZER_PORTAMENT_UP_ENABLE
// #define BUZZER_PORTAMENT_DOWN_ENABLE
// #define BUZZER_FALL_ENABLE

struct buzzer_state {
    struct pwm_dt_spec pwm;

    struct k_work_delayable boot_adv_check_work;
    struct k_work_q work_q;
    struct k_work work, melody_work;
    struct k_timer adv_timer;

    const note_t *melody;
    uint32_t melody_len;
    uint32_t index;
    bool loop;
    buzzer_voice_fn_t voice;

    struct buzzer_request req;

    atomic_t state;

    bool adv_active;
    bool keypress_enabled;
};

static struct buzzer_state buzzer;

enum buzzer_state_bits {
    BUZZER_ENABLED = 0,
    BUZZER_BUSY,
    BUZZER_ABORT,
    BUZZER_INTERRUPT_PENDING,
};

typedef enum {
    BLE_ADVERTISING = 0,
    BLE_CONNECTED,
    BLE_NON_CONNECTED,
} profile_status_t;

K_THREAD_STACK_DEFINE(buzzer_stack, 1024);

static profile_status_t get_ble_active_status(void) {
    profile_status_t state;

    if (zmk_ble_active_profile_is_connected()) {
        state = BLE_CONNECTED;
    } else if (zmk_ble_active_profile_is_open()) {
        state = BLE_ADVERTISING;
    } else {
        state = BLE_NON_CONNECTED;
    }

    return state;
}

/* 0..255 scale (approx exponential decay) */
static const uint8_t decay_lut[] = {
    255, 220, 190, 165, 142, 122, 104,  88,
     74,  62,  52,  43,  35,  28,  22,  17,
     13,   9,   6,   4,   2,   1,   0,
};

struct buzzer_melody_request {
    sys_snode_t node;
    const note_t *melody;
    uint32_t length;
    bool loop;
    buzzer_voice_fn_t voice;
};

// BUZZER_MELODY_QUEUE_LEN SHOULD BE <= 32
#define BUZZER_MELODY_QUEUE_LEN 16

static struct buzzer_melody_request melody_pool[BUZZER_MELODY_QUEUE_LEN];
static atomic_t melody_pool_bitmap;

K_FIFO_DEFINE(buzzer_melody_fifo);


// BLE profile change melody
static const note_t ble_profile_change[] = {
    {NOTE_G7, 140}, 
    {NOTE_E7, 140}, 
    {NOTE_C7, 140},
};

// BLE bond clear melody
static const note_t ble_bond_clear[] = {
    {NOTE_C7, 140}, 
    {NOTE_G7, 140},
};

// BLE disconnect melody
static const note_t ble_disconnect[] = {
    {NOTE_C7, 140}, 
};

// BLE advertising beep (repeating pip-pip)
static const note_t ble_advertising_beep[] = {
    {NOTE_G7, 150}, 
    {NOTE_REST, 50}, 
    {NOTE_G7, 150},
};

static const note_t start[] = {
    {NOTE_REST, 50}, 
    {NOTE_D6, 140},
    {NOTE_REST, 100}, 
    {NOTE_G6, 140}, 
    {NOTE_REST, 100}, 
    {NOTE_B6, 140}, 
    {NOTE_REST, 100}, 
    {NOTE_C7, 140},
    {NOTE_REST, 100}, 
};

static const note_t soft_off[] = {
    {NOTE_C7, 140}, 
    {NOTE_REST, 100}, 
    {NOTE_B6, 140}, 
    {NOTE_REST, 100}, 
    {NOTE_G6, 140}, 
    {NOTE_REST, 100}, 
    {NOTE_D6, 140},
};

// keyprss toggle sound
// static const note_t keypress_on[] = { {NOTE_C7, 100}, {NOTE_C8, 100} };
static const note_t keypress_off[] = { {NOTE_E7, 100}, {NOTE_D7, 100} };


#define BUZZER_MELODY_REQ(notes_array) \
    buzzer_melody_request((notes_array), sizeof(notes_array) / sizeof(note_t), false, buzzer_voice_ad)

static void buzzer_voice_plain(
    const struct pwm_dt_spec *pwm,
    uint32_t freq_hz,
    uint32_t duration_ms
)
{
    uint32_t period_ns = 1000000000UL / freq_hz;
    pwm_set_dt(pwm, period_ns, period_ns / 2);
    k_msleep(duration_ms);
    pwm_set_dt(pwm, 0, 0);
}


#ifdef BUZZER_FALL_ENABLE
static void buzzer_fall_quadratic_hz(
    const struct pwm_dt_spec *pwm,
    uint32_t f_start_hz,
    uint32_t f_end_hz,
    uint32_t duration_ms
) {
    const uint32_t step_ms = 5;
    const uint32_t steps = duration_ms / step_ms;
    if (steps == 0) return;

    for (uint32_t i = 0; i <= steps; i++) {
        uint32_t num = i * i;                      // i^2
        uint32_t den = steps * steps;              // steps^2
        uint32_t df  = f_start_hz - f_end_hz;

        uint32_t f = f_start_hz - (df * num) / den;

        // period in nanoseconds
        uint32_t period_ns = 1000000000UL / f;
        uint32_t pulse_ns  = period_ns / 2;

        pwm_set_dt(pwm, period_ns, pulse_ns);
        k_sleep(K_MSEC(step_ms));
    }

    pwm_set_dt(pwm, 0, 0);
}

static void buzzer_voice_fall(
    const struct pwm_dt_spec *pwm,
    uint32_t freq_hz,
    uint32_t duration_ms
)
{
    buzzer_fall_quadratic_hz(
        pwm,
        freq_hz,
        freq_hz * 3 / 4,
        duration_ms
    );
}

static void buzzer_pitch_fall() 
{
    buzzer_fall_quadratic_hz(&buzzer.pwm, 4000, 3000, 50);
}
#endif /* BUZZER_FALL_ENABLE */

static void buzzer_beep_ad(
    const struct pwm_dt_spec *pwm,
    uint32_t freq_hz,
    uint32_t attack_ms,
    uint32_t decay_ms
) {
    const uint32_t period_ns = 1000000000UL / freq_hz;
    const uint32_t max_pulse = period_ns / 2;   // 50% duty

    /* ---------- Attack (linear) ---------- */
    const uint32_t attack_step_ms = 2;
    uint32_t a_steps = attack_ms / attack_step_ms;
    if (a_steps == 0) {
        a_steps = 1;
    }

    for (uint32_t i = 0; i <= a_steps; i++) {
        if (atomic_test_bit(&buzzer.state, BUZZER_ABORT)) return;
        uint32_t pulse = (max_pulse * i) / a_steps;
        pwm_set_dt(pwm, period_ns, pulse);
        k_sleep(K_MSEC(attack_step_ms));
    }

    /* ---------- Decay (LUT-based) ---------- */
    /* Spread LUT over requested decay_ms */
    uint32_t decay_step_ms = decay_ms / ARRAY_SIZE(decay_lut);
    if (decay_step_ms == 0) {
        decay_step_ms = 1;
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(decay_lut); i++) {
        if (atomic_test_bit(&buzzer.state, BUZZER_ABORT)) return;
        uint32_t pulse = (max_pulse * decay_lut[i]) / 255;
        pwm_set_dt(pwm, period_ns, pulse);
        k_sleep(K_MSEC(decay_step_ms));
    }

    /* Stop */
    pwm_set_dt(pwm, 0, 0);
}

#if defined(BUZZER_PORTAMENT_UP_ENABLE) || defined(BUZZER_PORTAMENT_DOWN_ENABLE) || defined(BUZZER_PORTAMENT_ENABLE)
static void buzzer_ad_portamento(
    const struct pwm_dt_spec *pwm,
    uint32_t freq_start_hz,
    uint32_t freq_end_hz,
    uint32_t duration_ms
)
{
    const uint32_t step_ms = 2;

    uint32_t attack_ms = duration_ms / 6;
    uint32_t decay_ms  = duration_ms / 4;

    if (attack_ms < 4) attack_ms = 4;
    if (decay_ms  < 8) decay_ms  = 8;

    int32_t total_steps = duration_ms / step_ms;
    if (total_steps == 0) total_steps = 1;

    uint32_t period_start = 1000000000UL / freq_start_hz;
    uint32_t period_end   = 1000000000UL / freq_end_hz;
    int32_t period_diff = period_start - period_end;

    uint32_t max_pulse  = period_start / 2;
    uint32_t duty_floor = max_pulse / 4;

    for (int32_t i = 0; i <= total_steps; i++) {
        if (atomic_test_bit(&buzzer.state, BUZZER_ABORT)) return;

        uint32_t period = period_start - (period_diff * i) / total_steps;
        uint32_t t_ms = i * step_ms;
        uint32_t pulse;

        if (t_ms < attack_ms) {
            uint32_t a = t_ms * t_ms;
            uint32_t b = attack_ms * attack_ms;
            pulse = duty_floor +
                ((max_pulse - duty_floor) * a) / b;
        } else if (t_ms > (duration_ms - decay_ms)) {
            uint32_t d = duration_ms - t_ms;
            pulse = duty_floor +
                ((max_pulse - duty_floor) * d) / decay_ms;
        } else {
            pulse = max_pulse * 3 / 4;
        }

        pwm_set_dt(pwm, period, pulse);
        k_sleep(K_MSEC(step_ms));
    }

    pwm_set_dt(pwm, 0, 0);
}
#endif /* defined(BUZZER_PORTAMENT_UP_ENABLE) || defined(BUZZER_PORTAMENT_DOWN_ENABLE) || defined(BUZZER_PORTAMENT_ENABLE) */

#ifdef BUZZER_PORTAMENT_UP_ENABLE
static void buzzer_voice_ad_portamento_up(const struct pwm_dt_spec *pwm, uint32_t freq, uint32_t duration_ms) {
    int32_t freq_end = freq + freq / 12;
    buzzer_ad_portamento(pwm, freq, freq_end, duration_ms);
}
#endif

#ifdef BUZZER_PORTAMENTO_DOWN_ENABLE
static void buzzer_voice_ad_portamento_down(const struct pwm_dt_spec *pwm, uint32_t freq, uint32_t duration_ms) {
    int32_t freq_end = freq - freq / 12;
    if (freq_end <= 0) return;
    buzzer_ad_portamento(pwm, freq, freq_end, duration_ms);
}
#endif

static void buzzer_voice_ad(
    const struct pwm_dt_spec *pwm,
    uint32_t freq_hz,
    uint32_t duration_ms
)
{
    uint32_t attack_ms = duration_ms / 6;
    uint32_t decay_ms  = duration_ms - attack_ms;

    buzzer_beep_ad(pwm, freq_hz, attack_ms, decay_ms);
}

/* worker functions */
static void buzzer_work_handler(struct k_work *work)
{
//    if (atomic_test_and_set_bit(&buzzer.state, BUZZER_BUSY)) {
//        return;
//    }
    atomic_clear_bit(&buzzer.state, BUZZER_INTERRUPT_PENDING);
    
    buzzer.req.voice(
        &buzzer.pwm,
        buzzer.req.freq_hz,
        buzzer.req.duration_ms
    );

//    atomic_clear_bit(&buzzer.state, BUZZER_BUSY);
    if (!k_fifo_is_empty(&buzzer_melody_fifo)) {
        k_work_submit_to_queue(&buzzer.work_q, &buzzer.melody_work);
    }
}

static bool buzzer_request(
    buzzer_voice_fn_t voice,
    uint32_t freq_hz,
    uint32_t duration_ms
)
{
    if (!atomic_test_bit(&buzzer.state, BUZZER_ENABLED)) {
        return false;
    }

    atomic_set_bit(&buzzer.state, BUZZER_ABORT);
    pwm_set_dt(&buzzer.pwm,0, 0);
    atomic_clear_bit(&buzzer.state, BUZZER_ABORT);

    buzzer.req.voice       = voice;
    buzzer.req.freq_hz     = freq_hz;
    buzzer.req.duration_ms = duration_ms;

    k_work_submit_to_queue(&buzzer.work_q, &buzzer.work);
    atomic_set_bit(&buzzer.state, BUZZER_INTERRUPT_PENDING);
    return true;
}

static struct buzzer_melody_request *alloc_melody_req(void) {
    for (int i = 0; i < BUZZER_MELODY_QUEUE_LEN; i++) {
        if (!atomic_test_and_set_bit(&melody_pool_bitmap, i)) {
            return &melody_pool[i];
        }
    }
    return NULL;
}

static void free_melody_req(struct buzzer_melody_request *req) {
    int index = req - melody_pool;
    if (index >= 0 && index < BUZZER_MELODY_QUEUE_LEN) {
        atomic_clear_bit(&melody_pool_bitmap, index);
    }
}

bool buzzer_melody_request(
    const note_t *melody,
    uint32_t len,
    bool loop,
    buzzer_voice_fn_t voice
)
{
    if (!atomic_test_bit(&buzzer.state, BUZZER_ENABLED)) {
        return false;
    }

    struct buzzer_melody_request *req = alloc_melody_req();
    if (!req) {
        LOG_WRN("Melody queue is full, request dropped");
        return false;
    }

    req->melody = melody;
    req->length = len;
    req->loop = loop;
    req->voice = voice ? voice : buzzer_voice_plain;

    k_fifo_put(&buzzer_melody_fifo, req);
    k_work_submit_to_queue(&buzzer.work_q, &buzzer.melody_work);

    return true;
}

static void melody_work_handler(struct k_work *work) {
    static struct buzzer_melody_request *req = NULL;
    static uint32_t i = 0;

//    if (atomic_test_and_set_bit(&buzzer.state, BUZZER_BUSY)) {
//        return;
//    }

    while (1) {
        if (!req) {
            req = k_fifo_get(&buzzer_melody_fifo, K_NO_WAIT);
            i = 0;
        }
        if (!req) {
            break;
        }

        const note_t *melody = req->melody;
        uint32_t len = req->length;
        buzzer_voice_fn_t voice = req->voice;

        for (; i < len; i++) {
            if (atomic_test_and_clear_bit(&buzzer.state, BUZZER_INTERRUPT_PENDING)) {
            //    atomic_clear_bit(&buzzer.state, BUZZER_BUSY);
              //  atomic_clear_bit(&buzzer.state, BUZZER_INTERRUPT_PENDING);
                k_work_submit_to_queue(&buzzer.work_q, &buzzer.melody_work);
                return;
            }

            const note_t *note = &melody[i];

            if (note->freq == NOTE_REST) {
                pwm_set_dt(&buzzer.pwm, 0,0);
                k_msleep(note->duration);
            } else {
                voice(&buzzer.pwm, note->freq, note->duration);
            }
        }
        pwm_set_dt(&buzzer.pwm, 0, 0);
        free_melody_req(req);
        req = NULL;
    }

    atomic_clear_bit(&buzzer.state, BUZZER_BUSY);
}

void buzzer_toggle_keypress_beep(void)
{
    buzzer.keypress_enabled = !buzzer.keypress_enabled;
    LOG_INF("Keypress beep %s", buzzer.keypress_enabled ? "ENABLED" : "DISABLED");
    
    // Play confirmation sound
    if (buzzer.keypress_enabled) {
        buzzer_voice_ad_portamento_up(&buzzer.pwm, NOTE_C7, 140);
        //BUZZER_MELODY_REQ(keypress_on);
    } else {
        BUZZER_MELODY_REQ(keypress_off);
    }
}

bool buzzer_is_keypress_enabled(void)
{
    return buzzer.keypress_enabled;
}

static void advertising_beep_callback(struct k_timer *timer)
{
    if (get_ble_active_status() == BLE_ADVERTISING) {
        BUZZER_MELODY_REQ(ble_advertising_beep);
    } else {
        // Connected or trying to connect, stop advertising beep
        buzzer.adv_active = false;
        k_timer_stop(&(buzzer.adv_timer));
    }
}

static void start_advertising_beep(void)
{
    if (!buzzer.adv_active) {
        buzzer.adv_active = true;
        k_timer_start(&(buzzer.adv_timer), K_SECONDS(3), K_SECONDS(3));
        LOG_INF("Advertising beep started");
    }
}

static void stop_advertising_beep(void)
{
    if (buzzer.adv_active) {
        buzzer.adv_active = false;
        k_timer_stop(&(buzzer.adv_timer));
        LOG_INF("Advertising beep stopped");
    }
}

static struct buzzer_state buzzer = {
    .pwm = PWM_DT_SPEC_GET(BUZZER_NODE),
    .state = ATOMIC_INIT(BIT(BUZZER_ENABLED)),
    .voice = buzzer_voice_plain,
};

static void boot_adv_check_handler(struct k_work *work) {
    const int status = get_ble_active_status();
    switch (status) {
        case BLE_ADVERTISING:
            start_advertising_beep();
            break;
        case BLE_NON_CONNECTED:
        case BLE_CONNECTED:
        default:
            break;
    }
}

static int buzzer_init(void)
{
    LOG_INF("========================================");
    LOG_INF("BUZZER MODULE INITIALIZED");
    if (!device_is_ready(buzzer.pwm.dev)) {
        LOG_INF("PWM Device NOT READY!");
        return -ENODEV;
    }
    LOG_INF("PWM Device: %s", buzzer.pwm.dev->name);
    LOG_INF("PWM Ready: YES");
    LOG_INF("========================================");
    
    k_work_queue_start(
        &buzzer.work_q,
        buzzer_stack,
        K_THREAD_STACK_SIZEOF(buzzer_stack),
        K_PRIO_PREEMPT(5),
        NULL
    );

    k_work_init(&buzzer.work, buzzer_work_handler);
    k_work_init(&buzzer.melody_work, melody_work_handler);
    k_timer_init(&buzzer.adv_timer, advertising_beep_callback, NULL);

    BUZZER_MELODY_REQ(start);

    k_work_init_delayable(&buzzer.boot_adv_check_work, boot_adv_check_handler);
    k_work_schedule(&buzzer.boot_adv_check_work, K_SECONDS(1));    

    return 0;
}

SYS_INIT(buzzer_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int buzzer_keypress_listener(const zmk_event_t *eh)
{
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Only play sound when the key is pressed (do not play when released)
    if (ev->state && buzzer.keypress_enabled) {
        LOG_INF("KEY PRESSED at position %d", ev->position);
        buzzer_request(buzzer_voice_ad, NOTE_B7, 150);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(buzzer, buzzer_keypress_listener);
ZMK_SUBSCRIPTION(buzzer, zmk_position_state_changed);

static int buzzer_ble_profile_listener(const zmk_event_t *eh)
{
    struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_INF("BLE Profile changed to: %d", ev->index);
    
    const int status = get_ble_active_status();

    switch (status) {
        case BLE_ADVERTISING:
            BUZZER_MELODY_REQ(ble_bond_clear);
            start_advertising_beep();
            break;
        case BLE_CONNECTED:
            BUZZER_MELODY_REQ(ble_profile_change);
            stop_advertising_beep();
            break;
        case BLE_NON_CONNECTED:
            BUZZER_MELODY_REQ(ble_disconnect);
            stop_advertising_beep();
            break;
        default:
            break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(buzzer_ble, buzzer_ble_profile_listener);
ZMK_SUBSCRIPTION(buzzer_ble, zmk_ble_active_profile_changed);

void buzzer_toggle_enable(void)
{
    if (atomic_test_bit(&buzzer.state, BUZZER_ENABLED)) {
        atomic_clear_bit(&buzzer.state, BUZZER_ENABLED);
        pwm_set_dt(&buzzer.pwm, 0, 0);
        atomic_set_bit(&buzzer.state, BUZZER_ABORT);
        LOG_INF("Buzzer muted");
    } else {
        atomic_set_bit(&buzzer.state, BUZZER_ENABLED);
        LOG_INF("Buzzer enabled");
    }
}

void titan8000_play_soft_off_tone(void) {
    BUZZER_MELODY_REQ(soft_off);
    k_sleep(K_MSEC(900));
}

#endif /* CONFIG_TITAN8000_BUZZER */