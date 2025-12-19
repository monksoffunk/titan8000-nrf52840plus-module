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
// Buzzer implementation (only compiled when CONFIG_TITAN8000_BUZZER is enabled)
static const struct pwm_dt_spec buzzer_pwm = PWM_DT_SPEC_GET(BUZZER_NODE);
static const note_t *current_melody = NULL;
static uint32_t melody_length = 0;
static uint32_t current_index = 0;
static bool melody_loop = false;
static struct k_timer melody_timer;
static struct k_timer advertising_beep_timer;
static bool is_advertising_beep_active = false;
static bool keypress_beep_enabled = false;
static struct k_work_delayable melody_work;
static bool melody_work_running = false;
static struct k_work buzzer_work;
static struct k_work_delayable melody_work;
static struct buzzer_request buzzer_req;
static atomic_t buzzer_busy;
static atomic_t melody_active;
static atomic_t buzzer_abort;

K_THREAD_STACK_DEFINE(buzzer_stack, 1024);
static struct k_work_q buzzer_work_q;

/* 0..255 scale (approx exponential decay) */
static const uint8_t decay_lut[] = {
    255, 220, 190, 165, 142, 122, 104,  88,
     74,  62,  52,  43,  35,  28,  22,  17,
     13,   9,   6,   4,   2,   1,   0
};

// BLE profile change melody (ascending tones)
const note_t ble_profile_change[] = {
    {NOTE_C7, 140},
    {NOTE_E7, 100},
    {NOTE_G7, 140}
};

// BLE bond clear melody (descending tones)
const note_t ble_bond_clear[] = {
    {NOTE_G7, 140},
    {NOTE_E7, 100},
    {NOTE_C7, 140}
};

// BLE all bonds clear melody (warning sound)
const note_t ble_all_bonds_clear[] = {
    {NOTE_A7, 100},
    {NOTE_REST, 50},
    {NOTE_A7, 100},
    {NOTE_REST, 50},
    {NOTE_A7, 150}
};

// BLE disconnect melody (short beep)
const note_t ble_disconnect[] = {
    {NOTE_E7, 140},
    {NOTE_REST, 100},
    {NOTE_C7, 140}
};

// BLE advertising beep (repeating pip-pip)
const note_t ble_advertising_beep[] = {
    {NOTE_G7, 150},
    {NOTE_REST, 50},
    {NOTE_G7, 150}
};

const note_t success[] = {
    {NOTE_E7, 140}, {NOTE_B7, 140}, {NOTE_E8, 400},
};

const note_t warning[] = {
    {NOTE_A6, 100}, {NOTE_REST, 80},
    {NOTE_A6, 100}, {NOTE_REST, 80},
    {NOTE_A6, 100},
};

void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms)
{
    if (!device_is_ready(buzzer_pwm.dev)) {
		    LOG_INF("========================================");
			LOG_INF("PWM BUZZER not ready!!");
		    LOG_INF("========================================");
        return;
    }

    uint32_t period_ns = 1000000000UL / freq_hz;

    pwm_set_dt(&buzzer_pwm, period_ns, period_ns / 2);  // 50% duty
    k_msleep(duration_ms);
	// off
	pwm_set_dt(&buzzer_pwm, 0, 0);
}

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

static buzzer_voice_fn_t current_voice = buzzer_voice_plain;

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
        // t = i/steps, curve = t^2 (0->1)
        // freq = f_start - (f_start-f_end)*t^2
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
        if (atomic_get(&buzzer_abort)) return;
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
        if (atomic_get(&buzzer_abort)) return;
        uint32_t pulse = (max_pulse * decay_lut[i]) / 255;
        pwm_set_dt(pwm, period_ns, pulse);
        k_sleep(K_MSEC(decay_step_ms));
    }

    /* Stop */
    pwm_set_dt(pwm, 0, 0);
}

static void buzzer_voice_soft(
    const struct pwm_dt_spec *pwm,
    uint32_t freq_hz,
    uint32_t duration_ms
)
{
    const uint32_t period_ns = 1000000000UL / freq_hz;
    const uint32_t max_pulse = period_ns / 2;

    /* ---- envelope design ---- */
    uint32_t attack_ms  = duration_ms / 8;        // ~12%
    uint32_t sustain_ms = duration_ms / 4;        // ~25%
    uint32_t decay_ms   = duration_ms - attack_ms - sustain_ms;

    if (attack_ms < 4)  attack_ms = 4;
    if (decay_ms  < 8)  decay_ms  = 8;

    const uint32_t duty_floor = max_pulse / 4;    // 25%

    /* ---------- Attack (quadratic ease-in) ---------- */
    const uint32_t attack_step_ms = 2;
    uint32_t a_steps = attack_ms / attack_step_ms;
    if (a_steps == 0) a_steps = 1;

    for (uint32_t i = 0; i <= a_steps; i++) {
        if (atomic_get(&buzzer_abort)) return;

        uint32_t num = i * i;
        uint32_t den = a_steps * a_steps;
        uint32_t pulse =
            duty_floor +
            ((max_pulse - duty_floor) * num) / den;

        pwm_set_dt(pwm, period_ns, pulse);
        k_sleep(K_MSEC(attack_step_ms));
    }

    /* ---------- Sustain (fixed level) ---------- */
    pwm_set_dt(pwm, period_ns, max_pulse * 3 / 4);   // ~75%
    k_sleep(K_MSEC(sustain_ms));

    /* ---------- Decay (LUT, but never to zero) ---------- */
    uint32_t decay_step_ms = decay_ms / ARRAY_SIZE(decay_lut);
    if (decay_step_ms == 0) decay_step_ms = 1;

    for (uint32_t i = 0; i < ARRAY_SIZE(decay_lut); i++) {
        if (atomic_get(&buzzer_abort)) return;

        uint32_t pulse =
            duty_floor +
            ((max_pulse - duty_floor) * decay_lut[i]) / 255;

        pwm_set_dt(pwm, period_ns, pulse);
        k_sleep(K_MSEC(decay_step_ms));
    }

    /* stop (explicit silence) */
    pwm_set_dt(pwm, 0, 0);
}

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
    buzzer_req.voice(
        &buzzer_pwm,
        buzzer_req.freq_hz,
        buzzer_req.duration_ms
    );

    atomic_clear(&buzzer_busy);
}

static void melody_work_handler(struct k_work *work)
{
    /* Acquire PWM ownership */ 
    if (!atomic_cas(&buzzer_busy, 0, 1)) {
        /* Another beep is in-flight; reschedule a little later */
        k_work_schedule_for_queue(&buzzer_work_q, &melody_work, K_MSEC(5));
        return;                           
    }                                     

    if (!current_melody || melody_length == 0) {
        atomic_clear(&melody_active);
        atomic_clear(&buzzer_busy);       
        return;
    }

    while (current_melody) {
        if (atomic_get(&buzzer_abort)) break;

        if (current_index >= melody_length) {
            if (melody_loop) {
                current_index = 0;
            } else {
                break;
            }
        }

        const note_t *note = &current_melody[current_index++];

        if (note->freq == NOTE_REST) {
            pwm_set_dt(&buzzer_pwm, 0, 0);
            k_msleep(note->duration);
            continue;
        }

        current_voice(&buzzer_pwm, note->freq, note->duration);
    }

    pwm_set_dt(&buzzer_pwm, 0, 0);
    atomic_clear(&melody_active);
    current_melody = NULL;
    atomic_clear(&buzzer_busy);           
    atomic_clear(&buzzer_abort);
}

static bool buzzer_request(
    buzzer_voice_fn_t voice,
    uint32_t freq_hz,
    uint32_t duration_ms
)
{
    if (atomic_get(&melody_active))
        return false;

    atomic_set(&buzzer_abort, 1);
    pwm_set_dt(&buzzer_pwm,0, 0);

    atomic_set(&buzzer_busy, 1);
    atomic_clear(&buzzer_abort);

    buzzer_req.voice       = voice;
    buzzer_req.freq_hz     = freq_hz;
    buzzer_req.duration_ms = duration_ms;

    k_work_submit_to_queue(&buzzer_work_q, &buzzer_work);
    return true;
}
/*
static void melody_timer_callback(struct k_timer *timer)
{
    if (current_index >= melody_length) {
        if (melody_loop) {
            current_index = 0;
        } else {
            buzzer_stop_melody();
            return;
        }
    }

    const note_t *note = &current_melody[current_index++];

    if (note->freq == NOTE_REST) {
        pwm_set_dt(&buzzer_pwm, 0, 0);                    // off
    } else {
        uint32_t period_ns = 1000000000UL / note->freq;
        pwm_set_dt(&buzzer_pwm, period_ns, period_ns / 2); // plain beep
    }

    k_timer_start(&melody_timer, K_MSEC(note->duration), K_NO_WAIT);
}
*/

void buzzer_play_melody_ex(
    const note_t *melody,
    uint32_t length,
    bool loop,
    buzzer_voice_fn_t voice
)
{
    buzzer_stop_melody();

    current_melody = melody;
    melody_length = length;
    current_index = 0;
    melody_loop = loop;
    current_voice = voice ? voice : buzzer_voice_plain;

    atomic_set(&melody_active, 1);
    k_work_schedule_for_queue(&buzzer_work_q, &melody_work, K_NO_WAIT);
}

void buzzer_play_melody(const note_t *melody, uint32_t length, bool loop)
{
    buzzer_play_melody_ex(melody, length, loop, buzzer_voice_plain);
}

void buzzer_stop_melody(void)
{
    k_work_cancel_delayable(&melody_work);
    pwm_set_dt(&buzzer_pwm, 0, 0);
    current_melody = NULL;
    atomic_clear(&melody_active);
}

bool buzzer_is_playing(void)
{
    return (current_melody != NULL);
}

void buzzer_toggle_keypress_beep(void)
{
    keypress_beep_enabled = !keypress_beep_enabled;
    LOG_INF("Keypress beep %s", keypress_beep_enabled ? "ENABLED" : "DISABLED");
    
    // Play confirmation sound
    if (keypress_beep_enabled) {
        note_t melody[] = { {NOTE_C6, 100} };
        buzzer_play_melody(melody, 1, false);
    } else {
        note_t melody[] = { {NOTE_E6, 100} };
        buzzer_play_melody(melody, 1, false);
    }
}

void buzzer_pitch_fall() 
{
    buzzer_fall_quadratic_hz(&buzzer_pwm, 4000, 3000, 50);
}

bool buzzer_is_keypress_beep_enabled(void)
{
    return keypress_beep_enabled;
}

static void advertising_beep_callback(struct k_timer *timer)
{
    if (!zmk_ble_active_profile_is_connected()) {
        // Not connected, play advertising beep
        buzzer_play_melody_ex(ble_advertising_beep, sizeof(ble_advertising_beep) / sizeof(note_t), false, buzzer_voice_ad);
    } else {
        // Connected, stop advertising beep
        is_advertising_beep_active = false;
        k_timer_stop(&advertising_beep_timer);
    }
}

static void start_advertising_beep(void)
{
    if (!is_advertising_beep_active) {
        is_advertising_beep_active = true;
        k_timer_start(&advertising_beep_timer, K_SECONDS(3), K_SECONDS(3));
        LOG_INF("Advertising beep started");
    }
}

static void stop_advertising_beep(void)
{
    if (is_advertising_beep_active) {
        is_advertising_beep_active = false;
        k_timer_stop(&advertising_beep_timer);
        LOG_INF("Advertising beep stopped");
    }
}

static int buzzer_init(void)
{
    LOG_INF("========================================");
    LOG_INF("BUZZER MODULE INITIALIZED");
    if (!device_is_ready(buzzer_pwm.dev)) {
        LOG_INF("PWM Device NOT READY!");
        return -ENODEV;
    }
    LOG_INF("PWM Device: %s", buzzer_pwm.dev->name);
    LOG_INF("PWM Ready: YES");
    LOG_INF("========================================");
        
    k_work_queue_start(
        &buzzer_work_q,
        buzzer_stack,
        K_THREAD_STACK_SIZEOF(buzzer_stack),
        K_PRIO_PREEMPT(5),
        NULL
    );

    k_work_init(&buzzer_work, buzzer_work_handler);
    k_work_init_delayable(&melody_work, melody_work_handler);
    k_timer_init(&advertising_beep_timer, advertising_beep_callback, NULL);

    buzzer_play_melody_ex(success, ARRAY_SIZE(success), false, buzzer_voice_ad);

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
    if (ev->state && keypress_beep_enabled) {
        LOG_INF("KEY PRESSED at position %d", ev->position);
        buzzer_request(buzzer_voice_soft, 4000, 150);
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
    
    // Check if the profile is open (no bond) - likely a clear operation
    if (zmk_ble_profile_is_open(ev->index)) {
        LOG_INF("Profile is open (cleared)");
        buzzer_play_melody_ex(ble_bond_clear, sizeof(ble_bond_clear) / sizeof(note_t), false, buzzer_voice_ad); 
        // Start advertising beep after clearing
        start_advertising_beep();
    } else {
        LOG_INF("Profile switched");
        buzzer_play_melody_ex(ble_profile_change, sizeof(ble_profile_change) / sizeof(note_t), false, buzzer_voice_ad);
        
        // Check if connected, if not start advertising beep
        if (!zmk_ble_active_profile_is_connected()) {
            start_advertising_beep();
        } else {
            stop_advertising_beep();
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(buzzer_ble, buzzer_ble_profile_listener);
ZMK_SUBSCRIPTION(buzzer_ble, zmk_ble_active_profile_changed);

#endif /* CONFIG_TITAN8000_BUZZER */