#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "buzzer.h"

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/ble.h>

LOG_MODULE_REGISTER(buzzer, CONFIG_ZMK_LOG_LEVEL);

#define BUZZER_NODE DT_CHILD(DT_PATH(buzzers), buzzer)
static const struct pwm_dt_spec buzzer_pwm = PWM_DT_SPEC_GET(BUZZER_NODE);
static const note_t *current_melody = NULL;
static uint32_t melody_length = 0;
static uint32_t current_index = 0;
static bool melody_loop = false;
static struct k_timer melody_timer;
static struct k_timer advertising_beep_timer;
static bool is_advertising_beep_active = false;
static bool keypress_beep_enabled = false;

// BLE profile change melody (ascending tones)
const note_t ble_profile_change[] = {
    {NOTE_C6, 80},
    {NOTE_E6, 80},
    {NOTE_G6, 120}
};

// BLE bond clear melody (descending tones)
const note_t ble_bond_clear[] = {
    {NOTE_G6, 80},
    {NOTE_E6, 80},
    {NOTE_C6, 120}
};

// BLE all bonds clear melody (warning sound)
const note_t ble_all_bonds_clear[] = {
    {NOTE_A6, 100},
    {NOTE_REST, 50},
    {NOTE_A6, 100},
    {NOTE_REST, 50},
    {NOTE_A6, 150}
};

// BLE disconnect melody (short beep)
const note_t ble_disconnect[] = {
    {NOTE_E6, 100},
    {NOTE_REST, 50},
    {NOTE_C6, 100}
};

// BLE advertising beep (repeating pip-pip)
const note_t ble_advertising_beep[] = {
    {NOTE_G6, 60},
    {NOTE_REST, 40},
    {NOTE_G6, 60}
};

const note_t success[] = {
    {NOTE_E6, 80}, {NOTE_B6, 80}, {NOTE_E7, 400},
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
//    pwm_set_dt(&buzzer_pwm, 0, 0);  // off
	pwm_set_dt(&buzzer_pwm, period_ns, 0);   // off
}

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
        pwm_set_dt(&buzzer_pwm, 0, 0);
    } else {
        uint32_t period_ns = 1000000000UL / note->freq;
        pwm_set_dt(&buzzer_pwm, period_ns, period_ns / 2);
    }

    k_timer_start(&melody_timer, K_MSEC(note->duration), K_NO_WAIT);
}

void buzzer_play_melody(const note_t *melody, uint32_t length, bool loop)
{
    buzzer_stop_melody();

    current_melody = melody;
    melody_length = length;
    current_index = 0;
    melody_loop = loop;

    k_timer_start(&melody_timer, K_NO_WAIT, K_NO_WAIT);
}

void buzzer_stop_melody(void)
{
    k_timer_stop(&melody_timer);
    pwm_set_dt(&buzzer_pwm, 0, 0);
    current_melody = NULL;
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

bool buzzer_is_keypress_beep_enabled(void)
{
    return keypress_beep_enabled;
}

static void advertising_beep_callback(struct k_timer *timer)
{
    if (!zmk_ble_active_profile_is_connected()) {
        // Not connected, play advertising beep
        buzzer_play_melody(ble_advertising_beep, sizeof(ble_advertising_beep) / sizeof(note_t), false);
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
    
    // Initialize timers once
    k_timer_init(&melody_timer, melody_timer_callback, NULL);
    k_timer_init(&advertising_beep_timer, advertising_beep_callback, NULL);
    
    buzzer_play_melody(success, ARRAY_SIZE(success), false);

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
        buzzer_beep(4000, 50);  // 4kHz, 50ms
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
        buzzer_play_melody(ble_bond_clear, sizeof(ble_bond_clear) / sizeof(note_t), false);
        // Start advertising beep after clearing
        start_advertising_beep();
    } else {
        LOG_INF("Profile switched");
        buzzer_play_melody(ble_profile_change, sizeof(ble_profile_change) / sizeof(note_t), false);
        
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