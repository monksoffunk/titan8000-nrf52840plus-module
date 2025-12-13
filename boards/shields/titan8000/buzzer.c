#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "buzzer.h"

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_REGISTER(buzzer, CONFIG_ZMK_LOG_LEVEL);

#define BUZZER_NODE DT_CHILD(DT_PATH(buzzers), buzzer)
static const struct pwm_dt_spec buzzer_pwm = PWM_DT_SPEC_GET(BUZZER_NODE);
static const note_t *current_melody = NULL;
static uint32_t melody_length = 0;
static uint32_t current_index = 0;
static bool melody_loop = false;
static struct k_timer melody_timer;

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
		    LOG_ERR("========================================");
			LOG_ERR("PWM BUZZER not ready!!");
		    LOG_ERR("========================================");
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

    k_timer_init(&melody_timer, melody_timer_callback, NULL);
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

static int buzzer_init(void)
{
    LOG_ERR("========================================");
    LOG_ERR("BUZZER MODULE INITIALIZED");
    if (!device_is_ready(buzzer_pwm.dev)) {
        LOG_ERR("PWM Device NOT READY!");
        return -ENODEV;
    }
    LOG_ERR("PWM Device: %s", buzzer_pwm.dev->name);
    LOG_ERR("PWM Ready: YES");
    LOG_ERR("========================================");
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

    // キーが押された時のみ音を鳴らす（離された時は鳴らさない）
    if (ev->state) {
        LOG_ERR("KEY PRESSED at position %d", ev->position);
        buzzer_beep(1000, 50);  // 1kHz, 50ms
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(buzzer, buzzer_keypress_listener);
ZMK_SUBSCRIPTION(buzzer, zmk_position_state_changed);