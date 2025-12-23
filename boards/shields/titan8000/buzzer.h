/*
 * Copyright (c) 2025 monksoffunk
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/drivers/pwm.h>

#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7   2794
#define NOTE_FS7  2960
#define NOTE_G7   3136
#define NOTE_GS7  3322
#define NOTE_A7   3520
#define NOTE_AS7  3729
#define NOTE_B7   3951
#define NOTE_C8   4186
#define NOTE_CS8  4435
#define NOTE_D8   4699
#define NOTE_DS8  4978
#define NOTE_E8   5274
#define NOTE_F8   5588
#define NOTE_FS8  5920
#define NOTE_G8   6272
#define NOTE_GS8  6645
#define NOTE_A8   7040
#define NOTE_AS8  7459
#define NOTE_B8   7902
#define NOTE_REST 0

typedef struct {
  uint16_t freq;
  uint16_t duration;
} note_t;

typedef void (*buzzer_voice_fn_t)(
    const struct pwm_dt_spec *pwm,
    uint32_t freq_hz,
    uint32_t duration_ms
);

struct buzzer_request {
    buzzer_voice_fn_t voice;
    uint32_t freq_hz;
    uint32_t duration_ms;
};

void buzzer_play_melody(const note_t *melody, uint32_t length, bool loop);
void buzzer_stop_melody(void);
bool buzzer_is_playing(void);
void buzzer_toggle_enable(void);
void titan8000_play_soft_off_tone(void);


/**
 * @brief Toggles the keypress beep feature on or off.
 *
 * When enabled, the buzzer will emit a beep sound on each keypress.
 * Calling this function switches the current state (enabled/disabled).
 */
void buzzer_toggle_keypress_beep(void);

/**
 * @brief Checks if the keypress beep feature is enabled.
 *
 * @return true if keypress beep is enabled, false otherwise.
 */
bool buzzer_is_keypress_beep_enabled(void);