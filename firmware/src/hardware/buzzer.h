/**
 * @file buzzer.h
 * @brief Buzzer Driver
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include "app_log.h"
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace hardware {

class Buzzer {
public:
  enum Music : uint8_t {
    SELECT,
    CANCEL,
    CONFIRM,
    SUCCESSFUL,
    ERROR,
    SHORT6,
    SHORT7,
    SHORT8,
    SHORT9,
    BOOT,
    SHUTDOWN,
    TIMEOUT,
    EMERGENCY,
    COMPLETE,
    MAZE_BACKUP,
    MAZE_RESTORE,
    CALIBRATION,
    AEBS,
  };

public:
  Buzzer() {
    playList = xQueueCreate(/* uxQueueLength = */ 10, sizeof(enum Music));
  }
  bool init(gpio_num_t pin, ledc_channel_t channel, ledc_timer_t timer) {
    this->channel = channel;
    this->timer = timer;
    // LEDC Timer
    static ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = mode;                  // timer mode
    ledc_timer.duty_resolution = LEDC_TIMER_8_BIT; // resolution of PWM duty
    ledc_timer.timer_num = timer;                  // timer index
    ledc_timer.freq_hz = 5000;                     // frequency of PWM signal
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    // LEDC Channel
    static ledc_channel_config_t ledc_channel;
    ledc_channel.channel = channel;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = pin;
    ledc_channel.speed_mode = mode;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    // Player Task
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "Buzzer", 4096, this, 1, NULL);
    return true;
  }
  void play(const enum Music music) { xQueueSendToBack(playList, &music, 0); }

private:
  static constexpr const char *tag = "Buzzer";
  ledc_channel_t channel;
  ledc_timer_t timer;
  ledc_mode_t mode = LEDC_HIGH_SPEED_MODE;
  QueueHandle_t playList;
  typedef enum {
    NOTE_C,
    NOTE_Cs,
    NOTE_D,
    NOTE_Eb,
    NOTE_E,
    NOTE_F,
    NOTE_Fs,
    NOTE_G,
    NOTE_Gs,
    NOTE_A,
    NOTE_Bb,
    NOTE_B,
    NOTE_MAX,
  } note_t;

  void write_note(note_t note, uint8_t octave) {
    static const uint32_t noteFrequencyBase[12] = {
        // C    C#     D    Eb     E     F    F#     G    G#     A    Bb     B
        4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902};
    if (octave > 8 || note >= NOTE_MAX) {
      app_loge << "out of range" << std::endl;
      return;
    }
    uint32_t freq = noteFrequencyBase[note] / (1 << (8 - octave));
    ledc_set_freq(mode, timer, freq);
    ledc_set_duty(mode, channel, 8);
    ledc_update_duty(mode, channel);
  }
  void write(uint32_t duty) {
    ledc_set_duty(mode, channel, 0);
    ledc_update_duty(mode, channel);
  }
  void sound(const note_t note, uint8_t octave, uint32_t time_ms) {
    write_note(note, octave);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
  }
  void mute(uint32_t time_ms = 400) {
    write(0);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
  }
  void task() {
    while (1) {
      Music music;
      xQueueReceive(playList, &music, portMAX_DELAY);
      switch (music) {
      case SELECT:
        sound(NOTE_C, 6, 100);
        mute(100);
        break;
      case CANCEL:
        sound(NOTE_E, 6, 100);
        sound(NOTE_C, 6, 100);
        mute(100);
        break;
      case CONFIRM:
        sound(NOTE_C, 6, 100);
        sound(NOTE_E, 6, 100);
        mute(100);
        break;
      case SUCCESSFUL:
        sound(NOTE_C, 6, 100);
        sound(NOTE_E, 6, 100);
        sound(NOTE_G, 6, 100);
        mute(100);
        break;
      case ERROR:
        for (int i = 0; i < 6; i++) {
          sound(NOTE_C, 7, 100);
          sound(NOTE_E, 7, 100);
        }
        mute();
        break;
      case BOOT:
        sound(NOTE_B, 5, 200);
        sound(NOTE_E, 6, 400);
        sound(NOTE_Fs, 6, 200);
        sound(NOTE_B, 6, 600);
        mute();
        break;
      case SHUTDOWN:
        sound(NOTE_Gs, 6, 200);
        sound(NOTE_Eb, 6, 200);
        sound(NOTE_Gs, 5, 200);
        sound(NOTE_Bb, 5, 600);
        mute();
        break;
      case TIMEOUT:
        sound(NOTE_C, 7, 400);
        mute(200);
        sound(NOTE_C, 7, 400);
        mute(200);
        sound(NOTE_C, 7, 400);
        mute(200);
        break;
      case EMERGENCY:
        sound(NOTE_C, 6, 100);
        sound(NOTE_F, 6, 100);
        mute(100);
        sound(NOTE_F, 6, 75);
        mute(25);

        sound(NOTE_F, 6, 176);
        sound(NOTE_E, 6, 176);
        sound(NOTE_D, 6, 176);
        sound(NOTE_C, 6, 200);
        mute(100);
        break;
      case COMPLETE:
        sound(NOTE_C, 6, 100);
        sound(NOTE_D, 6, 100);
        sound(NOTE_E, 6, 100);
        sound(NOTE_F, 6, 100);
        sound(NOTE_G, 6, 100);
        sound(NOTE_A, 6, 100);
        sound(NOTE_B, 6, 100);
        sound(NOTE_C, 7, 100);
        mute(100);
        break;
      case SHORT6:
        sound(NOTE_C, 6, 50);
        mute(50);
        break;
      case SHORT7:
        sound(NOTE_C, 7, 50);
        mute(50);
        break;
      case SHORT8:
        sound(NOTE_C, 8, 50);
        mute(50);
        break;
      case SHORT9:
        sound(NOTE_G, 8, 50);
        mute(50);
        break;
      case MAZE_BACKUP:
        sound(NOTE_G, 7, 100);
        sound(NOTE_E, 7, 100);
        sound(NOTE_C, 7, 100);
        mute(100);
        break;
      case MAZE_RESTORE:
        sound(NOTE_C, 7, 100);
        sound(NOTE_E, 7, 100);
        sound(NOTE_G, 7, 100);
        mute(100);
        break;
      case CALIBRATION:
        sound(NOTE_C, 7, 100);
        sound(NOTE_E, 7, 100);
        mute(100);
        break;
      case AEBS:
        for (int i = 0; i < 8; ++i) {
          sound(NOTE_E, 7, 150);
          mute(50);
        }
        mute(200);
        break;
      default:
        sound(NOTE_C, 4, 1000);
        mute();
        break;
      }
    }
  }
};

}; // namespace hardware
