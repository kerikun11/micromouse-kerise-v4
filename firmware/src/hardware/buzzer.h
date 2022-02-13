/**
 * @file buzzer.h
 * @brief Buzzer Driver for MicroMouse KERISE
 * @copyright Copyright 2021 Ryotaro Onuki <kerikun11+github@gmail.com>
 * @date 2021-11-21
 */
#pragma once

#include <driver/ledc.h>
#include <esp_log.h>
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
    COMPLETE,
    BOOT,
    SHUTDOWN,
    TIMEOUT,
    EMERGENCY,
    MAZE_BACKUP,
    MAZE_RESTORE,
    CALIBRATION,
    AEBS,
    SHORT6,
    SHORT7,
    SHORT8,
    SHORT9,
    MUSIC_MAX,
  };

public:
  static Buzzer *get_instance() {
    static Buzzer *instatnce = new Buzzer();
    return instatnce;
  }
  bool init(gpio_num_t gpio_num, ledc_channel_t channel, ledc_timer_t timer) {
    this->channel = channel;
    this->timer = timer;
    // LEDC Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = mode,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = timer,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    // LEDC Channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num,
        .speed_mode = mode,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    // Player Task
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "Buzzer", /* stack */ 4096, this, /* priority */ 1, NULL);
    // Ending
    initialized = true;
    return true;
  }
  bool is_initialized() const { return initialized; }
  void play(const enum Music music, TickType_t xTicksToWait = 0) {
    xQueueSendToBack(playList, &music, xTicksToWait);
  }

protected:
  static constexpr const char *TAG = "Buzzer";
  bool initialized = false;
  QueueHandle_t playList;
  ledc_channel_t channel;
  ledc_timer_t timer;
  ledc_mode_t mode = LEDC_HIGH_SPEED_MODE;
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

  Buzzer() {
    playList = xQueueCreate(/* uxQueueLength = */ 10, sizeof(enum Music));
  }
  void task() {
    vTaskDelay(pdMS_TO_TICKS(1)); //< for first note drop bug avoidance
    while (1) {
      enum Music music;
      xQueueReceive(playList, &music, portMAX_DELAY);
      play_music(music);
      mute(0);
    }
  }
  void ledc_write_note(note_t note, uint8_t octave) {
    static const uint32_t noteFrequencyBase[12] = {
        // C    C#     D    Eb     E     F    F#     G    G#     A    Bb     B
        4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902};
    if (octave > 8 || note >= NOTE_MAX) {
      ESP_LOGE(TAG, "note out of range. octave: %d, note: %d", octave, note);
      return;
    }
    uint32_t freq = noteFrequencyBase[note] / (1 << (8 - octave));
    ledc_set_freq(mode, timer, freq);
    ledc_set_duty(mode, channel, 4); //< duty in [0, 2^duty_resolution]
    ledc_update_duty(mode, channel);
  }
  void note(const note_t note, uint8_t octave, uint32_t time_ms) {
    ledc_write_note(note, octave);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
  }
  void mute(uint32_t time_ms = 200) {
    ledc_set_duty(mode, channel, 0);
    ledc_update_duty(mode, channel);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
  }
  void play_music(enum Music music) {
    switch (music) {
    case SELECT:
      note(NOTE_C, 6, 100);
      mute(100);
      break;
    case CANCEL:
      note(NOTE_E, 6, 100);
      note(NOTE_C, 6, 100);
      mute(100);
      break;
    case CONFIRM:
      note(NOTE_C, 6, 100);
      note(NOTE_E, 6, 100);
      mute(100);
      break;
    case SUCCESSFUL:
      note(NOTE_C, 6, 100);
      note(NOTE_E, 6, 100);
      note(NOTE_G, 6, 100);
      mute();
      break;
    case ERROR:
      for (int i = 0; i < 4; i++) {
        note(NOTE_C, 7, 100);
        note(NOTE_E, 7, 100);
      }
      mute();
      break;
    case COMPLETE:
      note(NOTE_C, 6, 100);
      note(NOTE_D, 6, 100);
      note(NOTE_E, 6, 100);
      note(NOTE_F, 6, 100);
      note(NOTE_G, 6, 100);
      note(NOTE_A, 6, 100);
      note(NOTE_B, 6, 100);
      note(NOTE_C, 7, 100);
      mute();
      break;
    case BOOT:
      note(NOTE_B, 5, 200);
      note(NOTE_E, 6, 400);
      note(NOTE_Fs, 6, 200);
      note(NOTE_B, 6, 600);
      mute();
      break;
    case SHUTDOWN:
      note(NOTE_Gs, 6, 200);
      note(NOTE_Eb, 6, 200);
      note(NOTE_Gs, 5, 200);
      note(NOTE_Bb, 5, 600);
      mute();
      break;
    case TIMEOUT:
      note(NOTE_C, 7, 400);
      mute(200);
      note(NOTE_C, 7, 400);
      mute(200);
      note(NOTE_C, 7, 400);
      mute(200);
      break;
    case EMERGENCY:
      note(NOTE_C, 6, 100);
      note(NOTE_F, 6, 100);
      mute(100);
      note(NOTE_F, 6, 75);
      mute(25);
      note(NOTE_F, 6, 176);
      note(NOTE_E, 6, 176);
      note(NOTE_D, 6, 176);
      note(NOTE_C, 6, 200);
      mute();
      break;
    case MAZE_BACKUP:
      note(NOTE_G, 7, 100);
      note(NOTE_E, 7, 100);
      note(NOTE_C, 7, 100);
      mute();
      break;
    case MAZE_RESTORE:
      note(NOTE_C, 7, 100);
      note(NOTE_E, 7, 100);
      note(NOTE_G, 7, 100);
      mute();
      break;
    case CALIBRATION:
      note(NOTE_C, 7, 100);
      note(NOTE_E, 7, 100);
      mute();
      break;
    case AEBS:
      for (int i = 0; i < 8; ++i) {
        note(NOTE_E, 7, 150);
        mute(50);
      }
      break;
    case SHORT6:
      note(NOTE_C, 6, 50);
      mute(50);
      break;
    case SHORT7:
      note(NOTE_C, 7, 50);
      mute(50);
      break;
    case SHORT8:
      note(NOTE_C, 8, 50);
      mute(50);
      break;
    case SHORT9:
      note(NOTE_G, 8, 50);
      mute(50);
      break;
    default:
      note(NOTE_C, 4, 1000);
      mute();
      break;
    }
  }
};

}; // namespace hardware
