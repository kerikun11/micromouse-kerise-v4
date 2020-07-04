#pragma once

#include <esp32-hal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

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
    playList = xQueueCreate(/* uxQueueLength = */ 20, sizeof(enum Music));
  }
  bool init(int8_t pin, uint8_t channel) {
    this->channel = channel;
    ledcSetup(channel, 880, 4);
    ledcAttachPin(pin, channel);
    xTaskCreate([](void *arg) { static_cast<decltype(this)>(arg)->task(); },
                "Buzzer", 4096, this, 1, NULL);
    return true;
  }
  void play(const enum Music music) { xQueueSendToBack(playList, &music, 0); }

private:
  uint8_t channel;
  QueueHandle_t playList;

  void sound(const note_t note, uint8_t octave, uint32_t time_ms) {
    ledcWriteNote(channel, note, octave);
    vTaskDelay(pdMS_TO_TICKS(time_ms));
  }
  void mute(uint32_t time_ms = 400) {
    ledcWrite(channel, 0);
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
