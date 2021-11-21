/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief MicroMouse KERISE
 * @date 2019-04-02
 */
#include "machine/machine.h"

void driveTask(void *arg);
void printTask(void *arg);

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(2000000);
  std::cout << std::endl << "I'm KERISE v" << KERISE_SELECT << "." << std::endl;
  Machine::init();
  xTaskCreate(printTask, "print", 4096, NULL, 2, NULL);
  xTaskCreate(driveTask, "drive", 4096, NULL, 2, NULL);
}

void loop() { vTaskDelay(portMAX_DELAY); }

void printTask(void *arg) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(99));
    // tof.print();
    // wd.print();
    // enc.csv();
  }
}

void driveTask(void *arg) {
  // Machine::driveAutomatically();
  while (1) {
    int mode = ui.waitForSelect(16);
    switch (mode) {
    case 0: /* 迷路走行 */
      Machine::driveNormally();
      break;
    case 1: /* パラメータの相対設定 */
      Machine::selectParamManually();
      break;
    case 2: /* パラメータの絶対設定 */
      Machine::selectParamPreset();
      break;
    case 3: /* 斜め走行などの設定 */
      Machine::selectRunConfig();
      break;
    case 4: /* ファンの設定 */
      Machine::selectFanGain();
      break;
    case 5: /* 迷路データの復元 */
      Machine::restore();
      break;
    case 6: /* データ消去 */
      Machine::reset();
      break;
    case 7: /* 宴会芸 */
      Machine::partyStunt();
      break;
    case 8: /* 壁センサキャリブレーション */
      Machine::wallCalibration();
      break;
    case 9: /* プチコン */
      Machine::petitcon();
      break;
    case 10: /* 迷路の表示 */
      mr.print();
      break;
    case 11: /* ゴール区画の設定 */
      Machine::setGoalPositions();
      break;
    case 12:
      Machine::position_recovery();
      // Machine::sysid();
      break;
    case 13:
      // Machine::pidTuner();
      // Machine::encoder_test();
      // Machine::accel_test();
      Machine::wall_attach_test();
      break;
    case 14: /* テスト */
      Machine::slalom_test();
      break;
    case 15: /* ログの表示 */
      lgr.print();
      break;
    }
  }
}
