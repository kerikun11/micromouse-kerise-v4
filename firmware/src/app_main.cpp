/**
 * @file app_main.cpp
 * @author Ryotaro Onuki (kerikun11+github@gmail.com)
 * @brief KERISE
 * @date 2019-04-02
 */
#include "Machine.h"
#include <iostream>

void driveTask(void *arg);
void printTask(void *arg);
void timeKeepTask(void *arg);

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(2000000);
  std::cout << std::endl << "******** KERISE ********" << std::endl;
  Machine::init();
  xTaskCreate(printTask, "print", 4096, NULL, 2, NULL);
  // xTaskCreate(timeKeepTask, "TimeKeep", 4096, NULL, 2, NULL);
  xTaskCreate(driveTask, "drive", 4096, NULL, 2, NULL);
}

void loop() { vTaskDelay(portMAX_DELAY); }

void printTask(void *arg) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(99));
    // ref.csv();
    // wd.print();
    // tof.print();
    // enc.print();
    // std::cout << enc.getPulses(0) << "," << enc.getPulses(1) << std::endl;
    // std::cout << "0,600,";
    // std::cout << sc.enc_v.wheel[0] << "," << sc.enc_v.wheel[1] << std::endl;
  }
}

void timeKeepTask(void *arg) {
  const int searching_time_ms = 1 * 60 * 1000;
  while (millis() < searching_time_ms)
    delay(1000);
  bz.play(Buzzer::TIMEOUT);
  mr.setForceBackToStart();
  vTaskDelay(portMAX_DELAY);
}

void driveTask(void *arg) {
  // Machine::driveAutomatically();
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1));
    int mode = ui.waitForSelect(16);
    switch (mode) {
    case 0: /* 迷路走行 */
      Machine::driveNormally();
      break;
    case 1: /* マニュアル走行パラメータの設定 */
      Machine::selectParamManually();
      break;
    case 2: /* プリセット走行パラメータ */
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
      std::cout << "sc.position: " << sc.position << std::endl;
      ref.print();
      tof.print();
      enc.print();
      imu.print();
      wd.print();
      mr.print();
      break;
    case 11: /* ゴール区画の設定 */
      Machine::setGoalPositions();
      break;
    case 13:
      Machine::position_recovery();
      break;
    case 14: /* テスト */
      // Machine::accel_test();
      Machine::slalom_test();
      // Machine::sysid();
      // Machine::enc_id();
      break;
    case 15: /* ログの表示 */
      lgr.print();
      break;
    }
  }
}
