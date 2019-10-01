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
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  std::cout << mac[0] << std::endl;
  Machine::init();
  xTaskCreate(printTask, "print", 4096, NULL, 2, NULL);
  // xTaskCreate(timeKeepTask, "TimeKeep", 4096, NULL, 2, NULL);
  xTaskCreate(driveTask, "drive", 4096, NULL, 2, NULL);
}

void loop() { delay(1000); }

void printTask(void *arg) {
  // Accumulator<float, 100> p[2];
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    // for (int i = 0; i < 2; ++i)
    //   p[i].push(enc.position(i));
    // std::cout << p[0][0] - p[0][p[0].size() - 1] << "\t"
    //           << p[1][0] - p[1][p[0].size() - 1] << std::endl;
    // ref.csv();
    // tof.csv();
    // wd.print();
    // wd.printDiff();
    // wd.csv();
    // enc.csv();
    // tof.print();
    // enc.print();
    vTaskDelayUntil(&xLastWakeTime, 99 / portTICK_RATE_MS);
  }
}

void timeKeepTask(void *arg) {
  const int searching_time_ms = 1 * 60 * 1000;
  while (millis() < searching_time_ms)
    delay(1000);
  bz.play(Buzzer::LOW_BATTERY);
  mr.setForceBackToStart();
  vTaskDelay(portMAX_DELAY);
}

void driveTask(void *arg) {
  Machine::driveAutomatically();
  while (1) {
    delay(1);
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
    case 3: { /* 壁制御，斜め走行の設定 */
      int value = ui.waitForSelect(16);
      if (value < 0)
        break;
      sr.rp_search.diag_enabled = value & 0x01;
      sr.rp_fast.diag_enabled = value & 0x02;
      sr.rp_search.front_wall_fix_enabled = value & 0x04;
      sr.rp_fast.front_wall_fix_enabled = value & 0x04;
      sr.rp_search.wall_avoid_enabled = value & 0x08;
      sr.rp_fast.wall_avoid_enabled = value & 0x08;
    }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    case 4: /* ファンの設定 */
      Machine::selectFanGain();
      break;
    case 5: /* 迷路データの復元 */
      if (!mr.restore())
        bz.play(Buzzer::ERROR);
      else
        bz.play(Buzzer::MAZE_RESTORE);
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
    case 14: /* テスト */
      Machine::accel_test();
      // Machine::sysid();
      break;
    case 15: /* ログの表示 */
      lgr.print();
      break;
    }
  }
}
