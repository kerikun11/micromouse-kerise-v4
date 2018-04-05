/**
  KERISE
  Author:  kerikun11 (Github: kerikun11)
  Date:    2017.10.25
*/

#include <WiFi.h>
#include <SPIFFS.h>

#include "global.h"

void task(void* arg) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  float prev[2] = {0, 0};
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //    printf("0,100,%f\n", (enc.position(1) - prev[1]) * 1000); prev[0] = enc.position(0); prev[1] = enc.position(1);
    //    enc.csv();
    //    ref.csv();
    //    tof.csv();
    //    ref.print(); vTaskDelayUntil(&xLastWakeTime, 99 / portTICK_RATE_MS);
    wd.print(); vTaskDelayUntil(&xLastWakeTime, 99 / portTICK_RATE_MS);
    //    imu.print(); vTaskDelayUntil(&xLastWakeTime, 99 / portTICK_RATE_MS);
    //    tof.print(); vTaskDelayUntil(&xLastWakeTime, 99 / portTICK_RATE_MS);

    //    printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
    //           sc.target.trans,
    //           sc.actual.trans,
    //           sc.enconly.trans,
    //           sc.acconly.trans,
    //           sc.Kp * sc.proportional.trans,
    //           sc.Ki * sc.integral.trans,
    //           sc.Kd * sc.differential.trans,
    //           sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans + sc.Kd * sc.differential.trans
    //          );
  }
}

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(2000000);
  log_i("\n**************** KERISEv5 ****************\n");
  if (!bz.begin()) bz.play(Buzzer::ERROR);
  if (!led.begin(true)) bz.play(Buzzer::ERROR);
  ui.batteryCheck();
  bz.play(Buzzer::BOOT);
  delay(200);
  pinMode(AS5048A_CS_PIN, INPUT_PULLUP);
  pinMode(ICM20602_L_CS_PIN, INPUT_PULLUP);
  pinMode(ICM20602_R_CS_PIN, INPUT_PULLUP);

  if (!SPIFFS.begin(false)) bz.play(Buzzer::ERROR);
  if (!imu.begin(ICM20602_SPI_HOST, ICM20602_CS_PINS, true, ICM20602_SCLK_PIN, ICM20602_MISO_PIN, ICM20602_MOSI_PIN, ICM20602_SPI_DMA_CHAIN)) bz.play(Buzzer::ERROR);
  if (!enc.begin(AS5048A_SPI_HOST, AS5048A_CS_PIN, false, AS5048A_SCLK_PIN, AS5048A_MISO_PIN, AS5048A_MOSI_PIN, AS5048A_SPI_DMA_CHAIN)) bz.play(Buzzer::ERROR);

  if (!ref.begin()) bz.play(Buzzer::ERROR);
  if (!tof.begin(false)) bz.play(Buzzer::ERROR);
  if (!wd.begin()) bz.play(Buzzer::ERROR);
  em.begin();
  ec.begin();

  xTaskCreate(task, "test", 4096, NULL, 0, NULL); // debug output
  xTaskCreate(timeKeepTask, "TimeKeep", 4096, NULL, 0, NULL); // debug output
}

void loop() {
  normal_drive();
  //  position_test();
  //  step_test();
  //  trapizoid_test();
  //  straight_test();
  //  turn_test();
}

void timeKeepTask(void* arg) {
  const int searchig_time_ms = 3 * 60 * 1000;
  while (millis() < searchig_time_ms) delay(1000);
  bz.play(Buzzer::LOW_BATTERY);
  ms.forceBackToStart();
  while (1) delay(1000);
}

void normal_drive() {
  int mode = ui.waitForSelect(16);
  switch (mode) {
    //* 走行
    case 0:
      bz.play(Buzzer::SUCCESSFUL);
      if (!ui.waitForCover()) return;
      led = 9;
      ms.start();
      btn.flags = 0;
      while (ms.isRunning() && !btn.pressed) delay(100);
      bz.play(Buzzer::CANCEL);
      btn.flags = 0;
      ms.terminate();
      break;
    //* 走行パラメータの選択 & 走行
    case 1: {
        int preset = ui.waitForSelect(16);
        if (preset < 0) return;
        switch (preset) {
          case 0:  fr.runParameter = FastRun::RunParameter(0.8,  900, 6000, 6000); break;
          case 1:  fr.runParameter = FastRun::RunParameter(0.8, 1200, 7200, 7200); break;
          case 2:  fr.runParameter = FastRun::RunParameter(0.8, 1500, 9000, 9000); break;
          case 3:  fr.runParameter = FastRun::RunParameter(0.8, 2100, 12000, 12000); break;

          case 4:  fr.runParameter = FastRun::RunParameter(0.9,  900, 6000, 6000); break;
          case 5:  fr.runParameter = FastRun::RunParameter(0.9, 1200, 7200, 7200); break;
          case 6:  fr.runParameter = FastRun::RunParameter(0.9, 1500, 9000, 9000); break;
          case 7:  fr.runParameter = FastRun::RunParameter(0.9, 2100, 12000, 12000); break;

          case 8:  fr.runParameter = FastRun::RunParameter(1.0,  900, 6000, 6000); break;
          case 9:  fr.runParameter = FastRun::RunParameter(1.0, 1200, 7200, 7200); break;
          case 10: fr.runParameter = FastRun::RunParameter(1.0, 1500, 9000, 9000); break;
          case 11: fr.runParameter = FastRun::RunParameter(1.0, 2100, 12000, 12000); break;

          case 12: fr.runParameter = FastRun::RunParameter(1.1,  900, 6000, 6000); break;
          case 13: fr.runParameter = FastRun::RunParameter(1.1, 1200, 7200, 7200); break;
          case 14: fr.runParameter = FastRun::RunParameter(1.1, 1500, 9000, 9000); break;
          case 15: fr.runParameter = FastRun::RunParameter(1.1, 2100, 12000, 12000); break;
        }
      }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    //* 速度の設定
    case 2: {
        int value;
        for (int i = 0; i < 1; i++) bz.play(Buzzer::SHORT);
        value = ui.waitForSelect(16);
        if (value < 0) return;
        const float curve_gain = 0.1f * value;
        for (int i = 0; i < 2; i++) bz.play(Buzzer::SHORT);
        value = ui.waitForSelect(16);
        if (value < 0) return;
        const float v_max = 300.0f * value;
        for (int i = 0; i < 3; i++) bz.play(Buzzer::SHORT);
        value = ui.waitForSelect(16);
        if (value < 0) return;
        const float accel = 1000.0f * value;
        fr.runParameter = FastRun::RunParameter(curve_gain,  v_max, accel, accel);
      }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    //* 壁制御の設定
    case 3: {
        int value = ui.waitForSelect(16);
        if (value < 0) return;
        fr.wallAvoidFlag = value & 0x01;
        fr.wallAvoid45Flag = value & 0x02;
        fr.wallCutFlag = value & 0x04;
        fr.V90Enabled = value & 0x08;
      }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    //* ファンの設定
    case 4: {
        fan.drive(fr.fanDuty);
        delay(100);
        fan.drive(0);
        int value = ui.waitForSelect(11);
        if (value < 0) return;
        fr.fanDuty = 0.1f * value;
        fan.drive(fr.fanDuty);
        ui.waitForSelect(1);
        fan.drive(0);
      }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    //* 迷路データの復元
    case 7:
      bz.play(Buzzer::MAZE_RESTORE);
      if (!ms.restore()) {
        bz.play(Buzzer::ERROR);
        return;
      }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    //* 前壁キャリブレーション
    case 8:
      led = 6;
      if (!ui.waitForCover(true)) return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibrationFront();
      bz.play(Buzzer::CANCEL);
      break;
    //* 横壁キャリブレーション
    case 9:
      led = 9;
      if (!ui.waitForCover()) return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      wd.calibrationSide();
      bz.play(Buzzer::CANCEL);
      break;
    //* 前壁補正データの保存
    case 10:
      if (!ui.waitForCover()) return;
      if (wd.backup()) {
        bz.play(Buzzer::SUCCESSFUL);
      } else {
        bz.play(Buzzer::ERROR);
      }
      break;
    //* ゴール区画の設定
    case 11: {
        for (int i = 0; i < 2; i++) bz.play(Buzzer::SHORT);
        int value = ui.waitForSelect(4);
        if (value < 0) return;
        switch (value) {
          case 0: ms.set_goal({Vector(45, 39)}); break; //4238 * 3638
          case 1: ms.set_goal({Vector(1, 0)}); break;
          case 2: ms.set_goal({Vector(4, 4), Vector(4, 5), Vector(5, 4), Vector(5, 5)}); break;
          case 3: ms.set_goal({Vector(15, 15)}); break;
        }
        bz.play(Buzzer::SUCCESSFUL);
      }
      break;
    //* マス直線
    case 12:
      if (!ui.waitForCover(true)) return;
      delay(1000);
      bz.play(Buzzer::CONFIRM);
      imu.calibration();
      bz.play(Buzzer::CANCEL);
      sc.enable();
      straight_x(9 * 90 - 6 - MACHINE_TAIL_LENGTH, 300, 0);
      sc.disable();
      break;
    //* リセット
    case 14:
      mt.drive(100, 100);
      delay(2000);
      mt.free();
      break;
    //* リセット
    case 15:
      ESP.restart();
      break;
  }
}

void position_test() {
  if (!ui.waitForCover()) return;
  delay(1000);
  bz.play(Buzzer::SELECT);
  imu.calibration();
  bz.play(Buzzer::CANCEL);
  sc.enable();
  sc.set_target(0, 0);
}

void trapizoid_test() {
  while (1) {
    if (btn.pressed) {
      btn.flags = 0;
      bz.play(Buzzer::CONFIRM);
      break;
    }
    if (btn.long_pressing_1) {
      btn.flags = 0;
      bz.play(Buzzer::SELECT);
      lg.print();
    }
    delay(100);
  }
  if (!ui.waitForCover()) return;
  delay(1000);
  imu.calibration();
  fan.drive(0.5); delay(500);
  lg.start();
  sc.enable();
  const float accel = 12000;
  const float decel = 6000;
  const float v_max = 1800;
  const float v_start = 0;
  float T = 1.5f * (v_max - v_start) / accel;
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (int ms = 0; ms / 1000.0f < T; ms++) {
    float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
    sc.set_target(velocity_a, 0);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
  bz.play(Buzzer::SELECT);
  delay(200);
  bz.play(Buzzer::SELECT);
  for (float v = v_max; v > 0; v -= decel / 1000) {
    sc.set_target(v, 0);
    delay(1);
  }
  sc.set_target(0, 0);
  delay(150);
  lg.end();
  bz.play(Buzzer::CANCEL);
  sc.disable();
  fan.drive(0);
}

void step_test() {
  while (1) {
    if (btn.pressed) {
      btn.flags = 0;
      bz.play(Buzzer::CONFIRM);
      break;
    }
    if (btn.long_pressing_1) {
      btn.flags = 0;
      bz.play(Buzzer::SELECT);
      lg.print();
    }
    delay(100);
  }
  if (!ui.waitForCover()) return;
  delay(1000);
  imu.calibration();
  fan.drive(0.5); delay(500);
  sc.enable();
  lg.start();
  const float value = 300;
  mt.drive(value, value);
  delay(1000);
  mt.drive(0, 0);
  lg.end();
  delay(1000);
  sc.disable();
  fan.drive(0);
}

void straight_test() {
  if (!ui.waitForCover()) return;
  delay(1000);
  bz.play(Buzzer::SELECT);
  imu.calibration();
  sc.enable();
  fan.drive(0.3);
  delay(500);
  straight_x(8 * 90 - 6 - MACHINE_TAIL_LENGTH, 1200, 0);
  sc.set_target(0, 0);
  fan.drive(0);
  delay(100);
  sc.disable();
}

void turn_test() {
  while (1) {
    if (btn.pressed) {
      btn.flags = 0;
      bz.play(Buzzer::CONFIRM);
      break;
    }
    if (btn.long_pressing_1) {
      btn.flags = 0;
      bz.play(Buzzer::SELECT);
      lg.print();
    }
    delay(100);
  }
  if (!ui.waitForCover()) return;
  delay(1000);
  imu.calibration();
  bz.play(Buzzer::CONFIRM);
  lg.start();
  sc.enable();
  turn(-10 * 2 * PI);
  turn(10 * 2 * PI);
  sc.disable();
  bz.play(Buzzer::CANCEL);
  lg.end();
}

#define TEST_LOOK_AHEAD 12
#define TEST_PROP_GAIN  20

void straight_x(const float distance, const float v_max, const float v_end) {
  const float accel = 1500;
  const float decel = 1200;
  int ms = 0;
  const float v_start = sc.actual.trans;
  const float T = 1.5f * (v_max - v_start) / accel;
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    Position cur = sc.position;
    if (v_end >= 1.0f && cur.x > distance - TEST_LOOK_AHEAD) break;
    if (v_end < 1.0f && cur.x > distance - 1.0f) break;
    float extra = distance - cur.x;
    float velocity_a = v_start + (v_max - v_start) * 6.0f * (-1.0f / 3 * pow(ms / 1000.0f / T, 3) + 1.0f / 2 * pow(ms / 1000.0f / T, 2));
    float velocity_d = sqrt(2 * decel * fabs(extra) + v_end * v_end);
    float velocity = v_max;
    if (velocity > velocity_d) velocity = velocity_d;
    if (ms / 1000.0f < T && velocity > velocity_a) velocity = velocity_a;
    float theta = atan2f(-cur.y, TEST_LOOK_AHEAD * (1 + velocity / 600)) - cur.theta;
    sc.set_target(velocity, TEST_PROP_GAIN * theta);
    //    if (avoid) wall_avoid();
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    ms++;
    printf("%f\t[%c %c]\n", cur.x + 6 + MACHINE_TAIL_LENGTH, wd.wall[0] ? 'X' : '_', wd.wall[1] ? 'X' : '_');
  }
  sc.set_target(v_end, 0);
}

void turn(const float angle) {
  const float speed = 3.6 * M_PI;
  const float accel = 36 * M_PI;
  const float decel = 36 * M_PI;
  const float back_gain = 1.0f;
  int ms = 0;
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    if (fabs(sc.actual.rot) > speed) break;
    float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
    if (angle > 0) {
      sc.set_target(-delta * back_gain, ms / 1000.0f * accel);
    } else {
      sc.set_target(-delta * back_gain, -ms / 1000.0f * accel);
    }
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    ms++;
  }
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    float extra = angle - sc.position.theta;
    if (fabs(sc.actual.rot) < 0.1 && fabs(extra) < 0.1) break;
    float target_speed = sqrt(2 * decel * fabs(extra));
    float delta = sc.position.x * cos(-sc.position.theta) - sc.position.y * sin(-sc.position.theta);
    target_speed = (target_speed > speed) ? speed : target_speed;
    if (extra > 0) {
      sc.set_target(-delta * back_gain, target_speed);
    } else {
      sc.set_target(-delta * back_gain, -target_speed);
    }
  }
  sc.set_target(0, 0);
  //  updateOrigin(Position(0, 0, angle));
  //  printPosition("Turn End");
}

