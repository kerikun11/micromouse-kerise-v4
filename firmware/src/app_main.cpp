#include "Machine.h"
#include <iostream>

void driveTask(void *arg);
void printTask(void *arg);
void timeKeepTask(void *arg);

void setup() {
  Serial.begin(2000000);
  WiFi.mode(WIFI_OFF);
  std::cout << std::endl;
  std::cout << "**************** KERISE v4 ****************" << std::endl;
  Machine::init();
  xTaskCreate(printTask, "print", 4096, NULL, 2, NULL);
  xTaskCreate(timeKeepTask, "TimeKeep", 4096, NULL, 2, NULL);
  xTaskCreate(driveTask, "drive", 4096, NULL, 2, NULL);
}

void loop() { delay(1000); }

void printTask(void *arg) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    // enc.print();
    // ref.csv();
    // tof.csv(); delay(100);
    // imu.print();
    // wd.print();
    // wd.csv();
  }
}

void timeKeepTask(void *arg) {
  const int searching_time_ms = 1 * 60 * 1000;
  while (millis() < searching_time_ms)
    delay(1000);
  bz.play(Buzzer::LOW_BATTERY);
  mr.forceBackToStart();
  vTaskDelay(portMAX_DELAY);
}

#define TEST_END_REMAIN 1
#define TEST_ST_LOOK_AHEAD(v) (6 + v / 100)
#define TEST_ST_FB_GAIN 10
#define TEST_ST_TR_FB_GAIN 0

void straight_x(const float distance, const float v_max, const float v_end) {
  const float a_max = 1000;
  const float v_start = std::max(sc.actual.trans, 0.0f);
  AccelDesigner ad(a_max, v_start, v_max, v_end, distance - TEST_END_REMAIN,
                   sc.position.x);
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (float t = 0.0f; true; t += 0.001f) {
    Position cur = sc.position;
    if (t > ad.t_end())
      break;
    float velocity = ad.v(t) + TEST_ST_TR_FB_GAIN * (ad.x(t) - cur.x);
    float theta = std::atan2(-cur.y, TEST_ST_LOOK_AHEAD(velocity)) - cur.theta;
    sc.set_target(velocity, TEST_ST_FB_GAIN * theta);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
  sc.set_target(ad.v_end(), 0);
}

void accel_test() {
  if (!ui.waitForCover())
    return;
  delay(500);
  lgr.clear();
  auto printLog = []() {
    lgr.push({
        0,
        sc.target_v.trans,
        sc.target_a.trans * 0.001f,
        sc.actual.trans,
        sc.enc_v.trans,
        sc.Kp * sc.proportional.trans,
        sc.Ki * sc.integral.trans,
        sc.Kd * sc.differential.trans,
        sc.Kp * sc.proportional.trans + sc.Ki * sc.integral.trans +
            sc.Kd * sc.differential.trans,
    });
  };
  imu.calibration();
  fan.drive(0.4);
  delay(500);
  sc.enable();
  const float accel = 9000;
  const float v_max = 1200;
  AccelDesigner ad(accel, 0, v_max, 0, 90 * 8);
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (float t = 0; t < ad.t_end() + 0.1f; t += 0.001f) {
    sc.set_target(ad.v(t), 0, ad.a(t), 0);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    xLastWakeTime = xTaskGetTickCount();
    if ((int)(t * 1000) % 2 == 0)
      printLog();
  }
  sc.set_target(0, 0);
  delay(200);
  bz.play(Buzzer::CANCEL);
  sc.disable();
  fan.drive(0);
}

void traj_test() {
  if (!ui.waitForCover())
    return;
  delay(500);
  lgr.clear();
  imu.calibration();
  sc.enable();
  const float accel = 9000;
  const float v_max = 1200;
  const float distance = 90 * 12;
  AccelDesigner ad(accel, 0, v_max, 0, distance);
  const float xi_threshold = 30.0f;
  float xi = xi_threshold;
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (float t = 0; t < ad.t_end() + 0.1f; t += 0.001f) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    if (ad.v(t) < xi_threshold) {
      sc.set_target(ad.v(t), 0, ad.a(t), 0);
      lgr.push({
          ad.v(t),
          ad.x(t),
          sc.actual.trans,
          sc.actual.rot,
          sc.position.x,
          sc.position.y,
          sc.position.theta,
      });
      continue;
    }
    const float x = sc.position.x;
    const float y = sc.position.y;
    const float theta = sc.position.theta;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    const float dx = sc.actual.trans * cos_theta;
    const float dy = sc.actual.trans * sin_theta;
    // const float kp1 = 1.0e1f;
    // const float kp2 = 1.0e1f;
    // const float kd1 = 1.0e-2f;
    // const float kd2 = 1.0e-2f;
    const float kp1 = 0.0e1f;
    const float kp2 = 0.0e1f;
    const float kd1 = 0.0e-2f;
    const float kd2 = 0.0e-2f;
    const float ddx_r = ad.a(t) * cos_theta;
    const float ddy_r = ad.a(t) * sin_theta;
    const float dx_r = ad.v(t) * cos_theta;
    const float dy_r = ad.v(t) * sin_theta;
    const float x_r = ad.x(t) * cos_theta;
    const float y_r = ad.x(t) * sin_theta;
    const float u1 = ddx_r + kp1 * (x_r - x) + kd1 * (dx_r - dx);
    const float u2 = ddy_r + kp2 * (y_r - y) + kd2 * (dy_r - dy);
    const float d_xi = u1 * cos_theta + u2 * sin_theta;
    const float v_r = xi;
    const float dv_r = d_xi;
    const float w_r = (u2 * cos_theta - u1 * sin_theta) / xi;
    const float dw_r = 0;
    sc.set_target(v_r, w_r, dv_r, dw_r);
    xi += d_xi * 0.001f;
    lgr.push({
        ad.v(t),
        ad.x(t),
        sc.actual.trans,
        sc.actual.rot,
        sc.position.x,
        sc.position.y,
        sc.position.theta,
        v_r,
        w_r,
    });
  }
  sc.set_target(0, 0);
  delay(200);
  bz.play(Buzzer::CANCEL);
  sc.disable();
}

void straight_test() {
  if (!ui.waitForCover())
    return;
  delay(1000);
  bz.play(Buzzer::SELECT);
  imu.calibration();
  sc.enable();
  fan.drive(0.2);
  delay(500);
  sc.position.x = 0;
  straight_x(8 * 90 - 3 - MACHINE_TAIL_LENGTH, 2400, 0);
  sc.set_target(0, 0);
  delay(100);
  fan.drive(0);
  delay(500);
  sc.disable();
}

void sysid_test() {
  int gain = ui.waitForSelect();
  if (gain < 0)
    return;
  if (!ui.waitForCover())
    return;
  delay(1000);
  lgr.clear();
  auto printLog = []() {
    lgr.push({
        enc.position(0),
        enc.position(1),
        imu.gyro.z,
        imu.accel.y,
        imu.angular_accel,
        ui.getBatteryVoltage(),
    });
  };
  bz.play(Buzzer::SELECT);
  imu.calibration();
  bz.play(Buzzer::CANCEL);
  fan.drive(0.5f);
  delay(500);
  portTickType xLastWakeTime = xTaskGetTickCount();
  mt.drive(-gain * 0.05, gain * 0.05);
  for (int i = 0; i < 2000; i++) {
    printLog();
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
  fan.drive(0);
  mt.drive(0, 0);
  delay(500);
  mt.free();
}

void driveTask(void *arg) {
  while (1) {
    delay(1);
    int mode = ui.waitForSelect(16);
    switch (mode) {
    /* 走行 */
    case 0:
      Machine::driveNormally();
      break;
    /* プリセット走行パラメータの選択 */
    case 1:
      Machine::selectParamPreset();
      break;
    /* マニュアル走行パラメータの設定 */
    case 2:
      Machine::selectParamManually();
      break;
    /* 壁制御の設定 */
    case 3: {
      int value = ui.waitForSelect(16);
      if (value < 0)
        return;
      fr.wallAvoidFlag = value & 0x01;
      fr.wallAvoid45Flag = value & 0x02;
      fr.wallCutFlag = value & 0x04;
      fr.V90Enabled = value & 0x08;
    }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    /* ファンの設定 */
    case 4:
      Machine::selectFanGain();
      break;
    /* 迷路データの復元 */
    case 5:
      bz.play(Buzzer::MAZE_RESTORE);
      if (!mr.restore()) {
        bz.play(Buzzer::ERROR);
        return;
      }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    /* データ消去 */
    case 6:
      bz.play(Buzzer::MAZE_BACKUP);
      mr.resetLastWall(10);
      return;
      break;
    /* 宴会芸 */
    case 7:
      Machine::partyStunt();
      break;
    /* 壁センサキャリブレーション */
    case 8:
      Machine::wallCalibration();
      break;
    /* ゴール区画の設定 */
    case 11:
      Machine::setGoalPositions();
      break;
    /* マス直線 */
    case 12:
      if (!ui.waitForCover(true))
        return;
      bz.play(Buzzer::CONFIRM);
      imu.calibration();
      bz.play(Buzzer::CANCEL);
      sc.enable();
      straight_x(9 * 90 - 6 - MACHINE_TAIL_LENGTH, 300, 0);
      sc.disable();
      break;
    /* テスト */
    case 13:
      // traj_test();
      // accel_test();
      sysid_test();
      break;
    /* ログの表示 */
    case 14:
      lgr.print();
      break;
    /* リセット */
    case 15:
      mr.print();
      ESP.restart();
      break;
    }
  }
}
