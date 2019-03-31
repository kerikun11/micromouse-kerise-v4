#include "Machine.h"
#include <iostream>

#include "TrajectoryTracker.h"

void driveTask(void *arg);
void printTask(void *arg);
void timeKeepTask(void *arg);

void setup() {
  WiFi.mode(WIFI_OFF);
  Serial.begin(2000000);
  std::cout << std::endl;
  std::cout << "**************** KERISE v4 ****************" << std::endl;
  Machine::init();
  xTaskCreate(printTask, "print", 4096, NULL, 2, NULL);
  // xTaskCreate(timeKeepTask, "TimeKeep", 4096, NULL, 2, NULL);
  xTaskCreate(driveTask, "drive", 4096, NULL, 2, NULL);
}

void loop() { delay(1000); }

void printTask(void *arg) {
  portTickType xLastWakeTime = xTaskGetTickCount();
  // Accumulator<float, 64> tmp;
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    // enc.print();
    // ref.csv();
    // tof.csv(); delay(100);
    // imu.print();
    // wd.print();
    // wd.csv();
    // for (int i = 0; i < 100; i++) {
    //   vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    //   tmp.push(enc.position(0));
    // }
    // std::cout << (tmp[0] - tmp[tmp.size() - 1]) / tmp.size() / 0.001f
    //           << std::endl;
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
  const float v_start = std::max(sc.est_v.tra, 0.0f);
  signal_processing::AccelDesigner ad(
      a_max, v_start, v_max, v_end, distance - TEST_END_REMAIN, sc.position.x);
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

void turn_test() {
  if (!ui.waitForCover())
    return;
  delay(500);
  const float jerk = 540 * PI;
  const float angle = -1 * PI;
  const float sign = (angle > 0) ? 1 : -1;
  const float omega = 4 * PI;
  const float d_omega = 48 * PI;
  imu.calibration();
  sc.enable();
  signal_processing::AccelDesigner ad(jerk, d_omega, 0, omega, 0, sign * angle);
  portTickType xLastWakeTime = xTaskGetTickCount();
  const float back_gain = 10.0f;
  for (float t = 0; t < ad.t_end(); t += 0.001f) {
    float delta = sc.position.x * std::cos(-sc.position.theta) -
                  sc.position.y * std::sin(-sc.position.theta);
    sc.set_target(-delta * back_gain, sign * ad.v(t), 0, sign * ad.a(t));
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
  }
  float int_error = 0;
  while (1) {
    float delta = sc.position.x * std::cos(-sc.position.theta) -
                  sc.position.y * std::sin(-sc.position.theta);
    const float Kp = 20.0f;
    const float Ki = 10.0f;
    const float error = angle - sc.position.theta;
    int_error += error * 0.001f;
    sc.set_target(-delta * back_gain, Kp * error + Ki * int_error);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    if (std::abs(Kp * error) + std::abs(Ki * int_error) < 0.01f * PI)
      break;
  }
  sc.set_target(0, 0);
  delay(200);
  bz.play(Buzzer::CANCEL);
  sc.disable();
}

void traj_test() {
  if (!ui.waitForCover())
    return;
  delay(500);
  lgr.clear();
  imu.calibration();
  sc.enable();
  const float jerk = 500000;
  const float accel = 4800;
  const float v_max = 1800;
  const float distance = 90 * 16;
  const float v_start = 0;
  signal_processing::AccelDesigner ad(jerk, accel, v_start, v_max, 0, distance);
  trajectory::TrajectoryTracker tt(v_start);
  portTickType xLastWakeTime = xTaskGetTickCount();
  for (float t = 0; t < ad.t_end() + 0.1f; t += 0.001f) {
    auto ref_q = Position(ad.x(t), 0);
    auto ref_dq = Position(ad.v(t), 0);
    auto ref_ddq = Position(ad.a(t), 0);
    auto ref_dddq = Position(ad.j(t), 0);
    auto ref = tt.update(sc.est_v, sc.est_a, sc.position, ref_q, ref_dq,
                         ref_ddq, ref_dddq);
    sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
    lgr.push({
        sc.ref_v.tra,
        sc.est_v.tra,
        sc.ref_a.tra,
        sc.est_a.tra,
        sc.ref_v.rot,
        sc.est_v.rot,
        sc.ref_a.rot,
        sc.est_a.rot,
        sc.position.x,
        sc.position.y,
        sc.position.theta,
        ad.a(t),
        ad.v(t),
        ad.x(t),
    });
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
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

#include "SlalomDesigner.h"

void slalom_test() {
  if (!ui.waitForCover())
    return;
  delay(500);
  lgr.clear();
  auto printLog = [](const Position ref_q, const Position est_q) {
    lgr.push({
        sc.ref_v.tra,
        sc.est_v.tra,
        sc.ref_a.tra,
        sc.est_a.tra,
        sc.ref_v.rot,
        sc.est_v.rot,
        sc.ref_a.rot,
        sc.est_a.rot,
        ref_q.x,
        est_q.x,
        ref_q.y,
        est_q.y,
        ref_q.theta,
        est_q.theta,
    });
  };
  auto sd = signal_processing::SlalomDesigner(
      signal_processing::SlalomDesigner::Constraint(-M_PI / 2, -40, 45, -45));
  // auto sd = signal_processing::SlalomDesigner(
  //     signal_processing::SlalomDesigner::Constraint(M_PI / 2, 70, 90, 90));
  signal_processing::AccelDesigner ad;
  bz.play(Buzzer::CONFIRM);
  imu.calibration();
  bz.play(Buzzer::CANCEL);
  sc.enable();
  const float jerk = 500000;
  const float accel = 4800;
  const float vel = 360;
  const float v_start = 0;
  trajectory::TrajectoryTracker tt(v_start);
  portTickType xLastWakeTime = xTaskGetTickCount();
  Position offset;
  /* 加速 */
  const float st_len = 60;
  ad.reset(jerk, accel, v_start, vel, vel, st_len);
  for (float t = 0; t < ad.t_end(); t += 0.001f) {
    auto est_q = (sc.position - offset).rotate(-offset.theta);
    auto ref_q = Position(ad.x(t), 0);
    auto ref_dq = Position(ad.v(t), 0);
    auto ref_ddq = Position(ad.a(t), 0);
    auto ref_dddq = Position(ad.j(t), 0);
    auto ref =
        tt.update(sc.est_v, sc.est_a, est_q, ref_q, ref_dq, ref_ddq, ref_dddq);
    sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    printLog(offset + ref_q.rotate(offset.theta),
             offset + est_q.rotate(offset.theta));
  }
  {
    auto net = Position(st_len, 0, 0);
    offset += net.rotate(offset.theta);
  }
  /* ターン */
  signal_processing::SlalomDesigner::State s;
  const float Ts = 0.001f;
  sd.reset(vel);
  for (float t = 0; t < sd.t_end(); t += 0.001f) {
    sd.update(&s, Ts);
    auto est_q = (sc.position - offset).rotate(-offset.theta);
    auto ref_q = Position(s.x, s.y, s.th);
    auto ref_dq = Position(s.dx, s.dy);
    auto ref_ddq = Position(s.ddx, s.ddy);
    auto ref_dddq = Position(s.dddx, s.dddy);
    auto ref =
        tt.update(sc.est_v, sc.est_a, est_q, ref_q, ref_dq, ref_ddq, ref_dddq);
    sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    printLog(offset + ref_q.rotate(offset.theta),
             offset + est_q.rotate(offset.theta));
  }
  {
    auto net = Position(sd.x_end(), sd.y_end(), sd.th_end());
    offset += net.rotate(offset.theta);
  }
  /* 減速 */
  ad.reset(jerk, accel, vel, vel, 0, st_len);
  for (float t = 0; t < ad.t_end(); t += 0.001f) {
    auto est_q = (sc.position - offset).rotate(-offset.theta);
    auto ref_q = Position(ad.x(t), 0);
    auto ref_dq = Position(ad.v(t), 0);
    auto ref_ddq = Position(ad.a(t), 0);
    auto ref_dddq = Position(ad.j(t), 0);
    auto ref =
        tt.update(sc.est_v, sc.est_a, est_q, ref_q, ref_dq, ref_ddq, ref_dddq);
    sc.set_target(ref.v, ref.w, ref.dv, ref.dw);
    vTaskDelayUntil(&xLastWakeTime, 1 / portTICK_RATE_MS);
    printLog(offset + ref_q.rotate(offset.theta),
             offset + est_q.rotate(offset.theta));
  }
  {
    auto net = Position(st_len, 0, 0);
    offset += net.rotate(offset.theta);
  }
  sc.set_target(0, 0);
  delay(200);
  bz.play(Buzzer::CANCEL);
  sc.disable();
  fan.drive(0);
}

void driveTask(void *arg) {
  while (1) {
    delay(1);
    int mode = ui.waitForSelect(16);
    switch (mode) {
    case 0: /* 迷路走行 */
      Machine::driveNormally();
      break;
    case 1: /* プリセット走行パラメータの選択 */
      Machine::selectParamPreset();
      break;
    case 2: /* マニュアル走行パラメータの設定 */
      Machine::selectParamManually();
      break;
    case 3: { /* 壁制御，斜め走行の設定 */
      int value = ui.waitForSelect(16);
      if (value < 0)
        break;
      fr.wallAvoidFlag = value & 0x01;
      fr.wallAvoid45Flag = value & 0x02;
      fr.wallCutFlag = value & 0x04;
      fr.V90Enabled = value & 0x08;
    }
      bz.play(Buzzer::SUCCESSFUL);
      break;
    case 4: /* ファンの設定 */
      Machine::selectFanGain();
      break;
    case 5: /* 迷路データの復元 */
      bz.play(Buzzer::MAZE_RESTORE);
      if (!mr.restore())
        bz.play(Buzzer::ERROR);
      else
        bz.play(Buzzer::SUCCESSFUL);
      break;
    case 6: /* データ消去 */
      bz.play(Buzzer::MAZE_BACKUP);
      mr.resetLastWall(10);
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
    case 11: /* ゴール区画の設定 */
      Machine::setGoalPositions();
      break;
    case 12: /* マス直線 */
      if (!ui.waitForCover())
        return;
      bz.play(Buzzer::CONFIRM);
      imu.calibration();
      bz.play(Buzzer::CANCEL);
      sc.enable();
      straight_x(9 * 90 - 6 - MACHINE_TAIL_LENGTH, 300, 0);
      sc.disable();
      break;
    case 13: /* 迷路の表示 */
      mr.print();
      // ESP.restart();
      break;
    case 14: /* テスト */
      // Machine::accel_test();
      // Machine::sysid();
      slalom_test();
      // turn_test();
      // traj_test();
      break;
    case 15: /* ログの表示 */
      lgr.print();
      break;
    }
  }
}
