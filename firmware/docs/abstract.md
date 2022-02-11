# 概要

迷路を探索して最短経路を導出して，その最短経路を高速に走行するロボット．

## 要求

- 入出力
  - センサーからデータを読み取る
  - アクチュエーターを駆動する
- 制御
  - 軌道計画を行う
  - 自己位置を推定する
  - 制御信号を算出する
- 探索
  - 迷路情報を保持する
  - 自己位置を管理する
  - 進むべき方向を決定する
- 自律
  - 探索状態を管理する
  - 走行パラメータを選択する
  - 競技時間を管理する
- 操作
  - ユーザーインターフェイスを提供する

## ハードウェア仕様

- センサー
  - IMU
  - 車輪エンコーダー
  - フォトリフレクター
  - ボタン
- アクチュエーター
  - 車輪モーター
  - 吸引ファン
  - LEDインジケーター
  - スピーカー

## スレッド

| name         | priority | core |                    stack |
| ------------ | -------: | ---: | -----------------------: |
| esp_timer    |       22 |  PRO |                 3584+512 |
| IMU          |        6 |  PRO |                     4096 |
| Encoder      |        6 |  PRO | configMINIMAL_STACK_SIZE |
| SpeedCtrl    |        5 |  PRO |                     4096 |
| WallDetector |        5 |  PRO |                     2048 |
| ToF          |        1 |  PRO |                     4096 |
| main         |        0 |  PRO |                     8192 |
| Reflector    |       20 |  APP |                     2048 |
| MoveAction   |        4 |   NO |                     8192 |
| Button       |        1 |   NO | configMINIMAL_STACK_SIZE |
| LED          |        1 |   NO |                     2048 |
| Buzzer       |        1 |   NO |                     4096 |
| drive        |        2 |   NO |                     4096 |
| print        |        1 |   NO |                     4096 |

- PRO_CPU_NUM: 0
- APP_CPU_NUM: 1
- tskNO_AFFINITY
