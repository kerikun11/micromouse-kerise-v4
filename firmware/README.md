# KERISE firmware

## Class Design

### Driver

- Buzzer
  - Event-driven Task
- LED
  - Event-driven Task
- Motor
- Fan

### Sensor

- Button
  - Periodic Update
- IMU
  - Periodic Update
- Encoder
  - Periodic Update
- Reflector
  - Periodic Update
- ToF
  - Periodic Update

### Supporter

- UserInterface
  - Blocking Process
  - Button
  - IMU
  - LED
  - Buzzer
- SpeedController
  - Periodic Update
  - Odometry
  - Motor
  - IMU
  - Encoder
- WallDetector
  - Periodic Update
  - Reflector
  - ToF

### Conductor

- SearchRun
  - Task
- FastRun
  - Task
- MazeRobot
  - Task
- Logger
- ExternalController
- Machine

## Controller

1. Sampling
2. Calculation
    1. Integration
    2. Control
3. Conduction

## Main Task

- DriveTask
- PrintTask
- TimeKeepTask

## Settings

### Maze

- Restore
- Backup
- Erase Partially

### Algorithm

- The number of path

### SearchRun

- Run Speed
- Max Speed
- KnwonDiag
- WallCut

### FastRun

- Diag or Along
- CurveGain
- WallCut
