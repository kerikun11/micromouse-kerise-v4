# KERISE v4 のシステム同定 2019.02.12

## 並進の伝達関数

- データ: 20190211 sysid/0.2-2.tab
- 1-1150 step, v_filtered/1000
- 入力振幅 0.2
- 実極2つの２次モデル
- 0.001秒で離散化 ModelTrans = c2d(tf(IdentifyedModelDutyToTranslationalVelocity), 0.001);

```matlab
IdentifyedModelDutyToTranslationalVelocity =
Process model with transfer function:
                Kp
  G(s) = -----------------
         (1+Tp1*s)(1+Tp2*s)

         Kp = 6.553
        Tp1 = 0.20501
        Tp2 = 0.37268

Parameterization:
    'P2'
   Number of free coefficients: 3
   Use "getpvec", "getcov" for parameters and their uncertainties.

Status:
Estimated using PROCEST on time domain data.
Fit to estimation data: 94.07%
FPE: 0.0003379, MSE: 0.000335
```

## 回転の伝達関数

- データ: 20190211 sysid/r0.3-2.tab
- 0-500 step, omega_enc
- 入力振幅 0.6
- 0.001 [s] で離散化 ModelRot = c2d(tf(IdentifyedModelDutyToRotationalVelocity), 0.001);

```matlab
IdentifyedModelDutyToRotationalVelocity =
Process model with transfer function:
                Kp
  G(s) = -----------------
         (1+Tp1*s)(1+Tp2*s)

         Kp = 32.956
        Tp1 = 1.0567e-07
        Tp2 = 0.08153

Parameterization:
    'P2'
   Number of free coefficients: 3
   Use "getpvec", "getcov" for parameters and their uncertainties.

Status:
Estimated using PROCEST on time domain data.
Fit to estimation data: 51.68%
FPE: 16.98, MSE: 16.78
```
