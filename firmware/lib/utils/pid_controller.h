#pragma once

class PIDController {
public:
  struct Parameter {
    float Kp, Ki, Kd;
    Parameter(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd) {}
    Parameter() {}
  };

public:
  PIDController(struct Parameter gain) : gain(gain) {}
  PIDController() {}
  float p, i, d;

  void setGain(const struct Parameter gain) { this->gain = gain; }
  const struct Parameter getGain() const { return gain; }
  float calc(float r, float y, float ie, float dr, float dy) {
    p = gain.Kp * (r - y);
    i = gain.Ki * ie;
    d = gain.Kd * (dr - dy);
    return p + i + d;
  }

private:
  struct Parameter gain;
};

class Disc2DOFPIDController {
public:
  struct Parameter {
    float Kp, Ki, Kd, Tf, b, c, Ts;
    Parameter(float Kp, float Ki, float Kd, float Tf, float b, float c,
              float Ts)
        : Kp(Kp), Ki(Ki), Kd(Kd), Tf(Tf), b(b), c(c), Ts(Ts) {}
    Parameter() {}
  };

public:
  Disc2DOFPIDController(struct Parameter gain) : gain(gain) {}
  Disc2DOFPIDController() {}
  float p, i, d;

  void setGain(const struct Parameter gain) { this->gain = gain; }
  const struct Parameter getGain() const { return gain; }
  float calc(float r, float y, float ie, float dr, float dy) {
    p = gain.Kp * (gain.b * r - y);
    i = gain.Ki * ie;
    d = gain.Kd * (gain.c * dr - dy);
    return p + i + d;
  }

private:
  struct Parameter gain;
};
