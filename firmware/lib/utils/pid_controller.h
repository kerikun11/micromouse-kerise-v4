#pragma once

template <typename T_var, typename T_gain> class PIDController {
public:
  template <typename T> struct PID_Vector {
    T p, i, d;
    PID_Vector(T_gain p = T_gain(), T_gain i = T_gain(), T_gain d = T_gain())
        : p(p), i(i), d(d) {}
    PID_Vector(const PID_Vector &obj) : p(obj.p), i(obj.i), d(obj.d) {}
    const PID_Vector &operator=(const PID_Vector &obj) {
      p = obj.p;
      i = obj.i;
      d = obj.d;
      return *this;
    }
  };
  typedef PID_Vector<T_var> Vars;
  typedef PID_Vector<T_gain> Gains;

public:
  PIDController(Gains gains = Gains()) {}
  const Gains getGains() const { return gains; }
  void setGain(Gains newGains) { gains = newGains; }
  const Vars getError() const { return e; }
  void resetError() { e.p = e.i = e.d = T_var(); }
  const T_var calculate(const float Ts, const T_var r, const T_var y,
                        const T_var r_dot, const T_var y_dot) {
    e.p = r - y;
    e.i += (r - y) * Ts;
    e.d = r_dot - y_dot;
    return e.p * gains.p + e.i * gains.i + e.d * gains.d;
  }

private:
  Gains gains;
  Vars e;
};
