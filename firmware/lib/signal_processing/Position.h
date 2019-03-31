#pragma once

#include <cmath>
#include <cstdio>

namespace coordinate {

class Position {
public:
  float x, y, theta;

public:
  Position(const float x = 0, const float y = 0, const float theta = 0)
      : x(x), y(y), theta(theta) {}
  Position(const Position &obj) : x(obj.x), y(obj.y), theta(obj.theta) {}
  Position(const float pos[3]) : x(pos[0]), y(pos[1]), theta(pos[2]) {}
  void clear() { x = y = theta = 0; }
  void set(const float x = 0, const float y = 0, const float theta = 0) {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }
  const Position rotate(const float angle) const {
    const float cos_angle = std::cos(angle);
    const float sin_angle = std::sin(angle);
    return Position(x * cos_angle - y * sin_angle,
                    x * sin_angle + y * cos_angle, theta);
  }
  float getNorm() const { return std::sqrt(x * x + y * y); }
  const Position mirror_x() const { return Position(x, -y, -theta); }
  const Position &operator=(const Position &obj) {
    x = obj.x;
    y = obj.y;
    theta = obj.theta;
    return *this;
  }
  const Position operator+() const { return Position(x, y, theta); }
  const Position operator+(const Position &obj) const {
    return Position(x + obj.x, y + obj.y, theta + obj.theta);
  }
  const Position &operator+=(const Position &obj) {
    x += obj.x;
    y += obj.y;
    theta += obj.theta;
    return *this;
  }
  const Position operator-() const { return Position(-x, -y, -theta); }
  const Position operator-(const Position &obj) const {
    return Position(x - obj.x, y - obj.y, theta - obj.theta);
  }
  const Position &operator-=(const Position &obj) {
    x -= obj.x;
    y -= obj.y;
    theta -= obj.theta;
    return *this;
  }
  void print(const char *name = "Pos") {
    std::printf("%s: (%.1f,\t%.1f,\t%.3f)\n", name, x, y, theta);
  }
};

struct WheelParameter {
public:
  float tra;      //< tralation [mm]
  float rot;      //< rotation [rad]
  float wheel[2]; //< wheel position [mm], wheel[0]:left, wheel[1]:right
public:
  WheelParameter() { clear(); }
  void pole2wheel() {
    wheel[0] = tra - MACHINE_ROTATION_RADIUS * rot;
    wheel[1] = tra + MACHINE_ROTATION_RADIUS * rot;
  }
  void wheel2pole() {
    rot = (wheel[1] - wheel[0]) / 2 / MACHINE_ROTATION_RADIUS;
    tra = (wheel[1] + wheel[0]) / 2;
  }
  void clear() {
    tra = 0;
    rot = 0;
    wheel[0] = 0;
    wheel[1] = 0;
  }
};

class Polar {
public:
  float tra; //< tralation [mm]
  float rot; //< rotation [rad]
public:
  Polar(const float tra = 0, const float rot = 0) : tra(tra), rot(rot) {}
  Polar(const Polar &o) : tra(o.tra), rot(o.rot) {}
  const Polar &operator=(const Polar &o) {
    tra = o.tra;
    rot = o.rot;
    return *this;
  }
  const Polar &operator+=(const Polar &o) {
    tra += o.tra;
    rot += o.rot;
    return *this;
  }
  const Polar &operator*=(const Polar &o) {
    tra *= o.tra;
    rot *= o.rot;
    return *this;
  }
  const Polar &operator/=(const Polar &o) {
    tra /= o.tra;
    rot /= o.rot;
    return *this;
  }
  const Polar operator+(const Polar &o) const {
    return Polar(tra + o.tra, rot + o.rot);
  }
  const Polar operator-(const Polar &o) const {
    return Polar(tra - o.tra, rot - o.rot);
  }
  const Polar operator*(const Polar &o) const {
    return Polar(tra * o.tra, rot * o.rot);
  }
  const Polar operator/(const Polar &o) const {
    return Polar(tra / o.tra, rot / o.rot);
  }
  const Polar operator+(const float &k) const {
    return Polar(tra + k, rot + k);
  }
  const Polar operator-(const float &k) const {
    return Polar(tra - k, rot - k);
  }
  const Polar operator*(const float &k) const {
    return Polar(tra * k, rot * k);
  }
  const Polar operator/(const float &k) const {
    return Polar(tra / k, rot / k);
  }
  void clear() { tra = rot = 0; }
};

} // namespace coordinate
