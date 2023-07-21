#include "vector2.hpp"

#include <cmath>

namespace pr {
Vector2::Vector2() : x_{0.}, y_{0.} {};

Vector2::Vector2(float x, float y) : x_{x}, y_{y} {};

float Vector2::x_axis() const { return x_; }

float Vector2::y_axis() const { return y_; }

Vector2 Vector2::operator+=(const Vector2 &vec) {
  x_ += vec.x_;
  y_ += vec.y_;

  return *this;
}

Vector2 Vector2::operator+(const Vector2 &vec) const {
  Vector2 alias = *this;

  return alias += vec;
}

Vector2 Vector2::operator-(const Vector2 &vec) const {
  Vector2 alias = *this;
  Vector2 neg_vec{-vec.x_, -vec.y_};

  return alias += (neg_vec);
}

float Vector2::dist(const Vector2 &vec) const {
  Vector2 alias = *this;
  Vector2 difference = alias - vec;

  return std::sqrt(std::pow(difference.x_, 2) + std::pow(difference.y_, 2));
}

float Vector2::mod() const { return std::sqrt(x_ * x_ + y_ * y_); }

bool Vector2::operator!=(const Vector2 &vec) const {
  return (x_ != vec.x_ || y_ != vec.y_);
}

bool Vector2::operator==(const Vector2 &vec) const { return !(*this != vec); }

Vector2 Vector2::operator*(float s) const {
  Vector2 v1{s * x_, s * y_};

  return v1;
}
}  // namespace pr
