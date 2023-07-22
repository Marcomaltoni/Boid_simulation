#include "vector2.hpp"

#include <cmath>
#include <cassert>

namespace pr {
Vector2::Vector2() : x_{0.}, y_{0.} {};

Vector2::Vector2(float x, float y) : x_{x}, y_{y} {};

float Vector2::x_axis() const { return x_; }

float Vector2::y_axis() const { return y_; }

Vector2 Vector2::operator+=(const Vector2 &other_vector) {
  x_ += other_vector.x_;
  y_ += other_vector.y_;

  return *this;
}

Vector2 Vector2::operator+(const Vector2 &other_vector) const {
  Vector2 alias = *this;

  return alias += other_vector;
}

Vector2 Vector2::operator-(const Vector2 &other_vector) const {
  Vector2 alias = *this;
  const Vector2 neg_vec{-other_vector.x_, -other_vector.y_};

  return alias += (neg_vec);
}

float Vector2::distance(const Vector2 &other_vector) const {
  const Vector2 alias = *this;
  const Vector2 difference = alias - other_vector;
  const float distance = std::sqrt(std::pow(difference.x_, 2) + std::pow(difference.y_, 2));
  assert(distance >= 0.f);

  return distance;
}

float Vector2::lenght_of_vector() const { 
  const float lenght = std::sqrt(x_ * x_ + y_ * y_);
  assert(lenght >= 0.f);

  return lenght;  }

bool Vector2::operator!=(const Vector2 &other_vector) const {
  return (x_ != other_vector.x_ || y_ != other_vector.y_);
}

bool Vector2::operator==(const Vector2 &other_vector) const {
  return !(*this != other_vector);
}

Vector2 Vector2::operator*(float scalar) const {
  const Vector2 result{scalar * x_, scalar * y_};

  return result;
}
}  // namespace pr
