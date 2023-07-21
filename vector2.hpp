#ifndef VECTOR2_HPP
#define VECTOR2_HPP

namespace pr {
class Vector2 {
  float x_;

  float y_;

 public:
  Vector2();

  Vector2(float x, float y);

  float x_axis() const;

  float y_axis() const;

  Vector2 operator+=(const Vector2 &other_vector);

  Vector2 operator+(const Vector2 &other_vector) const;

  Vector2 operator-(const Vector2 &other_vector) const;

  float distance(const Vector2 &other_vector) const;

  float lenght_of_vector() const;

  bool operator!=(const Vector2 &other_vector) const;

  bool operator==(const Vector2 &other_vector) const;

  Vector2 operator*(float scalar) const;
};
}  // namespace pr

#endif