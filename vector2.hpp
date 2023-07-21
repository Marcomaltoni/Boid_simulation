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

  Vector2 operator+=(const Vector2 &vec);

  Vector2 operator+(const Vector2 &vec) const;

  Vector2 operator-(const Vector2 &vec) const;

  float dist(const Vector2 &vec) const;

  float mod() const;

  bool operator!=(const Vector2 &vec) const;

  bool operator==(const Vector2 &vec) const;

  Vector2 operator*(float s) const;
};
}  // namespace pr

#endif