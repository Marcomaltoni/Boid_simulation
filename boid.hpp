#ifndef BOID_HPP
#define BOID_HPP

#include "SFML/Graphics.hpp"
#include "vector2.hpp"

namespace pr {
class Boid {
  Vector2 x_;

  Vector2 v_;

  float vmax_;

  sf::CircleShape boidshape_;

 public:
  Boid();

  Boid(Vector2, Vector2, float);

  Vector2 position() const;

  Vector2 velocity() const;

  float maximum_velocity() const;

  Vector2 separation(const Boid& b_j, float s, float ds) const;

  Vector2 allignment(const Boid& b_j, float a, float n, float d) const;

  Vector2 cohesion(const Vector2& x_cm, float c) const;

  float get_angle() const;

  void limit_velocity();

  void change_velocity(const Vector2& vec);

  void change_position(const Vector2& vec);

  bool operator==(const Boid& b) const;

  sf::CircleShape& shape();

  const sf::CircleShape& c_shape()
      const;  // function with the only purpose of reading "boidshape",
              // necessary to call as a function argument a "const boid" object,
              // when on its shape are used methods which don't modify it, as
              // "getFillColor()".

  void setRadius(float r);

  void setPointCount();

  void setOrigin(float x, float y);

  void setScale(float x, float y);

  void setFillColor(const sf::Color& color);

  void setPosition(const Vector2& vec);

  void setRotation();
};
}  // namespace pr

#endif