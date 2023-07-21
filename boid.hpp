#ifndef BOID_HPP
#define BOID_HPP

#include "SFML/Graphics.hpp"
#include "vector2.hpp"

namespace pr {
class Boid {
  Vector2 position_;

  Vector2 velocity_;

  float velocity_max_;

  sf::CircleShape boidshape_;

 public:
  Boid();

  Boid(Vector2 position, Vector2 velocity, float maximum_velocity);

  Vector2 position() const;

  Vector2 velocity() const;

  float maximum_velocity() const;

  Vector2 separation(const Boid& other_boid, float separation_parameter,
                     float distance_of_separation) const;

  Vector2 allignment(const Boid& other_boid, float allignment_parameter,
                     float close_boids, float closeness_parameter) const;

  Vector2 cohesion(const Vector2& centre_of_mass,
                   float cohesion_parameter) const;

  float get_angle() const;

  void limit_velocity();

  void change_velocity(const Vector2& velocity_offset);

  void change_position(const Vector2& position_offset);

  bool operator==(const Boid& other_boid) const;

  sf::CircleShape& set_shape();

  const sf::CircleShape& get_shape()
      const;  // function with the only purpose of reading "boidshape",
              // necessary to call as a function argument a "const boid" object,
              // when on its shape are used methods which don't modify it, as
              // "getFillColor()".

  void setRadius(float radius);

  void setPointCount();

  void setOrigin(float origin_x, float origin_y);

  void setScale(float scale_x, float scale_y);

  void setFillColor(const sf::Color& color);

  void setPosition(const Vector2& new_position);

  void setRotation();
};
}  // namespace pr

#endif