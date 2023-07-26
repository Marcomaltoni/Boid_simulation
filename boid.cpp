#define _USE_MATH_DEFINES
#include "boid.hpp"

#include <cassert>
#include <cmath>

namespace pr {
Boid::Boid() : position_{Vector2{}}, velocity_{Vector2{}}, velocity_max_{0.f} {}

Boid::Boid(Vector2 position, Vector2 velocity, float maximum_velocity)
    : position_{position},
      velocity_{velocity},
      velocity_max_{maximum_velocity} {
        assert(velocity_max_ > 0);
      }

Vector2 Boid::position() const { return position_; }

Vector2 Boid::velocity() const { return velocity_; }

float Boid::maximum_velocity() const { return velocity_max_; }

bool Boid::isRed() const {
  return boidshape_.getFillColor() == sf::Color::Red;
}

Vector2 Boid::separation(const Boid& other_boid, float separation_parameter,
                         float distance_of_separation) const {
  if (position_.distance(other_boid.position()) < distance_of_separation &&
      position_.distance(other_boid.position()) != 0.f &&
      isRed() == true) {
    const Vector2 separation_velocity =
        (other_boid.position() - position_) * (-separation_parameter);

    return separation_velocity;

  } else {
    const Vector2 null{};

    return null;
  }
}

Vector2 Boid::allignment(const Boid& other_boid, float allignment_parameter,
                         float close_boids, float closeness_parameter) const {
  if (close_boids >= 1.f &&
      position_.distance(other_boid.position()) < closeness_parameter &&
      position_.distance(other_boid.position()) != 0.f &&
      boidshape_.getFillColor() == other_boid.get_shape().getFillColor() &&
      isRed() == true) {
    const Vector2 allignment_velocity = (other_boid.velocity() - velocity_) *
                                        (allignment_parameter / close_boids);

    return allignment_velocity;

  } else {
    const Vector2 null{};

    return null;
  }
}

Vector2 Boid::cohesion(const Vector2& center_of_mass,
                       float cohesion_parameter) const {
  if (center_of_mass.x_axis() != 0 && center_of_mass.y_axis() != 0) {
    const Vector2 cohesion_velocity =
        (center_of_mass - position_) * cohesion_parameter;

    return cohesion_velocity;

  } else {
    const Vector2 null{};

    return null;
  }
}

float Boid::get_angle() const {
  if (velocity_.y_axis() != 0.f) {
    if (velocity_.x_axis() >= 0.f && velocity_.y_axis() > 0.f) {
      const float angle =
          (180. / M_PI) *
              std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis())) +
          2 * (90.f -
               ((180. / M_PI) *
                std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis()))));
      assert(angle >= 90.f && angle <= 180.f);

      return angle;
    }

    if (velocity_.x_axis() < 0.f && velocity_.y_axis() > 0.f) {
      const float angle =
          ((180. / M_PI) *
           std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis()))) +
          2 * (90.f -
               ((180. / M_PI) *
                std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis())))) +
          2 * ((180. / M_PI) *
               std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis())));
      assert(angle >= 180.f && angle <= 270.f);

      return angle;
    }

    if (velocity_.x_axis() <= 0.f && velocity_.y_axis() < 0.f) {
      const float angle =
          ((180. / M_PI) *
           std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis()))) +
          4 * (90.f -
               ((180. / M_PI) *
                std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis())))) +
          2 * ((180. / M_PI) *
               std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis())));
      assert(angle >= 270.f && angle <= 360.f);

      return angle;
    }

    if (velocity_.x_axis() > 0.f && velocity_.y_axis() < 0.f) {
      const float angle =
          ((180. / M_PI) *
           std::atan(std::abs(velocity_.x_axis() / velocity_.y_axis())));
      assert(angle >= 0.f && angle <= 90.f);

      return angle;
    }

  } else {
    if (velocity_.x_axis() < 0.f && velocity_.y_axis() == 0.f) {
      return 270.f;
    }

    if (velocity_.x_axis() > 0.f && velocity_.y_axis() == 0.f) {
      return 90.f;
    }

    if (velocity_.x_axis() == 0.f && velocity_.y_axis() == 0.f) {
      return 0.f;
    }
  }

  return 180.f;
}

void Boid::limit_velocity() {
  if (velocity_.lenght_of_vector() >= velocity_max_) {
    velocity_ = velocity_ * 0.5f;
  }
}

void Boid::change_velocity(const Vector2& velocity_offset) {
  velocity_ += velocity_offset;
}

void Boid::change_position(const Vector2& position_offset) {
  position_ += position_offset;
}

bool Boid::operator==(const Boid& other_boid) const {
  return (position_ == other_boid.position_ &&
          velocity_ == other_boid.velocity_ &&
          velocity_max_ == other_boid.velocity_max_);
};

sf::CircleShape& Boid::set_shape() { return boidshape_; }

const sf::CircleShape& Boid::get_shape() const { return boidshape_; }

void Boid::setRadius(float radius) { boidshape_.setRadius(radius); }

void Boid::setPointCount() { boidshape_.setPointCount(3); }

void Boid::setOrigin(float origin_x, float origin_y) {
  boidshape_.setOrigin(origin_x, origin_y);
}

void Boid::setScale(float scale_x, float scale_y) {
  boidshape_.setScale(scale_x, scale_y);
}

void Boid::setFillColor(const sf::Color& color) {
  boidshape_.setFillColor(color);
}

void Boid::setPosition(const Vector2& new_position) {
  const sf::Vector2f graphic_position{new_position.x_axis(),
                                      new_position.y_axis()};
  boidshape_.setPosition(graphic_position);
};

void Boid::setRotation() { boidshape_.setRotation(get_angle()); }

}  // namespace pr