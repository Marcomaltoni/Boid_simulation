#define _USE_MATH_DEFINES
#include "boid.hpp"

#include <cmath>

namespace pr {
Boid::Boid() : x_{Vector2{}}, v_{Vector2{}}, vmax_{0.f} {}

Boid::Boid(Vector2 position, Vector2 velocity, float maximum_velocity)
    : x_{position}, v_{velocity}, vmax_{maximum_velocity} {}

Vector2 Boid::position() const { return x_; }

Vector2 Boid::velocity() const { return v_; }

float Boid::maximum_velocity() const { return vmax_; }

Vector2 Boid::separation(const Boid& b_j, float s, float ds) const {
  
  if (std::abs(x_.dist(b_j.position())) < ds && x_.dist(b_j.position()) != 0 && boidshape_.getFillColor() != sf::Color::Black) {
    Vector2 v1 = (b_j.position() - x_) * (-s);

    return v1;

  } else {
    Vector2 null{};

    return null;
  }
}

Vector2 Boid::allignment(const Boid& b_j, float a, float n, float d) const {
  if (n >= 1 && std::abs(x_.dist(b_j.position())) < d &&
      std::abs(x_.dist(b_j.position())) != 0 &&
      boidshape_.getFillColor() == b_j.c_shape().getFillColor() && boidshape_.getFillColor() != sf::Color::Black) {
    Vector2 v2 = (b_j.velocity() - v_) * (a / n);

    return v2;

  } else {
    Vector2 null{};

    return null;
  }
}

Vector2 Boid::cohesion(const Vector2& x_cm, float c) const {
  if (x_cm.x_axis() != 0 && x_cm.y_axis() != 0) {
    Vector2 v3 = (x_cm - x_) * c;

    return v3;

  } else {
    Vector2 null{};

    return null;
  }
}

float Boid::get_angle() const {
  if (v_.y_axis() != 0.f) {
    if (v_.x_axis() >= 0.f && v_.y_axis() > 0.f) {
      return (180. / M_PI) * std::atan(std::abs(v_.x_axis() / v_.y_axis())) +
             2 * (90.f - ((180. / M_PI) *
                          std::atan(std::abs(v_.x_axis() / v_.y_axis()))));
    }

    if (v_.x_axis() < 0.f && v_.y_axis() > 0.f) {
      return ((180. / M_PI) * std::atan(std::abs(v_.x_axis() / v_.y_axis()))) +
             2 * (90.f - ((180. / M_PI) *
                          std::atan(std::abs(v_.x_axis() / v_.y_axis())))) +
             2 * ((180. / M_PI) *
                  std::atan(std::abs(v_.x_axis() / v_.y_axis())));
    }

    if (v_.x_axis() <= 0.f && v_.y_axis() < 0.f) {
      return ((180. / M_PI) * std::atan(std::abs(v_.x_axis() / v_.y_axis()))) +
             4 * (90.f - ((180. / M_PI) *
                          std::atan(std::abs(v_.x_axis() / v_.y_axis())))) +
             2 * ((180. / M_PI) *
                  std::atan(std::abs(v_.x_axis() / v_.y_axis())));
      ;
    }

    if (v_.x_axis() > 0.f && v_.y_axis() < 0.f) {
      return ((180. / M_PI) * std::atan(std::abs(v_.x_axis() / v_.y_axis())));
    }

  } else {
    if (v_.x_axis() < 0.f && v_.y_axis() == 0.f) {
      return 270.f;
    }

    if (v_.x_axis() > 0.f && v_.y_axis() == 0.f) {
      return 90.f;
    }

    if (v_.x_axis() == 0.f && v_.y_axis() == 0.f) {
      return 0.f;
    }
  }

  return 180.f;
}

void Boid::limit_velocity() {
  if (v_.mod() > vmax_) {
    v_ = v_ * 0.5;
  }
}

void Boid::change_velocity(const Vector2& vec) { v_ += vec; }

void Boid::change_position(const Vector2& vec) { x_ += vec; }

bool Boid::operator==(const Boid& b) const {
  return (x_ == b.x_ && v_ == b.v_ && vmax_ == b.vmax_);
};

sf::CircleShape& Boid::shape() { return boidshape_; }

const sf::CircleShape& Boid::c_shape() const { return boidshape_; }

void Boid::setRadius(float r) { boidshape_.setRadius(r); }

void Boid::setPointCount() { boidshape_.setPointCount(3); }

void Boid::setOrigin(float x, float y) { boidshape_.setOrigin(x, y); }

void Boid::setScale(float x, float y) { boidshape_.setScale(x, y); }

void Boid::setPosition(const Vector2& vec) {
  boidshape_.setPosition(sf::Vector2f{vec.x_axis(), vec.y_axis()});
};

void Boid::setRotation() { boidshape_.setRotation(get_angle()); }

void Boid::setFillColor(const sf::Color& color) {
  boidshape_.setFillColor(color);
}
}  // namespace pr