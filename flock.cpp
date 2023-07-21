#include "flock.hpp"

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace pr {
Flock::Flock(const float distanceance, const float ds_parameter,
             const float s_parameter, const float a_parameter,
             const float c_parameter)
    : d_{distanceance},
      ds_{ds_parameter},
      s_{s_parameter},
      a_{a_parameter},
      c_{c_parameter} {
  assert(d_ > 0 && ds_ > 0 && s_ > 0 && a_ > 0 && c_ > 0);
};

std::vector<Boid> Flock::n_birds() const { return birds_; };

int Flock::size() const { return birds_.size(); }

Boid Flock::single_bird(int n) const { return birds_[n]; }

void Flock::push_back(const Boid& b1) { birds_.push_back(b1); }

float Flock::n_near(const Boid& i) const {
  return count_if(birds_.begin(), birds_.end(), [this, &i](const Boid& x) {
    float distanceance = std::abs(i.position().distance(x.position()));

    return distanceance < this->d_ && distanceance != 0;
  });
}

Vector2 Flock::find_centermass(const Boid& i) const {
  std::vector<Vector2> near;
  
  for (const Boid& x : birds_) {
    if (std::abs(i.position().distance(x.position())) < d_ &&
        std::abs(i.position().distance(x.position())) != 0 &&
        x.c_shape().getFillColor() != sf::Color::Black) {
      near.push_back(x.position());
    }
    assert(near.size() <= birds_.size());
  }

  if (near.size() > 0) {
    Vector2 mass_center =
        std::accumulate(near.begin(), near.end(), Vector2{0.f, 0.f},
                        [](const Vector2& a, const Vector2& b) {
                          float x = a.x_axis() + b.x_axis();
                          float y = a.y_axis() + b.y_axis();

                          return Vector2{x, y};
                        }) *
        (1 / n_near(i));

    return mass_center;

  } else {
    return (i.position());
  }
}

Vector2 Flock::find_separation(const Boid& i) const {
  Vector2 null{};

  for (Boid b : birds_) {
    null += i.separation(b, s_, ds_);
  }

  return null;
}
Vector2 Flock::find_allignment(const Boid& i) const {
  Vector2 null{};

  for (Boid b : birds_) {
    null += i.allignment(b, a_, n_near(i), d_);
  }

  return null;
}
Vector2 Flock::find_cohesion(const Boid& i) const {
  Vector2 vec = find_separation(i) + find_allignment(i) +
                i.cohesion(find_centermass(i), c_);

  return vec;
}

bool Flock::is_predator(const Boid& x) const {
  if (x.c_shape().getFillColor() == sf::Color::Red) {
    return std::any_of(birds_.begin(), birds_.end(), [this, &x](const Boid& y) {
      float distanceance = std::abs(x.position().distance(y.position()));

      return y.c_shape().getFillColor() == sf::Color::Black &&
             distanceance < this->d_ && distanceance != 0;
    });

  } else {
    return false;
  }
}

void Flock::in_limits(Boid& b, unsigned int i, unsigned int j) {
  float max_x = static_cast<float>(i);
  float max_y = static_cast<float>(j);

  if (is_predator(b) == false) {
    for (Boid& x : birds_) {
      if (x.position().distance(b.position()) < d_) {
        if (b.shape().getPosition().x >= max_x) {
          x.change_velocity(
              Vector2{-std::abs(2.f * x.velocity().x_axis()), 0.f});
        }

        if (b.shape().getPosition().x <= 0.f) {
          x.change_velocity(
              Vector2{std::abs(2.0f * x.velocity().x_axis()), 0.f});
        }

        if (b.shape().getPosition().y >= max_y) {
          x.change_velocity(
              Vector2{0.f, -std::abs(2.f * x.velocity().y_axis())});
        }

        if (b.shape().getPosition().y <= 0.f) {
          x.change_velocity(
              Vector2{0.f, std::abs(2.0f * x.velocity().y_axis())});
        }
      }
    }

  } else {
    if (b.shape().getPosition().x > max_x) {
      b.change_position(Vector2{-max_x, 0.f});
      b.shape().setPosition(sf::Vector2f(0.f, b.position().y_axis()));
    }

    if (b.shape().getPosition().x < 0.f) {
      b.change_position(Vector2{max_x, 0.f});
      b.shape().setPosition(sf::Vector2f(max_x, b.position().y_axis()));
    }

    if (b.shape().getPosition().y > max_y) {
      b.change_position(Vector2{0.f, -max_y});
      b.shape().setPosition(sf::Vector2f(b.position().x_axis(), 0.f));
    }

    if (b.shape().getPosition().y < 0.f) {
      b.change_position(Vector2{0.f, max_y});
      b.shape().setPosition(sf::Vector2f(b.position().x_axis(), max_y));
    }
  }
}

Vector2 Flock::evolve(Boid& i, float dt) {
  if (n_near(i) != 0) {
    i.change_velocity(find_cohesion(i));

    Vector2 offset = i.velocity() * dt;
    i.change_position(offset);

    i.setRotation();

    return offset;

  } else {
    Vector2 offset = i.velocity() * dt;
    i.change_position(offset);

    return offset;
  }
}

Result Flock::state() const {
  if (birds_.size() > 2) {
    std::vector<float> v;
    std::vector<float> p;

    for (const Boid& x : n_birds()) {
      v.push_back(x.velocity().mod());

      auto it = std::find(birds_.begin(), birds_.end(), x);
      size_t number_of_boid = std::distance(birds_.begin(), it);
      for (size_t i{number_of_boid + 1}; i < birds_.size(); ++i) {
        float distanceance = x.position().distance(single_bird(i).position());
        p.push_back(distanceance);
      }
    }

    float vsize_f = static_cast<float>(v.size());
    float psize_f = static_cast<float>(p.size());

    float medium_velocity =
        (std::accumulate(v.begin(), v.end(), 0.f)) / vsize_f;

    float quadratic_differencev = std::accumulate(
        v.begin(), v.end(), 0.f, [&medium_velocity](float n, float t) {
          return n + std::pow((medium_velocity - t), 2.f);
        });

    float err_velocity =
        std::sqrt((1.f / ((vsize_f - 1.f) * vsize_f)) * quadratic_differencev);

    float medium_distanceance =
        (std::accumulate(p.begin(), p.end(), 0.f)) / psize_f;

    float quadratic_differencep = std::accumulate(
        p.begin(), p.end(), 0.f, [&medium_distanceance](float m, float s) {
          return m + std::pow((medium_distanceance - s), 2.f);
        });

    float err_distanceance =
        std::sqrt((1.f / ((psize_f - 1.f) * psize_f)) * quadratic_differencep);

    return {medium_velocity, err_velocity, medium_distanceance, err_distanceance};
  } else {
    return {0., 0., 0., 0.};
  }
}

void Flock::update(sf::Time const& time, unsigned int i, unsigned int j) {
  for (Boid& x : birds_) {
    x.limit_velocity();
    x.setRotation();

    float tim = time.asSeconds() * 210;

    Vector2 offset = evolve(x, tim);

    in_limits(x, i, j);

    sf::Vector2f vec{offset.x_axis(), offset.y_axis()};
    x.shape().move(vec);
  }
}
}  // namespace pr
