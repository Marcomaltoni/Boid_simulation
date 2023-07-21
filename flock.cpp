#include "flock.hpp"

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace pr {
Flock::Flock(const float distance, const float ds_parameter,
             const float s_parameter, const float a_parameter,
             const float c_parameter)
    : closeness_parameter_{distance},
      distance_of_separation_{ds_parameter},
      separation_parameter_{s_parameter},
      allignment_parameter_{a_parameter},
      cohesion_parameter_{c_parameter} {
  assert(closeness_parameter_ > 0.f && distance_of_separation_ > 0.f &&
         separation_parameter_ > 0.f && allignment_parameter_ > 0.f &&
         cohesion_parameter_ > 0.f);
};

std::vector<Boid> Flock::all_boids() const { return boids_; };

int Flock::size() const { return boids_.size(); }

Boid Flock::single_boid(int number_of_boid) const {
  return boids_[number_of_boid];
}

void Flock::push_back(const Boid& new_boid) { boids_.push_back(new_boid); }

float Flock::close_boids(const Boid& chosen_boid) const {
  return count_if(
      boids_.begin(), boids_.end(),
      [this, &chosen_boid](const Boid& other_boid) {
        float distanceance =
            std::abs(chosen_boid.position().distance(other_boid.position()));

        return distanceance < this->closeness_parameter_ && distanceance != 0;
      });
}

Vector2 Flock::find_centermass(const Boid& chosen_boid) const {
  std::vector<Vector2> near_boids;

  for (const Boid& other_boid : boids_) {
    if (std::abs(chosen_boid.position().distance(other_boid.position())) <
            closeness_parameter_ &&
        std::abs(chosen_boid.position().distance(other_boid.position())) != 0 &&
        other_boid.get_shape().getFillColor() != sf::Color::Black) {
      near_boids.push_back(other_boid.position());
    }
    assert(near_boids.size() <= boids_.size());
  }

  if (!near_boids.empty()) {
    Vector2 mass_center =
        std::accumulate(near_boids.begin(), near_boids.end(), Vector2{0.f, 0.f},
                        [](const Vector2& a, const Vector2& b) {
                          float x = a.x_axis() + b.x_axis();
                          float y = a.y_axis() + b.y_axis();

                          return Vector2{x, y};
                        }) *
        (1 / close_boids(chosen_boid));

    return mass_center;

  } else {
    return (chosen_boid.position());
  }
}

Vector2 Flock::find_separation(const Boid& chosen_boid) const {
  Vector2 null{};

  for (Boid other_boid : boids_) {
    null += chosen_boid.separation(other_boid, separation_parameter_,
                                   distance_of_separation_);
  }

  return null;
}

Vector2 Flock::find_allignment(const Boid& chosen_boid) const {
  Vector2 null{};

  for (Boid other_boid : boids_) {
    null +=
        chosen_boid.allignment(other_boid, allignment_parameter_,
                               close_boids(chosen_boid), closeness_parameter_);
  }

  return null;
}

Vector2 Flock::find_cohesion(const Boid& chosen_boid) const {
  Vector2 velocity_offset =
      find_separation(chosen_boid) + find_allignment(chosen_boid) +
      chosen_boid.cohesion(find_centermass(chosen_boid), cohesion_parameter_);

  return velocity_offset;
}

bool Flock::is_predator(const Boid& chosen_boid) const {
  if (chosen_boid.get_shape().getFillColor() == sf::Color::Red) {
    return std::any_of(
        boids_.begin(), boids_.end(),
        [this, &chosen_boid](const Boid& other_boid) {
          float distance =
              std::abs(chosen_boid.position().distance(other_boid.position()));

          return other_boid.get_shape().getFillColor() == sf::Color::Black &&
                 distance < this->closeness_parameter_ && distance != 0;
        });

  } else {
    return false;
  }
}

void Flock::in_limits(Boid& chosen_boid, unsigned int window_height,
                      unsigned int window_width) {
  float max_height = static_cast<float>(window_height);
  float max_width = static_cast<float>(window_width);

  if (is_predator(chosen_boid) == false) {
    for (Boid& other_boid : boids_) {
      if (other_boid.position().distance(chosen_boid.position()) <
          closeness_parameter_) {
        if (chosen_boid.set_shape().getPosition().x >= max_height) {
          other_boid.change_velocity(
              Vector2{-std::abs(2.f * other_boid.velocity().x_axis()), 0.f});
        }

        if (chosen_boid.set_shape().getPosition().x <= 0.f) {
          other_boid.change_velocity(
              Vector2{std::abs(2.0f * other_boid.velocity().x_axis()), 0.f});
        }

        if (chosen_boid.set_shape().getPosition().y >= max_width) {
          other_boid.change_velocity(
              Vector2{0.f, -std::abs(2.f * other_boid.velocity().y_axis())});
        }

        if (chosen_boid.set_shape().getPosition().y <= 0.f) {
          other_boid.change_velocity(
              Vector2{0.f, std::abs(2.0f * other_boid.velocity().y_axis())});
        }
      }
    }

  } else {
    if (chosen_boid.set_shape().getPosition().x > max_height) {
      chosen_boid.change_position(Vector2{-max_height, 0.f});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f(0.f, chosen_boid.position().y_axis()));
    }

    if (chosen_boid.set_shape().getPosition().x < 0.f) {
      chosen_boid.change_position(Vector2{max_height, 0.f});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f(max_height, chosen_boid.position().y_axis()));
    }

    if (chosen_boid.set_shape().getPosition().y > max_width) {
      chosen_boid.change_position(Vector2{0.f, -max_width});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f(chosen_boid.position().x_axis(), 0.f));
    }

    if (chosen_boid.set_shape().getPosition().y < 0.f) {
      chosen_boid.change_position(Vector2{0.f, max_width});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f(chosen_boid.position().x_axis(), max_width));
    }
  }
}

Vector2 Flock::evolve(Boid& chosen_boid, float delta_time) {
  if (close_boids(chosen_boid) != 0.f) {
    chosen_boid.change_velocity(find_cohesion(chosen_boid));

    Vector2 position_offset = chosen_boid.velocity() * delta_time;
    chosen_boid.change_position(position_offset);

    chosen_boid.setRotation();

    return position_offset;

  } else {
    Vector2 position_offset = chosen_boid.velocity() * delta_time;
    chosen_boid.change_position(position_offset);

    return position_offset;
  }
}

Result Flock::state() const {
  if (boids_.size() > 2) {
    std::vector<float> velocities;
    std::vector<float> distances;

    for (const Boid& boid : all_boids()) {
      velocities.push_back(boid.velocity().lenght_of_vector());

      auto boid_position = std::find(boids_.begin(), boids_.end(), boid);
      size_t number_of_boid = std::distance(boids_.begin(), boid_position);
      for (size_t i{number_of_boid + 1}; i < boids_.size(); ++i) {
        float distance = boid.position().distance(single_boid(i).position());
        distances.push_back(distance);
      }
    }

    float vsize_f = static_cast<float>(velocities.size());
    float psize_f = static_cast<float>(distances.size());

    float medium_velocity =
        (std::accumulate(velocities.begin(), velocities.end(), 0.f)) / vsize_f;

    float quadratic_differencev = std::accumulate(
        velocities.begin(), velocities.end(), 0.f,
        [&medium_velocity](float initial_value, float single_velocity) {
          return initial_value +
                 std::pow((medium_velocity - single_velocity), 2.f);
        });

    float err_velocity =
        std::sqrt((1.f / ((vsize_f - 1.f) * vsize_f)) * quadratic_differencev);

    float medium_distance =
        (std::accumulate(distances.begin(), distances.end(), 0.f)) / psize_f;

    float quadratic_differencep = std::accumulate(
        distances.begin(), distances.end(), 0.f,
        [&medium_distance](float initial_value, float single_distance) {
          return initial_value +
                 std::pow((medium_distance - single_distance), 2.f);
        });

    float err_distance =
        std::sqrt((1.f / ((psize_f - 1.f) * psize_f)) * quadratic_differencep);

    return {medium_velocity, err_velocity, medium_distance, err_distance};
  } else {
    return {0., 0., 0., 0.};
  }
}

void Flock::update(sf::Time const& time, unsigned int window_height,
                   unsigned int window_width) {
  for (Boid& boid : boids_) {
    boid.limit_velocity();
    boid.setRotation();

    float delta_time = time.asSeconds() * 210;

    Vector2 position_offset = evolve(boid, delta_time);

    in_limits(boid, window_height, window_width);

    sf::Vector2f graphic_offset{position_offset.x_axis(),
                                position_offset.y_axis()};
    boid.set_shape().move(graphic_offset);
  }
}
}  // namespace pr
