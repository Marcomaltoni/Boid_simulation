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
  assert(closeness_parameter_ >= 50.f && closeness_parameter_ <= 200.f &&
         distance_of_separation_ >= 25.f && distance_of_separation_ <= 40.f &&
         separation_parameter_ >= 0.005f && separation_parameter_ <= 0.08f &&
         allignment_parameter_ >= 0.2f && allignment_parameter_ <= 0.8f &&
         cohesion_parameter_ >= 0.0001f && cohesion_parameter_ <= 0.001f);
};

std::vector<Boid> Flock::all_boids() const { return boids_; };

std::size_t Flock::size() const { return boids_.size(); }

Boid Flock::single_boid(int number_of_boid) const {
  return boids_[number_of_boid];
}

void Flock::push_back(const Boid& new_boid) { boids_.push_back(new_boid); }

float Flock::close_boids(const Boid& chosen_boid) const {
  return count_if(boids_.begin(), boids_.end(),
                  [this, &chosen_boid](const Boid& other_boid) {
                    const float distance =
                        chosen_boid.position().distance(other_boid.position());

                    return distance < this->closeness_parameter_ &&
                           distance != 0;
                  });
}

Vector2 Flock::find_centermass(const Boid& chosen_boid) const {
  std::vector<Vector2> near_boids;

  for (const Boid& other_boid : boids_) {
    if (chosen_boid.position().distance(other_boid.position()) <
            closeness_parameter_ &&
        chosen_boid.position().distance(other_boid.position()) != 0.f &&
        other_boid.get_shape().getFillColor() != sf::Color::Black) {
      near_boids.push_back(other_boid.position());
    }
    assert(near_boids.size() <= boids_.size());
  }

  if (!near_boids.empty()) {
    const Vector2 mass_center =
        std::accumulate(near_boids.begin(), near_boids.end(), Vector2{0.f, 0.f},
                        [](const Vector2& a, const Vector2& b) {
                          const float x = a.x_axis() + b.x_axis();
                          const float y = a.y_axis() + b.y_axis();

                          return Vector2{x, y};
                        }) *
        (1.f / close_boids(chosen_boid));

    return mass_center;

  } else {
    return (chosen_boid.position());
  }
}

Vector2 Flock::find_separation(const Boid& chosen_boid) const {
  Vector2 null{};

  for (const Boid& other_boid : boids_) {
    null += chosen_boid.separation(other_boid, separation_parameter_,
                                   distance_of_separation_);
  }

  return null;
}

Vector2 Flock::find_allignment(const Boid& chosen_boid) const {
  Vector2 null{};

  for (const Boid& other_boid : boids_) {
    null +=
        chosen_boid.allignment(other_boid, allignment_parameter_,
                               close_boids(chosen_boid), closeness_parameter_);
  }

  return null;
}

Vector2 Flock::find_cohesion(const Boid& chosen_boid) const {
  const Vector2 velocity_offset =
      find_separation(chosen_boid) + find_allignment(chosen_boid) +
      chosen_boid.cohesion(find_centermass(chosen_boid), cohesion_parameter_);

  return velocity_offset;
}

bool Flock::is_predator(const Boid& chosen_boid) const {
  if (chosen_boid.get_shape().getFillColor() == sf::Color::Red) {
    const float predator_distance = 300.f;

    return std::any_of(
        boids_.begin(), boids_.end(),
        [predator_distance, &chosen_boid](const Boid& other_boid) {
          const float distance =
              chosen_boid.position().distance(other_boid.position());

          return other_boid.get_shape().getFillColor() == sf::Color::Black &&
                 distance < predator_distance && distance != 0;
        });

  } else {
    return false;
  }
}

void Flock::in_limits(Boid& chosen_boid, unsigned int window_height,
                      unsigned int window_width) {
  const float max_height = static_cast<float>(window_height);
  const float max_width = static_cast<float>(window_width);

  if (is_predator(chosen_boid) == false) {
    for (Boid& other_boid : boids_) {
      if (other_boid.position().distance(chosen_boid.position()) <
          closeness_parameter_) {
        if (chosen_boid.get_shape().getPosition().x >= max_height) {
          other_boid.change_velocity(
              Vector2{-std::abs(2.f * other_boid.velocity().x_axis()), 0.f});
          assert(other_boid.velocity().x_axis() <= 0.f);
        }

        if (chosen_boid.get_shape().getPosition().x <= 0.f) {
          other_boid.change_velocity(
              Vector2{std::abs(2.0f * other_boid.velocity().x_axis()), 0.f});
          assert(other_boid.velocity().x_axis() >= 0.f);
        }

        if (chosen_boid.get_shape().getPosition().y >= max_width) {
          other_boid.change_velocity(
              Vector2{0.f, -std::abs(2.f * other_boid.velocity().y_axis())});
          assert(other_boid.velocity().y_axis() <= 0.f);
        }

        if (chosen_boid.get_shape().getPosition().y <= 0.f) {
          other_boid.change_velocity(
              Vector2{0.f, std::abs(2.0f * other_boid.velocity().y_axis())});
          assert(other_boid.velocity().y_axis() >= 0.f);
        }
      }
    }

  } else {
    if (chosen_boid.get_shape().getPosition().x > max_height) {
      chosen_boid.change_position(Vector2{-max_height, 0.f});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f{0.f, chosen_boid.position().y_axis()});
    }

    if (chosen_boid.get_shape().getPosition().x < 0.f) {
      chosen_boid.change_position(Vector2{max_height, 0.f});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f{max_height, chosen_boid.position().y_axis()});
    }

    if (chosen_boid.get_shape().getPosition().y > max_width) {
      chosen_boid.change_position(Vector2{0.f, -max_width});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f{chosen_boid.position().x_axis(), 0.f});
    }

    if (chosen_boid.get_shape().getPosition().y < 0.f) {
      chosen_boid.change_position(Vector2{0.f, max_width});
      chosen_boid.set_shape().setPosition(
          sf::Vector2f{chosen_boid.position().x_axis(), max_width});
    }
  }
}

Vector2 Flock::evolve(Boid& chosen_boid, float delta_time) {
  if (close_boids(chosen_boid) != 0.f) {
    chosen_boid.change_velocity(find_cohesion(chosen_boid));

    const Vector2 position_offset = chosen_boid.velocity() * delta_time;
    chosen_boid.change_position(position_offset);

    chosen_boid.setRotation();

    return position_offset;

  } else {
    const Vector2 position_offset = chosen_boid.velocity() * delta_time;
    chosen_boid.change_position(position_offset);

    return position_offset;
  }
}

Simulation_state Flock::state() const {
  if (boids_.size() > 2) {
    std::vector<float> velocities;
    std::vector<float> distances;

    for (const Boid& boid : all_boids()) {
      velocities.push_back(boid.velocity().lenght_of_vector());

      auto boid_position = std::find(boids_.begin(), boids_.end(), boid);
      std::size_t number_of_boid = std::distance(boids_.begin(), boid_position);

      for (std::size_t i{number_of_boid + 1}; i < boids_.size(); ++i) {
        const float distance =
            boid.position().distance(single_boid(i).position());
        distances.push_back(distance);
      }
    }

    float vsize_f = static_cast<float>(velocities.size());
    float psize_f = static_cast<float>(distances.size());

    const float medium_velocity =
        (std::accumulate(velocities.begin(), velocities.end(), 0.f)) / vsize_f;

    float quadratic_difference_v = std::accumulate(
        velocities.begin(), velocities.end(), 0.f,
        [&medium_velocity](float initial_value, float single_velocity) {
          return initial_value +
                 std::pow((medium_velocity - single_velocity), 2.f);
        });

    const float err_velocity =
        std::sqrt((1.f / ((vsize_f - 1.f) * vsize_f)) * quadratic_difference_v);

    const float medium_distance =
        (std::accumulate(distances.begin(), distances.end(), 0.f)) / psize_f;

    float quadratic_difference_p = std::accumulate(
        distances.begin(), distances.end(), 0.f,
        [&medium_distance](float initial_value, float single_distance) {
          return initial_value +
                 std::pow((medium_distance - single_distance), 2.f);
        });

    const float err_distance =
        std::sqrt((1.f / ((psize_f - 1.f) * psize_f)) * quadratic_difference_p);

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

    const float delta_time = time.asSeconds() * 210.f;

    Vector2 position_offset = evolve(boid, delta_time);

    in_limits(boid, window_height, window_width);

    const sf::Vector2f graphic_offset{position_offset.x_axis(),
                                      position_offset.y_axis()};
    boid.set_shape().move(graphic_offset);
  }
}
}  // namespace pr
