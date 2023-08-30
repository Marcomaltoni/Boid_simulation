#ifndef FLOCK_HPP
#define FLOCK_HPP
#include <array>
#include <cmath>
#include <numeric>
#include <vector>

#include "boid.hpp"

namespace pr {

struct Simulation_state {
  float medium_velocity;
  float err_velocity;
  float medium_distance;
  float err_distance;
};

float quadratic_difference(const std::vector<float>& generic_vector);

class Flock {
  const float closeness_parameter_;

  const float distance_of_separation_;

  const float separation_parameter_;

  const float allignment_parameter_;

  const float cohesion_parameter_;

  std::vector<Boid> boids_;

 public:
  Flock(const float distance, const float ds_parameter, const float s_parameter,
        const float a_parameter, const float c_parameter);

  std::vector<Boid> all_boids() const;

  std::size_t size() const;

  Boid single_boid(int number_of_boid) const;

  void push_back(const Boid& new_boid);

  float close_boids_angle(const Boid& chosen_boid) const;

  float close_boids_360(const Boid& chosen_boid) const;

  Vector2 find_centermass(const Boid& chosen_boid) const;

  Vector2 find_separation(const Boid& chosen_boid) const;

  Vector2 find_allignment(const Boid& chosen_boid) const;

  Vector2 find_cohesion(const Boid& chosen_boid) const;

  bool is_predator(const Boid& chosen_boid) const;

  void in_limits(Boid& chosen_boid, unsigned int window_height,
                 unsigned int window_width);

  Vector2 evolve(Boid& chosen_boid, float delta_time);

  std::vector<float> extract_velocities() const;

  std::vector<float> extract_distances() const;

  Simulation_state state() const;

  void update(sf::Time const& time, unsigned int window_height,
              unsigned int window_width);
};
}  // namespace pr

#endif