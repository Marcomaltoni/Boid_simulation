#ifndef FLOCK_HPP
#define FLOCK_HPP
#include <array>
#include <vector>

#include "boid.hpp"
#include <numeric>
#include <cmath>

namespace pr {

struct Simulation_state {
  float medium_velocity;
  float err_velocity;
  float medium_distance;
  float err_distance;
};

inline float quadratic_difference(const std::vector<float>& generic_vector){
   const float medium_value =
        (std::accumulate(generic_vector.begin(), generic_vector.end(), 0.f)) / static_cast<float>(generic_vector.size());

    float quadratic_difference = std::accumulate(
        generic_vector.begin(), generic_vector.end(), 0.f,
        [&medium_value](float initial_value, float single_value) {
          return initial_value +
                 std::pow((medium_value - single_value), 2.f);
        }); 

        return quadratic_difference;
};

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

  float close_boids(const Boid& chosen_boid) const;

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