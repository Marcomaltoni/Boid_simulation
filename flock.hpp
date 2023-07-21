#ifndef FLOCK_HPP
#define FLOCK_HPP
#include <array>
#include <vector>

#include "boid.hpp"

namespace pr {

struct Result {
  float medium_velocity;
  float medium_distance;
  float err_velocity;
  float err_distance;
};

class Flock {
  const float d_;

  const float ds_;

  const float s_;

  const float a_;

  const float c_;

  std::vector<Boid> birds_;

 public:
  Flock(const float distance, const float ds_parameter, const float s_parameter,
        const float a_parameter, const float c_parameter);

  std::vector<Boid> n_birds() const;

  int size() const;

  Boid single_bird(int n) const;

  void push_back(const Boid& b1);

  float n_near(const Boid& i) const;

  Vector2 find_centermass(const Boid& i) const;

  Vector2 find_separation(const Boid& i) const;

  Vector2 find_allignment(const Boid& i) const;

  Vector2 find_cohesion(const Boid& i) const;

  bool is_predator(const Boid& x) const;

  void in_limits(Boid& b, unsigned int i, unsigned int j);

  Vector2 evolve(Boid& i, float dt);

  Result state() const;

  void update(sf::Time const& time, unsigned int i, unsigned int j);
};
}  // namespace pr

#endif