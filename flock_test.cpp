#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flock.hpp"

#include "doctest.h"

TEST_CASE("Testing the close_boids function") {
  pr::Flock flock{10, 1, 1, 1, 1};

  const pr::Vector2 v1{5, 4};
  const pr::Vector2 v2{10, 6};
  const pr::Vector2 v3{7, 9};
  const pr::Vector2 v4{13, 10};

  const pr::Boid b1{v1, v1, 1};
  const pr::Boid b2{v2, v2, 1};
  const pr::Boid b3{v3, v3, 1};
  const pr::Boid b4{v4, v4, 1};

  flock.push_back(b1);
  flock.push_back(b2);
  flock.push_back(b3);
  flock.push_back(b4);

  CHECK(flock.close_boids(b1) == 2);
}

TEST_CASE("Testing the find_centermass() function") {
  SUBCASE("case 1") {
    pr::Flock flock{10, 1, 1, 1, 1};

    const pr::Vector2 v1{5, 4};
    const pr::Vector2 v2{10, 6};
    const pr::Vector2 v3{7, 9};
    const pr::Vector2 v4{14, 10};

    const pr::Boid b1{v1, v1, 1};
    const pr::Boid b2{v2, v2, 1};
    const pr::Boid b3{v3, v3, 1};
    const pr::Boid b4{v4, v4, 1};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(flock.close_boids(b1) == 2);

    CHECK(center_mass.x_axis() == doctest::Approx(8.5));
    CHECK(center_mass.y_axis() == doctest::Approx(7.5));
  }

  SUBCASE("case 2") {
    pr::Flock flock{10, 1, 1, 1, 1};

    const pr::Vector2 v1{5.5, 4.6};
    const pr::Vector2 v2{10.1, 6.8};
    const pr::Vector2 v3{7.8, 9.7};
    const pr::Vector2 v4{13.5, 10.6};
    const pr::Vector2 v5{3.1, 5.2};

    const pr::Boid b1{v1, v1, 1};
    const pr::Boid b2{v2, v2, 1};
    const pr::Boid b3{v3, v3, 1};
    const pr::Boid b4{v4, v4, 1};
    const pr::Boid b5{v5, v5, 1};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(flock.close_boids(b1) == 3);

    CHECK(center_mass.x_axis() == doctest::Approx(7.000).epsilon(0.001));
    CHECK(center_mass.y_axis() == doctest::Approx(7.233).epsilon(0.001));
  }

  SUBCASE("case 3") {
    pr::Flock flock{10, 1, 1, 1, 1};

    const pr::Vector2 v1{5.5, 4.6};
    const pr::Vector2 v3{7.6, 9.5};
    const pr::Vector2 v4{13.5, 10.6};

    const pr::Boid b1{v1, v1, 1};
    const pr::Boid b3{v3, v3, 1};
    const pr::Boid b4{v4, v4, 1};

    flock.push_back(b1);
    flock.push_back(b3);
    flock.push_back(b4);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(center_mass.x_axis() == doctest::Approx(7.600).epsilon(0.001));
    CHECK(center_mass.y_axis() == doctest::Approx(9.500).epsilon(0.001));
  }

  SUBCASE("case 4") {
    pr::Flock flock{10, 1, 1, 1, 1};

    const pr::Vector2 v1{5, 6};
    const pr::Vector2 v2{1000, 1000};
    const pr::Vector2 v3{3000, 3000};
    const pr::Vector2 v4{2000, 2000};

    const pr::Boid b1{v1, v1, 1};
    const pr::Boid b2{v2, v2, 1};
    const pr::Boid b3{v3, v3, 1};
    const pr::Boid b4{v4, v4, 1};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(center_mass.x_axis() == doctest::Approx(5.000).epsilon(0.001));
    CHECK(center_mass.y_axis() == doctest::Approx(6.000).epsilon(0.001));
  }
}

TEST_CASE("Testing the is_predator function") {
  const pr::Vector2 v1{2, 3};
  const pr::Vector2 v2{4, 5};
  const pr::Vector2 v3{5, 6};
  const pr::Vector2 v4{1000, 1000};
  const pr::Vector2 v5{1, 1};

  pr::Boid b1{v1, v5, 1};
  pr::Boid b2{v2, v5, 1};
  pr::Boid b3{v3, v5, 1};
  pr::Boid b4{v4, v5, 1};

  b1.set_shape().setFillColor(sf::Color::Red);
  b2.set_shape().setFillColor(sf::Color::Red);
  b3.set_shape().setFillColor(sf::Color::Black);
  b4.set_shape().setFillColor(sf::Color::Black);

  pr::Flock flock{50, 10, 0.5, 0.5, 0.5};

  flock.push_back(b1);
  flock.push_back(b2);
  flock.push_back(b3);
  flock.push_back(b4);

  CHECK(flock.is_predator(b1) == true);
  CHECK(flock.is_predator(b2) == true);
  CHECK(flock.is_predator(b3) == false);
  CHECK(flock.is_predator(b4) == false);
}

TEST_CASE("Testing the evolve function") {
  SUBCASE("case 1") {
    const pr::Vector2 v1{1.58, 50};
    const pr::Vector2 v2{10, 5};
    const pr::Vector2 v3{2, 45};
    const pr::Vector2 v4{20, 2};
    const pr::Vector2 v5{100, 200};
    const pr::Vector2 v6{1, 1};
    const pr::Vector2 v7{120, 300};
    const pr::Vector2 v8{1, 1};
    const pr::Vector2 v9{1.65, 51};
    const pr::Vector2 v10{5, 8};

    pr::Boid b1{v1, v2, 1};
    const pr::Boid b2{v3, v4, 1};
    const pr::Boid b3{v5, v6, 1};
    const pr::Boid b4{v7, v8, 1};
    const pr::Boid b5{v9, v10, 1};

    pr::Flock flock{10, 2, 0.5, 0.4, 0.3};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);

    CHECK(flock.close_boids(b1) == 2);

    CHECK(b1.separation(b2, 0.5, 2).x_axis() == 0);
    CHECK(b1.separation(b2, 0.5, 2).y_axis() == 0);
    CHECK(b1.separation(b5, 0.5, 2).x_axis() ==
          doctest::Approx(-0.035).epsilon(0.001));
    CHECK(b1.separation(b5, 0.5, 2).y_axis() ==
          doctest::Approx(-0.500).epsilon(0.001));

    CHECK(b1.allignment(b2, 0.4, flock.close_boids(b1), 10).x_axis() == 2);
    CHECK(b1.allignment(b2, 0.4, flock.close_boids(b1), 10).y_axis() ==
          doctest::Approx(-0.600).epsilon(0.001));
    CHECK(b1.allignment(b5, 0.4, flock.close_boids(b1), 10).x_axis() == -1);
    CHECK(b1.allignment(b5, 0.4, flock.close_boids(b1), 10).y_axis() ==
          doctest::Approx(0.600).epsilon(0.001));

    CHECK(flock.find_centermass(b1).x_axis() ==
          doctest::Approx(1.825).epsilon(0.001));
    CHECK(flock.find_centermass(b1).y_axis() ==
          doctest::Approx(48.000).epsilon(0.001));

    CHECK(b1.cohesion(flock.find_centermass(b1), 0.3).x_axis() ==
          doctest::Approx(0.0735).epsilon(0.0001));
    CHECK(b1.cohesion(flock.find_centermass(b1), 0.3).y_axis() ==
          doctest::Approx(-0.600).epsilon(0.001));

    CHECK(flock.find_separation(b1).x_axis() ==
          doctest::Approx(-0.035).epsilon(0.001));
    CHECK(flock.find_separation(b1).y_axis() ==
          doctest::Approx(-0.500).epsilon(0.001));

    CHECK(flock.find_allignment(b1).x_axis() == 1);
    CHECK(flock.find_allignment(b1).y_axis() == 0);

    CHECK(flock.find_cohesion(b1).x_axis() ==
          doctest::Approx(1.0385).epsilon(0.0001));
    CHECK(flock.find_cohesion(b1).y_axis() ==
          doctest::Approx(-1.100).epsilon(0.0001));

    pr::Vector2 vec = flock.evolve(b1, 0.5);

    CHECK(vec.x_axis() == doctest::Approx(5.51925).epsilon(0.00001));
    CHECK(vec.y_axis() == doctest::Approx(1.950).epsilon(0.001));

    CHECK(b1.velocity().x_axis() == doctest::Approx(11.0385).epsilon(0.0001));
    CHECK(b1.velocity().y_axis() == doctest::Approx(3.900).epsilon(0.001));

    CHECK(b1.position().x_axis() == doctest::Approx(7.09925).epsilon(0.00001));
    CHECK(b1.position().y_axis() == doctest::Approx(51.950).epsilon(0.001));
  }

  SUBCASE("case 2") {
    const pr::Vector2 v1{50.65, 70.95};
    const pr::Vector2 v2{12.5, 16.7};
    const pr::Vector2 v3{60.55, 69.90};
    const pr::Vector2 v4{13.7, 18.5};
    const pr::Vector2 v5{55.55, 65.45};
    const pr::Vector2 v6{19.8, 20.2};
    const pr::Vector2 v7{100, 200};
    const pr::Vector2 v8{13.9, 15.7};
    const pr::Vector2 v9{300, 400};
    const pr::Vector2 v10{16.8, 17.9};
    const pr::Vector2 v11{51.55, 70.00};
    const pr::Vector2 v12{51.5, 53.5};
    const pr::Vector2 v13{50.05, 69.99};
    const pr::Vector2 v14{37.8, 38.9};

    pr::Boid b1{v1, v2, 1};
    const pr::Boid b2{v3, v4, 1};
    const pr::Boid b3{v5, v6, 1};
    const pr::Boid b4{v7, v8, 1};
    const pr::Boid b5{v9, v10, 1};
    const pr::Boid b6{v11, v12, 1};
    const pr::Boid b7{v13, v14, 1};

    pr::Flock flock{10, 2, 0.5, 0.4, 0.3};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);
    flock.push_back(b6);
    flock.push_back(b7);

    CHECK(flock.close_boids(b1) == 4);

    CHECK(b1.separation(b2, 0.5, 2).x_axis() == 0);
    CHECK(b1.separation(b2, 0.5, 2).y_axis() == 0);
    CHECK(b1.separation(b3, 0.5, 2).x_axis() == 0);
    CHECK(b1.separation(b3, 0.5, 2).y_axis() == 0);
    CHECK(b1.separation(b6, 0.5, 2).x_axis() ==
          doctest::Approx(-0.450).epsilon(0.001));
    CHECK(b1.separation(b6, 0.5, 2).y_axis() ==
          doctest::Approx(0.475).epsilon(0.001));
    CHECK(b1.separation(b7, 0.5, 2).x_axis() ==
          doctest::Approx(0.300).epsilon(0.001));
    CHECK(b1.separation(b7, 0.5, 2).y_axis() ==
          doctest::Approx(0.480).epsilon(0.001));

    CHECK(b1.allignment(b2, 0.4, flock.close_boids(b1), 10).x_axis() ==
          doctest::Approx(0.120).epsilon(0.001));
    CHECK(b1.allignment(b2, 0.4, flock.close_boids(b1), 10).y_axis() ==
          doctest::Approx(0.180).epsilon(0.001));
    CHECK(b1.allignment(b3, 0.4, flock.close_boids(b1), 10).x_axis() ==
          doctest::Approx(0.730).epsilon(0.001));
    CHECK(b1.allignment(b3, 0.4, flock.close_boids(b1), 10).y_axis() ==
          doctest::Approx(0.350).epsilon(0.001));
    CHECK(b1.allignment(b6, 0.4, flock.close_boids(b1), 10).x_axis() ==
          doctest::Approx(3.900).epsilon(0.001));
    CHECK(b1.allignment(b6, 0.4, flock.close_boids(b1), 10).y_axis() ==
          doctest::Approx(3.680).epsilon(0.001));
    CHECK(b1.allignment(b7, 0.4, flock.close_boids(b1), 10).x_axis() ==
          doctest::Approx(2.530).epsilon(0.001));
    CHECK(b1.allignment(b7, 0.4, flock.close_boids(b1), 10).y_axis() ==
          doctest::Approx(2.220).epsilon(0.001));

    CHECK(flock.find_centermass(b1).x_axis() ==
          doctest::Approx(54.425).epsilon(0.001));
    CHECK(flock.find_centermass(b1).y_axis() ==
          doctest::Approx(68.835).epsilon(0.001));

    CHECK(b1.cohesion(flock.find_centermass(b1), 0.3).x_axis() ==
          doctest::Approx(1.1325).epsilon(0.0001));
    CHECK(b1.cohesion(flock.find_centermass(b1), 0.3).y_axis() ==
          doctest::Approx(-0.6345).epsilon(0.0001));

    CHECK(flock.find_separation(b1).x_axis() ==
          doctest::Approx(-0.150).epsilon(0.001));
    CHECK(flock.find_separation(b1).y_axis() ==
          doctest::Approx(0.955).epsilon(0.001));

    CHECK(flock.find_allignment(b1).x_axis() ==
          doctest::Approx(7.280).epsilon(0.001));
    CHECK(flock.find_allignment(b1).y_axis() ==
          doctest::Approx(6.430).epsilon(0.001));

    CHECK(flock.find_cohesion(b1).x_axis() ==
          doctest::Approx(8.2625).epsilon(0.0001));
    CHECK(flock.find_cohesion(b1).y_axis() ==
          doctest::Approx(6.7505).epsilon(0.0001));

    pr::Vector2 vec = flock.evolve(b1, 0.05);

    CHECK(vec.x_axis() == doctest::Approx(1.038125).epsilon(0.000001));
    CHECK(vec.y_axis() == doctest::Approx(1.172525).epsilon(0.000001));

    CHECK(b1.velocity().x_axis() == doctest::Approx(20.7625).epsilon(0.0001));
    CHECK(b1.velocity().y_axis() == doctest::Approx(23.4505).epsilon(0.0001));

    CHECK(b1.position().x_axis() ==
          doctest::Approx(51.688125).epsilon(0.000001));
    CHECK(b1.position().y_axis() ==
          doctest::Approx(72.122525).epsilon(0.000001));
  }

  SUBCASE("case 3") {
    const pr::Vector2 v1{1, 2};
    const pr::Vector2 v2{1, 1};
    const pr::Vector2 v3{3, 4};
    const pr::Vector2 v4{1, 1};
    const pr::Vector2 v5{5, 6};
    const pr::Vector2 v6{1, 1};
    const pr::Vector2 v7{7, 8};
    const pr::Vector2 v8{1, 1};

    pr::Boid b1{v1, v2, 1};
    const pr::Boid b2{v3, v4, 1};
    const pr::Boid b3{v5, v6, 1};
    const pr::Boid b4{v7, v8, 1};

    pr::Flock flock{1, 0.5, 0.5, 0.4, 0.3};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);

    CHECK(flock.close_boids(b1) == 0);

    pr::Vector2 vec = flock.evolve(b1, 0.5);

    CHECK(vec.x_axis() == doctest::Approx(0.500).epsilon(0.001));
    CHECK(vec.y_axis() == doctest::Approx(0.500).epsilon(0.001));

    CHECK(b1.velocity().x_axis() == 1);
    CHECK(b1.velocity().y_axis() == 1);

    CHECK(b1.position().x_axis() == doctest::Approx(1.500).epsilon(0.001));
    CHECK(b1.position().y_axis() == doctest::Approx(2.500).epsilon(0.001));
  }
}

TEST_CASE("Testing the state() function") {
  SUBCASE("case 1") {
    const pr::Vector2 v1{2, 3};
    const pr::Vector2 v2{5, 6};
    const pr::Vector2 v3{4, 5};
    const pr::Vector2 v4{6, 7};
    const pr::Vector2 v5{8, 9};
    const pr::Vector2 v6{10, 11};

    const pr::Boid b1{v1, v2, 1};
    const pr::Boid b2{v3, v4, 1};
    const pr::Boid b3{v5, v6, 1};

    pr::Flock flock{10, 2, 0.5, 0.5, 0.5};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);

    pr::Simulation_state state = flock.state();

    CHECK(state.medium_velocity == doctest::Approx(10.632).epsilon(0.001));
    CHECK(state.err_velocity == doctest::Approx(3.266).epsilon(0.001));

    CHECK(state.medium_distance == doctest::Approx(3.734).epsilon(0.001));
    CHECK(state.err_distance == doctest::Approx(1.633).epsilon(0.001));
  }

  SUBCASE("case 2") {
    const pr::Vector2 v1{7.5, 8.9};
    const pr::Vector2 v2{10.1, 11.2};
    const pr::Vector2 v3{8.6, 9.8};
    const pr::Vector2 v4{5.3, 8.4};
    const pr::Vector2 v5{11.8, 13.9};
    const pr::Vector2 v6{2.1, 2.2};
    const pr::Vector2 v7{5.8, 6.3};
    const pr::Vector2 v8{4.9, 8.6};
    const pr::Vector2 v9{11.6, 3.5};
    const pr::Vector2 v10{9.1, 3.5};

    const pr::Boid b1{v1, v2, 1};
    const pr::Boid b2{v3, v4, 1};
    const pr::Boid b3{v5, v6, 1};
    const pr::Boid b4{v7, v8, 1};
    const pr::Boid b5{v9, v10, 1};

    pr::Flock flock{10, 2, 0.5, 0.5, 0.5};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);

    pr::Simulation_state state = flock.state();

    CHECK(state.medium_velocity == doctest::Approx(9.541).epsilon(0.001));
    CHECK(state.err_velocity == doctest::Approx(2.732).epsilon(0.001));

    CHECK(state.medium_distance == doctest::Approx(4.280).epsilon(0.001));
    CHECK(state.err_distance == doctest::Approx(0.864).epsilon(0.001));
  }
}
