#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "flock.hpp"

#include "doctest.h"

TEST_CASE("Testing the quadratic_difference() method") {
  SUBCASE("Three values:") {
    const std::vector<float> generic_vector{7.810249676f, 9.219544457f,
                                            14.86606875};
    const float result = pr::quadratic_difference(generic_vector);

    CHECK(result == doctest::Approx(27.88464373).epsilon(0.00000001));
  }

  SUBCASE("Three values") {
    const std::vector<float> generic_vector{2.828427125f, 8.485281374f,
                                            5.656854249f};
    const float result = pr::quadratic_difference(generic_vector);

    CHECK(result == doctest::Approx(16.0).epsilon(0.1));
  }
}

TEST_CASE("Testing the close_boids() method") {
  pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

  const pr::Vector2 v1{50.f, 40.f};
  const pr::Vector2 v2{100.f, 60.f};
  const pr::Vector2 v3{70.f, 90.f};
  const pr::Vector2 v4{130.f, 100.f};

  pr::Boid b1{v1, v1, 500.f};
  pr::Boid b2{v2, v2, 500.f};
  pr::Boid b3{v3, v3, 500.f};
  pr::Boid b4{v4, v4, 500.f};

  b1.set_shape().setFillColor(sf::Color::Red);
  b2.set_shape().setFillColor(sf::Color::Red);
  b3.set_shape().setFillColor(sf::Color::Red);
  b4.set_shape().setFillColor(sf::Color::Red);

  flock.push_back(b1);
  flock.push_back(b2);
  flock.push_back(b3);
  flock.push_back(b4);

  CHECK(flock.close_boids(b1) == doctest::Approx(2.0).epsilon(0.1));
}

TEST_CASE("Testing the find_centermass() method") {
  SUBCASE("Two close boids:") {
    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    const pr::Vector2 v1{50.f, 40.f};
    const pr::Vector2 v2{100.f, 60.f};
    const pr::Vector2 v3{70.f, 90.f};
    const pr::Vector2 v4{140.f, 100.f};

    pr::Boid b1{v1, v1, 200.f};
    pr::Boid b2{v2, v2, 200.f};
    pr::Boid b3{v3, v3, 200.f};
    pr::Boid b4{v4, v4, 200.f};

    b1.set_shape().setFillColor(sf::Color::Red);
    b2.set_shape().setFillColor(sf::Color::Red);
    b3.set_shape().setFillColor(sf::Color::Red);
    b4.set_shape().setFillColor(sf::Color::Red);

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(flock.close_boids(b1) == doctest::Approx(2.0).epsilon(0.1));

    CHECK(center_mass.x_axis() == doctest::Approx(85.0).epsilon(0.1));
    CHECK(center_mass.y_axis() == doctest::Approx(75.0).epsilon(0.1));
  }

  SUBCASE("Three close boids:") {
    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    const pr::Vector2 v1{55.f, 46.f};
    const pr::Vector2 v2{101.f, 68.f};
    const pr::Vector2 v3{78.f, 97.f};
    const pr::Vector2 v4{135.f, 106.f};
    const pr::Vector2 v5{31.f, 52.f};

    pr::Boid b1{v1, v1, 100.f};
    pr::Boid b2{v2, v2, 100.f};
    pr::Boid b3{v3, v3, 100.f};
    pr::Boid b4{v4, v4, 100.f};
    pr::Boid b5{v5, v5, 100.f};

    b1.set_shape().setFillColor(sf::Color::Red);
    b2.set_shape().setFillColor(sf::Color::Red);
    b3.set_shape().setFillColor(sf::Color::Red);
    b4.set_shape().setFillColor(sf::Color::Red);
    b5.set_shape().setFillColor(sf::Color::Red);

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(flock.close_boids(b1) == doctest::Approx(3.0).epsilon(0.1));

    CHECK(center_mass.x_axis() == doctest::Approx(70.00).epsilon(0.01));
    CHECK(center_mass.y_axis() == doctest::Approx(72.33).epsilon(0.01));
  }

  SUBCASE("One close boid:") {
    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    const pr::Vector2 v1{55.f, 46.f};
    const pr::Vector2 v2{76.f, 95.f};
    const pr::Vector2 v3{135.f, 106.f};

    pr::Boid b1{v1, v1, 100.f};
    pr::Boid b2{v2, v2, 100.f};
    pr::Boid b3{v3, v3, 100.f};

    b1.set_shape().setFillColor(sf::Color::Red);
    b2.set_shape().setFillColor(sf::Color::Red);
    b3.set_shape().setFillColor(sf::Color::Red);

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(center_mass.x_axis() == doctest::Approx(76.0).epsilon(0.1));
    CHECK(center_mass.y_axis() == doctest::Approx(95.0).epsilon(0.1));
  }

  SUBCASE("No close boids:") {
    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    const pr::Vector2 v1{50.f, 60.f};
    const pr::Vector2 v2{10000.f, 10000.f};
    const pr::Vector2 v3{30000.f, 30000.f};
    const pr::Vector2 v4{20000.f, 20000.f};

    pr::Boid b1{v1, v1, 100000.f};
    pr::Boid b2{v2, v2, 100000.f};
    pr::Boid b3{v3, v3, 100000.f};
    pr::Boid b4{v4, v4, 100000.f};

    b1.set_shape().setFillColor(sf::Color::Red);
    b2.set_shape().setFillColor(sf::Color::Red);
    b3.set_shape().setFillColor(sf::Color::Red);
    b4.set_shape().setFillColor(sf::Color::Red);

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);

    pr::Vector2 center_mass = flock.find_centermass(b1);

    CHECK(center_mass.x_axis() == doctest::Approx(50.0).epsilon(0.1));
    CHECK(center_mass.y_axis() == doctest::Approx(60.0).epsilon(0.1));
  }
}

TEST_CASE("Testing the is_predator() method") {
  const pr::Vector2 v1{2.f, 3.f};
  const pr::Vector2 v2{4.f, 5.f};
  const pr::Vector2 v3{5.f, 6.f};
  const pr::Vector2 v4{1000.f, 1000.f};
  const pr::Vector2 v5{1.f, 1.f};

  pr::Boid b1{v1, v5, 10000.f};
  pr::Boid b2{v2, v5, 10000.f};
  pr::Boid b3{v3, v5, 10000.f};
  pr::Boid b4{v4, v5, 10000.f};

  b1.set_shape().setFillColor(sf::Color::Red);
  b2.set_shape().setFillColor(sf::Color::Red);
  b3.set_shape().setFillColor(sf::Color::Black);
  b4.set_shape().setFillColor(sf::Color::Black);

  pr::Flock flock{100, 30, 0.05, 0.5, 0.0005};

  flock.push_back(b1);
  flock.push_back(b2);
  flock.push_back(b3);
  flock.push_back(b4);

  CHECK(flock.is_predator(b1) == true);
  CHECK(flock.is_predator(b2) == true);
  CHECK(flock.is_predator(b3) == false);
  CHECK(flock.is_predator(b4) == false);
}

TEST_CASE("Testing the evolve() method") {
  SUBCASE("Five boids, two are close to the chosen one:") {
    const pr::Vector2 v1{15.8f, 500.f};
    const pr::Vector2 v2{100.f, 50.f};
    const pr::Vector2 v3{20.f, 450.f};
    const pr::Vector2 v4{200.f, 20.f};
    const pr::Vector2 v5{1000.f, 2000.f};
    const pr::Vector2 v6{10.f, 10.f};
    const pr::Vector2 v7{1200.f, 3000.f};
    const pr::Vector2 v8{10.f, 10.f};
    const pr::Vector2 v9{16.5f, 510.f};
    const pr::Vector2 v10{50.f, 80.f};

    pr::Boid b1{v1, v2, 1.f};
    pr::Boid b2{v3, v4, 10000.f};
    pr::Boid b3{v5, v6, 10000.f};
    pr::Boid b4{v7, v8, 10000.f};
    pr::Boid b5{v9, v10, 10000.f};

    b1.set_shape().setFillColor(sf::Color::Red);
    b2.set_shape().setFillColor(sf::Color::Red);
    b3.set_shape().setFillColor(sf::Color::Red);
    b4.set_shape().setFillColor(sf::Color::Red);
    b5.set_shape().setFillColor(sf::Color::Red);

    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);

    CHECK(flock.close_boids(b1) == doctest::Approx(2.0).epsilon(0.1));

    CHECK(b1.separation(b2, 0.05f, 30.f).x_axis() ==
          doctest::Approx(0.0).epsilon(0.1));
    CHECK(b1.separation(b2, 0.05f, 30.f).y_axis() ==
          doctest::Approx(0.0).epsilon(0.1));
    CHECK(b1.separation(b5, 0.05f, 30.f).x_axis() ==
          doctest::Approx(-0.035).epsilon(0.001));
    CHECK(b1.separation(b5, 0.05f, 30.f).y_axis() ==
          doctest::Approx(-0.5).epsilon(0.1));

    CHECK(b1.allignment(b2, 0.5f, flock.close_boids(b1), 100.f).x_axis() ==
          doctest::Approx(25.0).epsilon(0.1));
    CHECK(b1.allignment(b2, 0.5f, flock.close_boids(b1), 100.f).y_axis() ==
          doctest::Approx(-7.5).epsilon(0.1));
    CHECK(b1.allignment(b5, 0.5f, flock.close_boids(b1), 100.f).x_axis() ==
          doctest::Approx(-12.5).epsilon(0.1));
    CHECK(b1.allignment(b5, 0.5f, flock.close_boids(b1), 100.f).y_axis() ==
          doctest::Approx(7.5).epsilon(0.1));

    CHECK(flock.find_centermass(b1).x_axis() ==
          doctest::Approx(18.25).epsilon(0.01));
    CHECK(flock.find_centermass(b1).y_axis() ==
          doctest::Approx(480.0).epsilon(0.1));

    CHECK(b1.cohesion(flock.find_centermass(b1), 0.0005f).x_axis() ==
          doctest::Approx(0.001225).epsilon(0.000001));
    CHECK(b1.cohesion(flock.find_centermass(b1), 0.0005f).y_axis() ==
          doctest::Approx(-0.01).epsilon(0.01));

    CHECK(flock.find_separation(b1).x_axis() ==
          doctest::Approx(-0.035).epsilon(0.01));
    CHECK(flock.find_separation(b1).y_axis() ==
          doctest::Approx(-0.5).epsilon(0.1));

    CHECK(flock.find_allignment(b1).x_axis() ==
          doctest::Approx(12.5).epsilon(0.1));
    CHECK(flock.find_allignment(b1).y_axis() ==
          doctest::Approx(0.0).epsilon(0.1));

    CHECK(flock.find_cohesion(b1).x_axis() ==
          doctest::Approx(12.466225).epsilon(0.000001));
    CHECK(flock.find_cohesion(b1).y_axis() ==
          doctest::Approx(-0.51).epsilon(0.01));

    pr::Vector2 position_offset = flock.evolve(b1, 0.5f);

    CHECK(b1.velocity().x_axis() ==
          doctest::Approx(112.466225).epsilon(0.000001));
    CHECK(b1.velocity().y_axis() == doctest::Approx(49.49).epsilon(0.01));

    CHECK(position_offset.x_axis() ==
          doctest::Approx(56.2331125).epsilon(0.0000001));
    CHECK(position_offset.y_axis() == doctest::Approx(24.745).epsilon(0.001));

    CHECK(b1.position().x_axis() ==
          doctest::Approx(72.0331125).epsilon(0.0000001));
    CHECK(b1.position().y_axis() == doctest::Approx(524.745).epsilon(0.001));
  }

  SUBCASE("Seven boids, four are close to the chosen one:") {
    const pr::Vector2 v1{506.5f, 709.5f};
    const pr::Vector2 v2{125.f, 167.f};
    const pr::Vector2 v3{605.5f, 699.f};
    const pr::Vector2 v4{137.f, 185.f};
    const pr::Vector2 v5{555.5, 654.5};
    const pr::Vector2 v6{198.f, 202.f};
    const pr::Vector2 v7{1000.f, 2000.f};
    const pr::Vector2 v8{139.f, 157.f};
    const pr::Vector2 v9{3000.f, 4000.f};
    const pr::Vector2 v10{168.f, 179.f};
    const pr::Vector2 v11{515.5f, 700.f};
    const pr::Vector2 v12{515.f, 535.f};
    const pr::Vector2 v13{500.5f, 699.9f};
    const pr::Vector2 v14{378.f, 389.f};

    pr::Boid b1{v1, v2, 1.f};
    pr::Boid b2{v3, v4, 1000.f};
    pr::Boid b3{v5, v6, 1000.f};
    pr::Boid b4{v7, v8, 1000.f};
    pr::Boid b5{v9, v10, 1000.f};
    pr::Boid b6{v11, v12, 1000.f};
    pr::Boid b7{v13, v14, 1000.f};

    b1.set_shape().setFillColor(sf::Color::Red);
    b2.set_shape().setFillColor(sf::Color::Red);
    b3.set_shape().setFillColor(sf::Color::Red);
    b4.set_shape().setFillColor(sf::Color::Red);
    b5.set_shape().setFillColor(sf::Color::Red);
    b6.set_shape().setFillColor(sf::Color::Red);
    b7.set_shape().setFillColor(sf::Color::Red);

    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);
    flock.push_back(b6);
    flock.push_back(b7);

    CHECK(flock.close_boids(b1) == doctest::Approx(4.0).epsilon(0.1));

    CHECK(b1.separation(b2, 0.05f, 30.f).x_axis() ==
          doctest::Approx(0.0).epsilon(0.1));
    CHECK(b1.separation(b2, 0.05f, 30.f).y_axis() ==
          doctest::Approx(0.0).epsilon(0.1));
    CHECK(b1.separation(b3, 0.05f, 30.f).x_axis() ==
          doctest::Approx(0.0).epsilon(0.1));
    CHECK(b1.separation(b3, 0.05f, 30.f).y_axis() ==
          doctest::Approx(0.0).epsilon(0.1));
    CHECK(b1.separation(b6, 0.05f, 30.f).x_axis() ==
          doctest::Approx(-0.45).epsilon(0.01));
    CHECK(b1.separation(b6, 0.05f, 30.f).y_axis() ==
          doctest::Approx(0.475).epsilon(0.001));
    CHECK(b1.separation(b7, 0.05f, 30.f).x_axis() ==
          doctest::Approx(0.3).epsilon(0.1));
    CHECK(b1.separation(b7, 0.05f, 30.f).y_axis() ==
          doctest::Approx(0.48).epsilon(0.01));

    CHECK(b1.allignment(b2, 0.5f, flock.close_boids(b1), 100.f).x_axis() ==
          doctest::Approx(1.5).epsilon(0.1));
    CHECK(b1.allignment(b2, 0.5f, flock.close_boids(b1), 100.f).y_axis() ==
          doctest::Approx(2.25).epsilon(0.01));
    CHECK(b1.allignment(b3, 0.5f, flock.close_boids(b1), 100.f).x_axis() ==
          doctest::Approx(9.125).epsilon(0.001));
    CHECK(b1.allignment(b3, 0.5f, flock.close_boids(b1), 100.f).y_axis() ==
          doctest::Approx(4.375).epsilon(0.001));
    CHECK(b1.allignment(b6, 0.5f, flock.close_boids(b1), 100.f).x_axis() ==
          doctest::Approx(48.75).epsilon(0.01));
    CHECK(b1.allignment(b6, 0.5f, flock.close_boids(b1), 100.f).y_axis() ==
          doctest::Approx(46.0).epsilon(0.1));
    CHECK(b1.allignment(b7, 0.5f, flock.close_boids(b1), 100.f).x_axis() ==
          doctest::Approx(31.625).epsilon(0.001));
    CHECK(b1.allignment(b7, 0.5f, flock.close_boids(b1), 100.f).y_axis() ==
          doctest::Approx(27.75).epsilon(0.01));

    CHECK(flock.find_centermass(b1).x_axis() ==
          doctest::Approx(544.25).epsilon(0.01));
    CHECK(flock.find_centermass(b1).y_axis() ==
          doctest::Approx(688.35).epsilon(0.01));

    CHECK(b1.cohesion(flock.find_centermass(b1), 0.0005f).x_axis() ==
          doctest::Approx(0.018875).epsilon(0.000001));
    CHECK(b1.cohesion(flock.find_centermass(b1), 0.0005f).y_axis() ==
          doctest::Approx(-0.010575).epsilon(0.00001));

    CHECK(flock.find_separation(b1).x_axis() ==
          doctest::Approx(-0.15).epsilon(0.01));
    CHECK(flock.find_separation(b1).y_axis() ==
          doctest::Approx(0.955).epsilon(0.001));

    CHECK(flock.find_allignment(b1).x_axis() ==
          doctest::Approx(91.0).epsilon(0.1));
    CHECK(flock.find_allignment(b1).y_axis() ==
          doctest::Approx(80.375).epsilon(0.001));

    CHECK(flock.find_cohesion(b1).x_axis() ==
          doctest::Approx(90.868875).epsilon(0.000001));
    CHECK(flock.find_cohesion(b1).y_axis() ==
          doctest::Approx(81.319425).epsilon(0.000001));

    pr::Vector2 position_offset = flock.evolve(b1, 0.05f);

    CHECK(b1.velocity().x_axis() ==
          doctest::Approx(215.868875).epsilon(0.000001));
    CHECK(b1.velocity().y_axis() ==
          doctest::Approx(248.319425).epsilon(0.000001));

    CHECK(position_offset.x_axis() ==
          doctest::Approx(10.79344375).epsilon(0.00000001));
    CHECK(position_offset.y_axis() ==
          doctest::Approx(12.4159713).epsilon(0.0000001));

    CHECK(b1.position().x_axis() ==
          doctest::Approx(517.2934438).epsilon(0.0000001));
    CHECK(b1.position().y_axis() ==
          doctest::Approx(721.9159713).epsilon(0.0000001));
  }

  SUBCASE("Four boids, none of them is close to the chosen one") {
    const pr::Vector2 v1{1.f, 2.f};
    const pr::Vector2 v2{1.f, 1.f};
    const pr::Vector2 v3{300.f, 400.f};
    const pr::Vector2 v4{1.f, 1.f};
    const pr::Vector2 v5{500.f, 600.f};
    const pr::Vector2 v6{1.f, 1.f};
    const pr::Vector2 v7{700.f, 800.f};
    const pr::Vector2 v8{1.f, 1.f};

    pr::Boid b1{v1, v2, 1.f};
    pr::Boid b2{v3, v4, 1000.f};
    pr::Boid b3{v5, v6, 1000.f};
    pr::Boid b4{v7, v8, 1000.f};

    b1.set_shape().setFillColor(sf::Color::Red);
    b2.set_shape().setFillColor(sf::Color::Red);
    b3.set_shape().setFillColor(sf::Color::Red);
    b4.set_shape().setFillColor(sf::Color::Red);

    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);

    CHECK(flock.close_boids(b1) == doctest::Approx(0.0).epsilon(0.1));

    pr::Vector2 position_offset = flock.evolve(b1, 0.5f);

    CHECK(b1.velocity().x_axis() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(b1.velocity().y_axis() == doctest::Approx(1.0).epsilon(0.1));

    CHECK(position_offset.x_axis() == doctest::Approx(0.5).epsilon(0.1));
    CHECK(position_offset.y_axis() == doctest::Approx(0.5).epsilon(0.1));

    CHECK(b1.position().x_axis() == doctest::Approx(1.5).epsilon(0.1));
    CHECK(b1.position().y_axis() == doctest::Approx(2.5).epsilon(0.1));
  }
}

TEST_CASE("Testing the state() function") {
  SUBCASE("Three boids:") {
    const pr::Vector2 v1{2.f, 3.f};
    const pr::Vector2 v2{5.f, 6.f};
    const pr::Vector2 v3{4.f, 5.f};
    const pr::Vector2 v4{6.f, 7.f};
    const pr::Vector2 v5{8.f, 9.f};
    const pr::Vector2 v6{10.f, 11.f};

    const pr::Boid b1{v1, v2, 1000.f};
    const pr::Boid b2{v3, v4, 1000.f};
    const pr::Boid b3{v5, v6, 1000.f};

    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);

    std::vector<float> velocities = flock.extract_velocities();
    std::vector<float> distances = flock.extract_distances();

    pr::Simulation_state state = flock.state();

    CHECK(velocities.size() == 3);
    CHECK(velocities[0] == doctest::Approx(7.8102497).epsilon(0.0000001));
    CHECK(velocities[1] == doctest::Approx(9.2195445).epsilon(0.0000001));
    CHECK(velocities[2] == doctest::Approx(14.866069).epsilon(0.000001));

    CHECK(distances.size() == 3);
    CHECK(distances[0] == doctest::Approx(2.8284271).epsilon(0.0000001));
    CHECK(distances[1] == doctest::Approx(8.4852814).epsilon(0.0000001));
    CHECK(distances[2] == doctest::Approx(5.6568542).epsilon(0.0000001));

    CHECK(state.medium_velocity ==
          doctest::Approx(10.6319544).epsilon(0.0000001));
    CHECK(state.err_velocity == doctest::Approx(2.1557924).epsilon(0.0000001));

    CHECK(state.medium_distance ==
          doctest::Approx(5.6568542).epsilon(0.0000001));
    CHECK(state.err_distance == doctest::Approx(1.6329931).epsilon(0.0000001));
  }

  SUBCASE("Five boids:") {
    const pr::Vector2 v1{7.5f, 8.9f};
    const pr::Vector2 v2{10.1f, 11.2f};
    const pr::Vector2 v3{8.6f, 9.8f};
    const pr::Vector2 v4{5.3f, 8.4f};
    const pr::Vector2 v5{11.8f, 13.9f};
    const pr::Vector2 v6{2.1f, 2.2f};
    const pr::Vector2 v7{5.8f, 6.3f};
    const pr::Vector2 v8{4.9f, 8.6f};
    const pr::Vector2 v9{11.6f, 3.5f};
    const pr::Vector2 v10{9.1f, 3.5f};

    const pr::Boid b1{v1, v2, 1000.f};
    const pr::Boid b2{v3, v4, 1000.f};
    const pr::Boid b3{v5, v6, 1000.f};
    const pr::Boid b4{v7, v8, 1000.f};
    const pr::Boid b5{v9, v10, 1000.f};

    pr::Flock flock{100.f, 30.f, 0.05f, 0.5f, 0.0005f};

    flock.push_back(b1);
    flock.push_back(b2);
    flock.push_back(b3);
    flock.push_back(b4);
    flock.push_back(b5);

    const std::vector<float> velocities = flock.extract_velocities();
    const std::vector<float> distances = flock.extract_distances();

    pr::Simulation_state state = flock.state();

    CHECK(velocities.size() == 5);
    CHECK(velocities[0] == doctest::Approx(15.081446).epsilon(0.000001));
    CHECK(velocities[1] == doctest::Approx(9.9322706).epsilon(0.0000001));
    CHECK(velocities[2] == doctest::Approx(3.0413813).epsilon(0.0000001));
    CHECK(velocities[3] == doctest::Approx(9.8979796).epsilon(0.0000001));
    CHECK(velocities[4] == doctest::Approx(9.7498718).epsilon(0.0000001));

    CHECK(distances.size() == 10);
    CHECK(distances[0] == doctest::Approx(1.421267).epsilon(0.000001));
    CHECK(distances[1] == doctest::Approx(6.5946948).epsilon(0.0000001));
    CHECK(distances[2] == doctest::Approx(3.106445).epsilon(0.000001));
    CHECK(distances[3] == doctest::Approx(6.7801180).epsilon(0.0000001));
    CHECK(distances[4] == doctest::Approx(5.2009615).epsilon(0.0000001));
    CHECK(distances[5] == doctest::Approx(4.4821870).epsilon(0.0000001));
    CHECK(distances[6] == doctest::Approx(6.9778220).epsilon(0.0000001));
    CHECK(distances[7] == doctest::Approx(9.6829748).epsilon(0.0000001));
    CHECK(distances[8] == doctest::Approx(10.401923).epsilon(0.000001));
    CHECK(distances[9] == doctest::Approx(6.4404969).epsilon(0.0000001));

    CHECK(state.medium_velocity ==
          doctest::Approx(9.5405899).epsilon(0.0000001));
    CHECK(pr::quadratic_difference(velocities) ==
          doctest::Approx(73.265739).epsilon(0.000001));
    CHECK(state.err_velocity == doctest::Approx(1.9139715).epsilon(0.0000001));

    CHECK(state.medium_distance == doctest::Approx(6.108889).epsilon(0.000001));
    CHECK(pr::quadratic_difference(distances) ==
          doctest::Approx(67.214754).epsilon(0.000001));
    CHECK(state.err_distance == doctest::Approx(0.8641936).epsilon(0.0000001));
  }
}
