#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boid.hpp"

#include "doctest.h"

TEST_CASE("Testing the separation() method") {
  SUBCASE("Positive separation_parameter and positive components:") {
    const pr::Vector2 x_i{1.f, 2.f};
    const pr::Vector2 v_i{1.f, 1.f};

    const pr::Boid B_i{x_i, v_i, 3.f};

    const pr::Vector2 x_j{3.f, 4.f};
    const pr::Vector2 v_j{1.f, 1.f};

    const pr::Boid B_j{x_j, v_j, 6.f};

    const float s = 1.f;
    const float ds = 5.f;

    pr::Vector2 v1 = B_i.separation(B_j, s, ds);

    CHECK(v1.x_axis() == doctest::Approx(-2.0).epsilon(0.1));
    CHECK(v1.y_axis() == doctest::Approx(-2.0).epsilon(0.1));
  }

  SUBCASE("Positive separation_parameter and positive components:") {
    const pr::Vector2 x_i{3.5f, 8.1f};
    const pr::Vector2 v_i{1.f, 1.f};

    const pr::Boid B_i{x_i, v_i, 20.f};

    const pr::Vector2 x_j{6.78f, 4.4f};
    const pr::Vector2 v_j{1.f, 1.f};

    const pr::Boid B_j{x_j, v_j, 20.f};

    const float s = 3.f;
    const float ds = 5.f;

    pr::Vector2 v1 = B_i.separation(B_j, s, ds);

    CHECK(v1.x_axis() == doctest::Approx(-9.84).epsilon(0.01));
    CHECK(v1.y_axis() == doctest::Approx(11.10).epsilon(0.01));
  }

  SUBCASE(
      "Positive separation_parameter and positive and negative components:") {
    const pr::Vector2 x_i{8.3f, 55.82f};
    const pr::Vector2 v_i{1.f, 1.f};

    const pr::Boid B_i{x_i, v_i, 100.f};

    const pr::Vector2 x_j{-7.98f, 22.55f};
    const pr::Vector2 v_j{1.f, 1.f};

    const pr::Boid B_j{x_j, v_j, 100.f};

    const float s = 7.90f;
    const float ds = 40.f;

    pr::Vector2 v1 = B_i.separation(B_j, s, ds);

    CHECK(v1.x_axis() == doctest::Approx(128.612).epsilon(0.001));
    CHECK(v1.y_axis() == doctest::Approx(262.833).epsilon(0.001));
  }
}

TEST_CASE("testing the allignment() method") {
  SUBCASE("Positive allignment_parameter and positive components:") {
    const pr::Vector2 x_i{1.f, 1.f};
    const pr::Vector2 v_i{3.f, 5.f};

    const pr::Boid B_i{x_i, v_i, 20.f};

    const pr::Vector2 x_j{2.f, 1.f};
    const pr::Vector2 v_j{4.f, 7.f};

    const pr::Boid B_j{x_j, v_j, 20.f};

    const float a = 0.5f;
    const float n = 9.f;

    pr::Vector2 v2 = B_i.allignment(B_j, a, n, 100.f);

    CHECK(v2.x_axis() == doctest::Approx(0.0555).epsilon(0.0001));
    CHECK(v2.y_axis() == doctest::Approx(0.1111).epsilon(0.0001));
  }

  SUBCASE("Positive allignment_parameter and negative components:") {
    const pr::Vector2 x_i{1.f, 1.f};
    const pr::Vector2 v_i{-1.f, -9.f};

    const pr::Boid B_i{x_i, v_i, 20.f};

    const pr::Vector2 x_j{2.f, 1.f};
    const pr::Vector2 v_j{-12.f, -7.f};

    const pr::Boid B_j{x_j, v_j, 30.f};

    const float a = 0.7f;
    const float n = 19.f;

    pr::Vector2 v2 = B_i.allignment(B_j, a, n, 100.f);

    CHECK(v2.x_axis() == doctest::Approx(-0.4052).epsilon(0.0001));
    CHECK(v2.y_axis() == doctest::Approx(0.0736).epsilon(0.0001));
  }

  SUBCASE(
      "Positive allignment_parameter and positive and negative components:") {
    const pr::Vector2 x_i{1.f, 1.f};
    const pr::Vector2 v_i{1.01f, -7.85f};

    const pr::Boid B_i{x_i, v_i, 20.f};

    const pr::Vector2 x_j{2.f, 1.f};
    const pr::Vector2 v_j{-8.98f, 15.1f};

    const pr::Boid B_j{x_j, v_j, 30.f};

    const float a = 0.2f;
    const float n = 49.f;

    pr::Vector2 v2 = B_i.allignment(B_j, a, n, 100.f);

    CHECK(v2.x_axis() == doctest::Approx(-0.0407).epsilon(0.0001));
    CHECK(v2.y_axis() == doctest::Approx(0.0936).epsilon(0.0001));
  }
}

TEST_CASE("testing the cohesion() method") {
  SUBCASE("Positive cohesion_parameter and positive components:") {
    const pr::Vector2 x_cm{3.5f, 8.9f};

    const float c = 3.f;

    const pr::Vector2 x_i{2.88f, 6.2f};
    const pr::Vector2 v_i{1.f, 1.f};

    const pr::Boid B_i{x_i, v_i, 20.f};

    pr::Vector2 v3 = B_i.cohesion(x_cm, c);

    CHECK(v3.x_axis() == doctest::Approx(1.86).epsilon(0.01));
    CHECK(v3.y_axis() == doctest::Approx(8.1).epsilon(0.1));
  }

  SUBCASE("Positive cohesion_parameter and positive components:") {
    const pr::Vector2 x_cm{2.f, 2.f};

    const float c = 1.f;

    const pr::Vector2 x_i{1.f, 1.f};
    const pr::Vector2 v_i{1.f, 1.f};

    const pr::Boid B_i{x_i, v_i, 5.f};

    pr::Vector2 v3 = B_i.cohesion(x_cm, c);

    CHECK(v3.x_axis() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(1.0).epsilon(0.1));
  }

  SUBCASE("Positive cohesion_parameter and positive components:") {
    const pr::Vector2 x_cm{-3.5f, 55.9f};

    const float c = 1.3f;

    const pr::Vector2 x_i{5.87f, -18.2f};
    const pr::Vector2 v_i{1.f, 1.f};

    const pr::Boid B_i{x_i, v_i, 50.f};

    pr::Vector2 v3 = B_i.cohesion(x_cm, c);

    CHECK(v3.x_axis() == doctest::Approx(-12.181).epsilon(0.001));
    CHECK(v3.y_axis() == doctest::Approx(96.33).epsilon(0.01));
  }
}

TEST_CASE("Testing the get_angle() function") {
  SUBCASE("Positive components of velocity:") {
    const pr::Vector2 v1{1.f, 1.f};
    const pr::Vector2 v2{2.f, 2.f};

    const pr::Boid b1{v1, v2, 5.f};

    CHECK(b1.get_angle() == doctest::Approx(135.0).epsilon(0.1));
  }

  SUBCASE("Negative and positive components of velocity:") {
    const pr::Vector2 v1{1.f, 1.f};
    const pr::Vector2 v2{-2.f, 2.f};

    const pr::Boid b1{v1, v2, 5.f};

    CHECK(b1.get_angle() == doctest::Approx(225.0).epsilon(0.1));
  }

  SUBCASE("Negative components of velocity:") {
    const pr::Vector2 v1{1.f, 1.f};
    const pr::Vector2 v2{-2.f, -2.f};

    const pr::Boid b1{v1, v2, 5.f};

    CHECK(b1.get_angle() == doctest::Approx(315.0).epsilon(0.1));
  }

  SUBCASE("Positive and negative components of velocity:") {
    const pr::Vector2 v1{1.f, 1.f};
    const pr::Vector2 v2{2.f, -2.f};

    const pr::Boid b1{v1, v2, 5.f};

    CHECK(b1.get_angle() == doctest::Approx(45.0).epsilon(0.1));
  }
}

TEST_CASE("Testing the limit_velocity() method") {
  pr::Vector2 v1{600.f, 0.f};
  pr::Vector2 v2{-600.f, 0.f};
  pr::Vector2 v3{0.f, 600.f};
  pr::Vector2 v4{0.f, -600.f};
  pr::Vector2 v5{3.f, 4.f};
  pr::Vector2 v6{-3.f, 4.f};
  pr::Vector2 v7{3.f, 4.f};

  pr::Boid b1{v1, v1, 500.f};
  pr::Boid b2{v2, v2, 500.f};
  pr::Boid b3{v3, v3, 500.f};
  pr::Boid b4{v4, v4, 500.f};
  pr::Boid b5{v5, v5, 4.f};
  pr::Boid b6{v6, v6, 4.f};
  pr::Boid b7{v7, v7, 6.f};

  b1.limit_velocity();
  b2.limit_velocity();
  b3.limit_velocity();
  b4.limit_velocity();
  b5.limit_velocity();
  b6.limit_velocity();
  b7.limit_velocity();

  CHECK(b1.velocity().x_axis() == doctest::Approx(300.0).epsilon(0.1));
  CHECK(b1.velocity().y_axis() == doctest::Approx(0.0).epsilon(0.1));
  CHECK(b2.velocity().x_axis() == doctest::Approx(-300.0).epsilon(0.1));
  CHECK(b2.velocity().y_axis() == doctest::Approx(0.0).epsilon(0.1));
  CHECK(b3.velocity().x_axis() == doctest::Approx(0.0).epsilon(0.1));
  CHECK(b3.velocity().y_axis() == doctest::Approx(300.0).epsilon(0.1));
  CHECK(b4.velocity().x_axis() == doctest::Approx(0.0).epsilon(0.1));
  CHECK(b4.velocity().y_axis() == doctest::Approx(-300.0).epsilon(0.1));
  CHECK(b5.velocity().x_axis() == doctest::Approx(1.5).epsilon(0.1));
  CHECK(b5.velocity().y_axis() == doctest::Approx(2.0).epsilon(0.1));
  CHECK(b6.velocity().x_axis() == doctest::Approx(-1.5).epsilon(0.1));
  CHECK(b6.velocity().y_axis() == doctest::Approx(2.0).epsilon(0.1));
  CHECK(b7.velocity().x_axis() == doctest::Approx(3.0).epsilon(0.1));
  CHECK(b7.velocity().y_axis() == doctest::Approx(4.0).epsilon(0.1));
}

TEST_CASE("Testing the change_velocity() method") {
  SUBCASE("Summed velocity with positive components_") {
    const pr::Vector2 v1{5.5f, 5.5f};
    const pr::Vector2 v2{5.f, 5.f};

    pr::Boid b1{v1, v1, 20.f};

    b1.change_velocity(v2);

    CHECK(b1.velocity().x_axis() == doctest::Approx(10.5).epsilon(0.1));
    CHECK(b1.velocity().y_axis() == doctest::Approx(10.5).epsilon(0.1));
  }

  SUBCASE("Summed velocity with negative components:") {
    const pr::Vector2 v1{5.5f, 5.5f};
    const pr::Vector2 v2{-5.f, -5.f};

    pr::Boid b1{v1, v1, 20.f};

    b1.change_velocity(v2);

    CHECK(b1.velocity().x_axis() == doctest::Approx(0.5).epsilon(0.1));
    CHECK(b1.velocity().y_axis() == doctest::Approx(0.5).epsilon(0.1));
  }
}

TEST_CASE("Testing the change_position() method") {
  SUBCASE("Summed position with positive components:") {
    const pr::Vector2 v1{5.5f, 5.5f};
    const pr::Vector2 v2{5.f, 5.f};

    pr::Boid b1{v1, v1, 20.f};

    b1.change_position(v2);

    CHECK(b1.position().x_axis() == doctest::Approx(10.5).epsilon(0.1));
    CHECK(b1.position().y_axis() == doctest::Approx(10.5).epsilon(0.1));
  }

  SUBCASE("Summed position with negative components:") {
    const pr::Vector2 v1{5.5f, 5.5f};
    const pr::Vector2 v2{-5.f, -5.f};

    pr::Boid b1{v1, v1, 20.f};

    b1.change_position(v2);

    CHECK(b1.position().x_axis() == doctest::Approx(0.5).epsilon(0.1));
    CHECK(b1.position().y_axis() == doctest::Approx(0.5).epsilon(0.1));
  }
}

TEST_CASE("Testing the == operator") {
  SUBCASE("Same boid:") {
    const pr::Vector2 v1{1.f, 1.f};
    const pr::Vector2 v2{2.f, 2.f};

    const pr::Boid b1{v1, v2, 15.f};
    const pr::Boid b2{v1, v2, 15.f};

    bool same = (b1 == b2);

    CHECK(same == true);
  }

  SUBCASE("Different boid:") {
    const pr::Vector2 v1{1.f, 1.f};
    const pr::Vector2 v2{2.f, 2.f};

    const pr::Boid b1{v1, v2, 5.f};
    const pr::Boid b2{v2, v1, 5.f};

    bool same = (b1 == b2);

    CHECK(same == false);
  }
}