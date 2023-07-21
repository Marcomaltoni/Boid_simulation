#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boid.hpp"

#include "doctest.h"

TEST_CASE("Testing the get_angle() function") {
  SUBCASE("positive components of velocity") {
    const pr::Vector2 v1{1, 1};
    const pr::Vector2 v2{2, 2};

    const pr::Boid b1{v1, v2, 1};

    CHECK(b1.get_angle() == 135);
  }

  SUBCASE("negative and positive components of velocity") {
    const pr::Vector2 v1{1, 1};
    const pr::Vector2 v2{-2, 2};

    const pr::Boid b1{v1, v2, 1};

    CHECK(b1.get_angle() == 225);
  }

  SUBCASE("negative components of velocity") {
    const pr::Vector2 v1{1, 1};
    const pr::Vector2 v2{-2, -2};

    const pr::Boid b1{v1, v2, 1};

    CHECK(b1.get_angle() == 315);
  }

  SUBCASE("positive and negative components of velocity") {
    const pr::Vector2 v1{1, 1};
    const pr::Vector2 v2{2, -2};

    const pr::Boid b1{v1, v2, 1};

    CHECK(b1.get_angle() == 45);
  }
}

TEST_CASE("testing the limit_velocity() function") {
  pr::Vector2 v1{600, 0};
  pr::Vector2 v2{-600, 0};
  pr::Vector2 v3{0, 600};
  pr::Vector2 v4{0, -600};
  pr::Vector2 v5{3, 4};
  pr::Vector2 v6{-3, 4};
  pr::Vector2 v7{3, 4};

  pr::Boid b1{v1, v1, 500};
  pr::Boid b2{v2, v2, 500};
  pr::Boid b3{v3, v3, 500};
  pr::Boid b4{v4, v4, 500};
  pr::Boid b5{v5, v5, 4};
  pr::Boid b6{v6, v6, 4};
  pr::Boid b7{v7, v7, 6};

  b1.limit_velocity();
  b2.limit_velocity();
  b3.limit_velocity();
  b4.limit_velocity();
  b5.limit_velocity();
  b6.limit_velocity();
  b7.limit_velocity();

  CHECK(b1.velocity().x_axis() == 300);
  CHECK(b1.velocity().y_axis() == 0);
  CHECK(b2.velocity().x_axis() == -300);
  CHECK(b2.velocity().y_axis() == 0);
  CHECK(b3.velocity().x_axis() == 0);
  CHECK(b3.velocity().y_axis() == 300);
  CHECK(b4.velocity().x_axis() == 0);
  CHECK(b4.velocity().y_axis() == -300);
  CHECK(b5.velocity().x_axis() == doctest::Approx(1.500).epsilon(0.001));
  CHECK(b5.velocity().y_axis() == 2);
  CHECK(b6.velocity().x_axis() == doctest::Approx(-1.500).epsilon(0.001));
  CHECK(b6.velocity().y_axis() == 2);
  CHECK(b7.velocity().x_axis() == 3);
  CHECK(b7.velocity().y_axis() == 4);
}

TEST_CASE("testing the separation() method") {
  SUBCASE("s intero e positivo e ingressi interi e positivi") {
    const pr::Vector2 xi{1, 2};
    const pr::Vector2 vi{1, 1};

    const pr::Boid B_i{xi, vi, 1};

    const pr::Vector2 xj{3, 4};
    const pr::Vector2 vj{1, 1};

    const pr::Boid B_j{xj, vj, 1};

    float s = 1;
    float ds = 5;

    pr::Vector2 v1 = B_i.separation(B_j, s, ds);

    CHECK(v1.x_axis() == -2);
    CHECK(v1.y_axis() == -2);
  }

  SUBCASE("s intero e negativo e ingressi float e positivi") {
    const pr::Vector2 xi{3.5, 8.1};
    const pr::Vector2 vi{1, 1};

    const pr::Boid B_i{xi, vi, 1};

    const pr::Vector2 xj{6.78, 4.4};
    const pr::Vector2 vj{1, 1};

    const pr::Boid B_j{xj, vj, 1};

    float s = 3;
    float ds = 5;

    pr::Vector2 v1 = B_i.separation(B_j, s, ds);

    CHECK(v1.x_axis() == doctest::Approx(-9.8400).epsilon(0.0001));
    CHECK(v1.y_axis() == doctest::Approx(11.1000).epsilon(0.0001));
  }

  SUBCASE("s float e ingressi float") {
    const pr::Vector2 xi{8.3, 55.82};
    const pr::Vector2 vi{1, 1};

    const pr::Boid B_i{xi, vi, 1};

    const pr::Vector2 xj{-7.98, 22.55};
    const pr::Vector2 vj{1, 1};

    const pr::Boid B_j{xj, vj, 1};

    float s = 7.90;
    float ds = 40;

    pr::Vector2 v1 = B_i.separation(B_j, s, ds);

    CHECK(v1.x_axis() == doctest::Approx(128.6120).epsilon(0.0001));
    CHECK(v1.y_axis() == doctest::Approx(262.8330).epsilon(0.0001));
  }
}

TEST_CASE("testing the allignment() method") {
  SUBCASE("ingressi interi positivi") {
    const pr::Vector2 xi{1, 1};
    const pr::Vector2 vi{3, 5};

    const pr::Boid B_i{xi, vi, 1};

    const pr::Vector2 xj{2, 1};
    const pr::Vector2 vj{4, 7};

    const pr::Boid B_j{xj, vj, 1};

    float a = 0.5;
    float n = 9;

    pr::Vector2 v2 = B_i.allignment(B_j, a, n, 100);

    CHECK(v2.x_axis() == doctest::Approx(0.0555).epsilon(0.0001));
    CHECK(v2.y_axis() == doctest::Approx(0.1111).epsilon(0.0001));
  }

  SUBCASE("ingressi interi negativi") {
    const pr::Vector2 xi{1, 1};
    const pr::Vector2 vi{-1, -9};

    const pr::Boid B_i{xi, vi, 1};

    const pr::Vector2 xj{2, 1};
    const pr::Vector2 vj{-12, -7};

    const pr::Boid B_j{xj, vj, 1};

    float a = 0.7;
    float n = 19;

    pr::Vector2 v2 = B_i.allignment(B_j, a, n, 100);

    CHECK(v2.x_axis() == doctest::Approx(-0.4052).epsilon(0.0001));
    CHECK(v2.y_axis() == doctest::Approx(0.0736).epsilon(0.0001));
  }

  SUBCASE("ingressi float") {
    const pr::Vector2 xi{1, 1};
    const pr::Vector2 vi{1.01, -7.85};

    const pr::Boid B_i{xi, vi, 1};

    const pr::Vector2 xj{2, 1};
    const pr::Vector2 vj{-8.98, 15.1};

    const pr::Boid B_j{xj, vj, 1};

    float a = 0.2;
    float n = 49;

    pr::Vector2 v2 = B_i.allignment(B_j, a, n, 100);

    CHECK(v2.x_axis() == doctest::Approx(-0.0407).epsilon(0.0001));
    CHECK(v2.y_axis() == doctest::Approx(0.0936).epsilon(0.0001));
  }
}

TEST_CASE("testing the cohesion() method") {
  SUBCASE("case 1") {
    const pr::Vector2 x_cm{3.5, 8.9};

    float c = 3;

    const pr::Vector2 xi{2.88, 6.2};
    const pr::Vector2 vi{1, 1};

    const pr::Boid B_i{xi, vi, 1};

    pr::Vector2 v3 = B_i.cohesion(x_cm, c);

    CHECK(v3.x_axis() == doctest::Approx(1.86).epsilon(0.0001));
    CHECK(v3.y_axis() == doctest::Approx(8.1).epsilon(0.0001));
  }

  SUBCASE("case 2") {
    const pr::Vector2 x_cm{2, 2};

    float c = 1;

    const pr::Vector2 xi{1, 1};
    const pr::Vector2 vi{1, 1};

    const pr::Boid B_i{xi, vi, 1};

    pr::Vector2 v3 = B_i.cohesion(x_cm, c);

    CHECK(v3.x_axis() == doctest::Approx(1).epsilon(0.0001));
    CHECK(v3.y_axis() == doctest::Approx(1).epsilon(0.0001));
  }

  SUBCASE("case 3") {
    const pr::Vector2 x_cm{-3.5, 55.9};

    float c = 1.3;

    const pr::Vector2 xi{5.87, -18.2};
    const pr::Vector2 vi{1, 1};

    const pr::Boid B_i{xi, vi, 1};

    pr::Vector2 v3 = B_i.cohesion(x_cm, c);

    CHECK(v3.x_axis() == doctest::Approx(-12.181).epsilon(0.0001));
    CHECK(v3.y_axis() == doctest::Approx(96.33).epsilon(0.0001));
  }
}

TEST_CASE("Testing the change_velocity() function") {
  SUBCASE("summed velocity with positive components") {
    pr::Vector2 v1{5.5, 5.5};
    pr::Vector2 v2{5, 5};

    pr::Boid b1{v1, v1, 1};

    b1.change_velocity(v2);

    CHECK(b1.velocity().x_axis() == 10.5);
    CHECK(b1.velocity().y_axis() == 10.5);
  }

  SUBCASE("summed velocity with negative components") {
    pr::Vector2 v1{5.5, 5.5};
    pr::Vector2 v2{-5, -5};

    pr::Boid b1{v1, v1, 1};

    b1.change_velocity(v2);

    CHECK(b1.velocity().x_axis() == 0.5);
    CHECK(b1.velocity().y_axis() == 0.5);
  }
}

TEST_CASE("Testing the change_position() function") {
  SUBCASE("summed position with positive components") {
    pr::Vector2 v1{5.5, 5.5};
    pr::Vector2 v2{5, 5};

    pr::Boid b1{v1, v1, 1};

    b1.change_position(v2);

    CHECK(b1.position().x_axis() == 10.5);
    CHECK(b1.position().y_axis() == 10.5);
  }

  SUBCASE("summed position with negative components") {
    pr::Vector2 v1{5.5, 5.5};
    pr::Vector2 v2{-5, -5};

    pr::Boid b1{v1, v1, 1};

    b1.change_position(v2);

    CHECK(b1.position().x_axis() == 0.5);
    CHECK(b1.position().y_axis() == 0.5);
  }
}

TEST_CASE("Testing the == operator") {
  SUBCASE("Same boid") {
    const pr::Vector2 v1{1., 1.};
    const pr::Vector2 v2{2., 2.};

    const pr::Boid b1{v1, v2, 1.};
    const pr::Boid b2{v1, v2, 1.};

    bool same = (b1 == b2);

    CHECK(same == true);
  }

  SUBCASE("different boid") {
    const pr::Vector2 v1{1., 1.};
    const pr::Vector2 v2{2., 2.};

    const pr::Boid b1{v1, v2, 1.};
    const pr::Boid b2{v2, v1, 1.};

    bool same = (b1 == b2);

    CHECK(same == false);
  }
}