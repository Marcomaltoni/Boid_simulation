#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "vector2.hpp"

#include "doctest.h"

TEST_CASE("Testing operator+") {
  SUBCASE("positive components") {
    const pr::Vector2 v1{1., 3.};
    const pr::Vector2 v2{3., 4.};
    const pr::Vector2 v3{3., 4.};
    const pr::Vector2 v4{v1 + v2 + v3};

    CHECK(v4.x_axis() == doctest::Approx(7.0).epsilon(0.1));
    CHECK(v4.y_axis() == doctest::Approx(11.0).epsilon(0.1));
  }

  SUBCASE("negative components") {
    const pr::Vector2 v1{-1., 3.};
    const pr::Vector2 v2{3., -4.};
    const pr::Vector2 v3{v1 + v2};

    CHECK(v3.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(-1.0).epsilon(0.1));
  }

  SUBCASE("null components") {
    const pr::Vector2 v1{1., 0.};
    const pr::Vector2 v2{0., 4.};
    const pr::Vector2 v3{v1 + v2};

    CHECK(v3.x_axis() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(4.0).epsilon(0.1));
  }
}

TEST_CASE("Testing operator-") {
  SUBCASE("positive components") {
    const pr::Vector2 v1{5., 6.};
    const pr::Vector2 v2{3., 2.};
    const pr::Vector2 v3{v1 - v2};

    CHECK(v3.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(4.0).epsilon(0.1));
  }

  SUBCASE("negative components") {
    const pr::Vector2 v1{-5., -6.};
    const pr::Vector2 v2{-3., 2.};
    const pr::Vector2 v3{v1 - v2};

    CHECK(v3.x_axis() == doctest::Approx(-2.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(-8.0).epsilon(0.1));
  }

  SUBCASE("null components") {
    const pr::Vector2 v1{5., 0.};
    const pr::Vector2 v2{0., 2.};
    const pr::Vector2 v3{v1 - v2};

    CHECK(v3.x_axis() == doctest::Approx(5.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(-2.0).epsilon(0.1));
  }
}

TEST_CASE("Testing distance method") {
  const pr::Vector2 v1{2., 3.};
  const pr::Vector2 v2{3., 4.};

  double distance = v1.distance(v2);

  CHECK(distance == doctest::Approx(1.414).epsilon(0.001));
}

TEST_CASE("Testing module method") {
  const pr::Vector2 v1{3., 4.};

  double lenght = v1.lenght_of_vector();

  CHECK(lenght == doctest::Approx(5.0).epsilon(0.1));
}

TEST_CASE("Testing operator!=") {
  SUBCASE("same vectors") {
    const pr::Vector2 v1{1., 2.};
    const pr::Vector2 v2{1., 2.};

    CHECK(v1 == v2);
  }

  SUBCASE("different vectors") {
    const pr::Vector2 v1{1., 2.};
    const pr::Vector2 v2{2., 3.};

    CHECK(v1 != v2);
  }
}

TEST_CASE("Testing operator==") {
  SUBCASE("same vectors") {
    const pr::Vector2 v1{1., 2.};
    const pr::Vector2 v2{1., 2.};

    CHECK(v1 == v2);
  }

  SUBCASE("1: different vectors") {
    const pr::Vector2 v1{1., 2.};
    const pr::Vector2 v2{2., 3.};

    CHECK(v1 != v2);
  }

  SUBCASE("2: different vectors") {
    const pr::Vector2 v1{1., 2.};
    const pr::Vector2 v2{1., 3.};

    CHECK(v1 != v2);
  }

  SUBCASE("3: different vectors") {
    const pr::Vector2 v1{4., 3.};
    const pr::Vector2 v2{1., 3.};

    CHECK(v1 != v2);
  }
}

TEST_CASE("Testing operator*") {
  SUBCASE("positive components") {
    const pr::Vector2 v1{1., 3.};

    double s = 2;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(6.0).epsilon(0.1));
  }

  SUBCASE("negative components") {
    const pr::Vector2 v1{-1., 3.};

    double s = -3;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(3.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(-9.0).epsilon(0.1));
  }

  SUBCASE("1: null components") {
    const pr::Vector2 v1{1., 0};

    double s = 2;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(0.0).epsilon(0.1));
  }

  SUBCASE("2: null components") {
    const pr::Vector2 v1{3., 2.};

    double s = 0;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(0.).epsilon(0.1));
  }
}

TEST_CASE("Testing the operator+= function") {
  SUBCASE("positive components") {
    pr::Vector2 v1{2, 2};
    const pr::Vector2 v2{2, 2};

    v1 += v2;

    CHECK(v1.x_axis() == doctest::Approx(4.0).epsilon(0.1));
    CHECK(v1.y_axis() == doctest::Approx(4.0).epsilon(0.1));
  }

  SUBCASE("negative components") {
    pr::Vector2 v1{2, 2};
    const pr::Vector2 v2{-2, -2};

    v1 += v2;

    CHECK(v1.x_axis() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(v1.y_axis() == doctest::Approx(0.0).epsilon(0.1));
  }
}
