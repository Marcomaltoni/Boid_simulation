#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "vector2.hpp"

#include "doctest.h"

TEST_CASE("Testing the operator+=") {
  SUBCASE("Positive components:") {
    pr::Vector2 v1{2.f, 2.f};
    const pr::Vector2 v2{2.f, 2.f};

    v1 += v2;

    CHECK(v1.x_axis() == doctest::Approx(4.0).epsilon(0.1));
    CHECK(v1.y_axis() == doctest::Approx(4.0).epsilon(0.1));
  }

  SUBCASE("Negative components:") {
    pr::Vector2 v1{2.f, 2.f};
    const pr::Vector2 v2{-2.f, -2.f};

    v1 += v2;

    CHECK(v1.x_axis() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(v1.y_axis() == doctest::Approx(0.0).epsilon(0.1));
  }
}

TEST_CASE("Testing operator+") {
  SUBCASE("Positive components:") {
    const pr::Vector2 v1{1.f, 3.f};
    const pr::Vector2 v2{3.f, 4.f};
    const pr::Vector2 v3{3.f, 4.f};
    const pr::Vector2 v4{v1 + v2 + v3};

    CHECK(v4.x_axis() == doctest::Approx(7.0).epsilon(0.1));
    CHECK(v4.y_axis() == doctest::Approx(11.0).epsilon(0.1));
  }

  SUBCASE("Negative components:") {
    const pr::Vector2 v1{-1.f, 3.f};
    const pr::Vector2 v2{3.f, -4.f};
    const pr::Vector2 v3{v1 + v2};

    CHECK(v3.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(-1.0).epsilon(0.1));
  }

  SUBCASE("Null components:") {
    const pr::Vector2 v1{1.f, 0.f};
    const pr::Vector2 v2{0.f, 4.f};
    const pr::Vector2 v3{v1 + v2};

    CHECK(v3.x_axis() == doctest::Approx(1.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(4.0).epsilon(0.1));
  }
}

TEST_CASE("Testing operator-") {
  SUBCASE("Positive components:") {
    const pr::Vector2 v1{5.f, 6.f};
    const pr::Vector2 v2{3.f, 2.f};
    const pr::Vector2 v3{v1 - v2};

    CHECK(v3.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(4.0).epsilon(0.1));
  }

  SUBCASE("Negative components:") {
    const pr::Vector2 v1{-5.f, -6.f};
    const pr::Vector2 v2{-3.f, 2.f};
    const pr::Vector2 v3{v1 - v2};

    CHECK(v3.x_axis() == doctest::Approx(-2.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(-8.0).epsilon(0.1));
  }

  SUBCASE("Null components:") {
    const pr::Vector2 v1{5.f, 0.f};
    const pr::Vector2 v2{0.f, 2.f};
    const pr::Vector2 v3{v1 - v2};

    CHECK(v3.x_axis() == doctest::Approx(5.0).epsilon(0.1));
    CHECK(v3.y_axis() == doctest::Approx(-2.0).epsilon(0.1));
  }
}

TEST_CASE("Testing distance() method") {
  const pr::Vector2 v1{2.f, 3.f};
  const pr::Vector2 v2{3.f, 4.f};

  double distance = v1.distance(v2);

  CHECK(distance == doctest::Approx(1.414).epsilon(0.001));
}

TEST_CASE("Testing lenght_of_vector() method") {
  const pr::Vector2 v1{3.f, 4.f};

  double lenght = v1.lenght_of_vector();

  CHECK(lenght == doctest::Approx(5.0).epsilon(0.1));
}

TEST_CASE("Testing operator!=") {
  SUBCASE("Same vectors:") {
    const pr::Vector2 v1{1.f, 2.f};
    const pr::Vector2 v2{1.f, 2.f};

    CHECK(v1 == v2);
  }

  SUBCASE("Different vectors:") {
    const pr::Vector2 v1{1.f, 2.f};
    const pr::Vector2 v2{2.f, 3.f};

    CHECK(v1 != v2);
  }
}

TEST_CASE("Testing operator==") {
  SUBCASE("Same vectors:") {
    const pr::Vector2 v1{1.f, 2.f};
    const pr::Vector2 v2{1.f, 2.f};

    CHECK(v1 == v2);
  }

  SUBCASE("Different vectors:") {
    const pr::Vector2 v1{1.f, 2.f};
    const pr::Vector2 v2{2.f, 3.f};

    CHECK(v1 != v2);
  }

  SUBCASE("Different vectors:") {
    const pr::Vector2 v1{1.f, 2.f};
    const pr::Vector2 v2{1.f, 3.f};

    CHECK(v1 != v2);
  }

  SUBCASE("Different vectors:") {
    const pr::Vector2 v1{4.f, 3.f};
    const pr::Vector2 v2{1.f, 3.f};

    CHECK(v1 != v2);
  }
}

TEST_CASE("Testing operator*") {
  SUBCASE("Positive components:") {
    const pr::Vector2 v1{1.f, 3.f};

    const double s = 2.f;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(6.0).epsilon(0.1));
  }

  SUBCASE("Negative components:") {
    const pr::Vector2 v1{-1.f, 3.f};

    const double s = -3.f;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(3.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(-9.0).epsilon(0.1));
  }

  SUBCASE("Null y component:") {
    const pr::Vector2 v1{1.f, 0.f};

    const double s = 2.f;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(2.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(0.0).epsilon(0.1));
  }

  SUBCASE("Null scalar:") {
    const pr::Vector2 v1{3.f, 2.f};

    const double s = 0.f;

    pr::Vector2 x{v1 * s};

    CHECK(x.x_axis() == doctest::Approx(0.0).epsilon(0.1));
    CHECK(x.y_axis() == doctest::Approx(0.0).epsilon(0.1));
  }
}
