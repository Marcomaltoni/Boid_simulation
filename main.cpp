#include <iomanip>
#include <iostream>
#include <random>

#include "SFML/Graphics.hpp"
#include "flock.hpp"

int main() {
  std::cout << "Insert the following parameters: \n"
            << " 1- Parameter which determines when two boids are 'near', "
               "values permitted are between [50; 200]: \n";
  float closeness_parameter;
  std::cin >> closeness_parameter;

  std::cout
      << " 2- Parameter which represents the distance at which starts to act "
         "the separation rule among boids, values permitted are between [25; "
         "40]: \n";
  float distance_of_separation;
  std::cin >> distance_of_separation;

  std::cout << " 3- Parameter which determines the intensity of the separation "
               "among boids, values permitted are between [0.005; 0.8]: \n";
  float separation_parameter;
  std::cin >> separation_parameter;

  std::cout << " 4- Parameter which determines the intensity of the allignment "
               "among boids, values permitted are between [0.2; 0.8]: \n";
  float allignment_parameter;
  std::cin >> allignment_parameter;

  std::cout
      << " 5- Parameter which determines the intensity of the cohesion among "
         "boids, values permitted are between [0.0001; 0.001]: "
         "(after you'll have inserted it, you'll be able to start the "
         "simulation: press the left button of your mouse/touchpad to make a "
         "boid appear, press the right one to make a predator appear.)\n";
  float cohesion_parameter;
  std::cin >> cohesion_parameter;

  sf::RenderWindow window(
      sf::VideoMode(0.9 * sf::VideoMode::getDesktopMode().width,
                    0.9 * sf::VideoMode::getDesktopMode().height),
      "FLOCK SIMULATION");

  sf::Event event;

  sf::Clock clock1;
  sf::Clock clock2;

  sf::Texture texture;

  if (!texture.loadFromFile("cielo.jpg")) {
    std::cout << "Error loading the background image\n";
  }

  sf::Sprite sprite;

  sprite.setTexture(texture);

  const float background_scaleX =
      static_cast<float>(window.getSize().x) /
      texture.getSize()
          .x;  // scale factor to adapt the background image to the window.
  const float background_scaleY =
      static_cast<float>(window.getSize().y) / texture.getSize().y;

  sprite.setScale(background_scaleX, background_scaleY);
  sprite.setPosition(0.f, 0.f);

  std::default_random_engine rand_engine;
  std::normal_distribution<float> velocity_distribution;
  std::uniform_int_distribution<> angle_distribution(120, 180);

  pr::Flock flock{closeness_parameter, distance_of_separation,
                  separation_parameter, allignment_parameter,
                  cohesion_parameter};

  while (window.isOpen()) {
    while (window.pollEvent(event)) {
      switch (event.type) {
        case sf::Event::Closed:
          window.close();

          break;

        case sf::Event::MouseButtonPressed: {
          switch (event.mouseButton.button) {
            case sf::Mouse::Left: {
              const sf::Vector2i position = sf::Mouse::getPosition(window);
              const float positionf_x = static_cast<float>(position.x);
              const float positionf_y = static_cast<float>(position.y);
              const pr::Vector2 position_f{positionf_x, positionf_y};

              const pr::Vector2 speed{velocity_distribution(rand_engine),
                                      velocity_distribution(rand_engine)};

              float view_angle =
                  static_cast<float>(angle_distribution(rand_engine));

              pr::Boid boid{position_f, speed, 0.5f, view_angle};
              boid.setPosition(position_f);
              boid.setRotation();
              boid.setPointCount();
              boid.setRadius(10.f);
              boid.setScale(1.f, 1.5f);
              boid.setFillColor(sf::Color::Red);
              boid.setOrigin(8.6602540f, 7.5f);

              flock.push_back(boid);

              break;
            }

            case sf::Mouse::Right: {
              const sf::Vector2i position = sf::Mouse::getPosition(window);
              const float positionf_x = static_cast<float>(position.x);
              const float positionf_y = static_cast<float>(position.y);
              const pr::Vector2 position_f{positionf_x, positionf_y};

              const pr::Vector2 speed{velocity_distribution(rand_engine),
                                      velocity_distribution(rand_engine)};

              float view_angle =
                  static_cast<float>(angle_distribution(rand_engine));

              pr::Boid boid{position_f, speed, 0.5f, view_angle};
              boid.setPosition(position_f);
              boid.setRotation();
              boid.setPointCount();
              boid.setRadius(15.f);
              boid.setScale(1.f, 1.5f);
              boid.setFillColor(sf::Color::Black);
              boid.setOrigin(12.9903811f, 10.0f);

              flock.push_back(boid);

              break;
            }

            default:

              break;
          }
          break;
        }

        default:

          break;
      }
    }

    sf::Time time_per_frame = clock1.restart();
    sf::Time time_passed = clock2.getElapsedTime();

    flock.update(time_per_frame, 0.9 * sf::VideoMode::getDesktopMode().width,
                 0.9 * sf::VideoMode::getDesktopMode().height);

    const pr::Simulation_state flock_state = flock.state();

    if (flock.all_boids().size() >= 2 && time_passed.asSeconds() >= 1.f) {
      std::cout << "Medium velocity: " << flock_state.medium_velocity << " +/- "
                << flock_state.err_velocity << ";       "
                << "Medium distance among boids: "
                << flock_state.medium_distance << " +/- "
                << flock_state.err_distance << ";\n";

      clock2.restart();
    }

    window.clear();

    window.draw(sprite);

    for (pr::Boid& boid : flock.all_boids()) {
      window.draw(boid.set_shape());
    }

    window.display();
  }
  return 0;
}
