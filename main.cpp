#include "flock.hpp"
#include <iomanip>
#include <iostream>
#include <random>

#include "SFML/Graphics.hpp"


int main() {
  std::cout << "Insert the following parameters: \n"
            << " 1- Parameter which determines when two boids are 'near': \n";
  float d;
  std::cin >> d;

  std::cout << " 2- Parameter which represents the distance at which starts "
               "the separation among boids: \n";
  float ds;
  std::cin >> ds;

  std::cout << " 3- Parameter which determines the intensity of the separation "
               "among boids: \n";
  float s;
  std::cin >> s;

  std::cout << " 4- Parameter which determines the intensity of the allignment "
               "among boids: \n";
  float a;
  std::cin >> a;

  std::cout << " 5- Parameter which determines the cohesion among boids: "
               "(after you'll have inserted it, you'll be able to start the "
               "simulation: press the left button of your mouse/pad to make a "
               "boid appear, press the right one to make a predator appear.)\n";
  float c;
  std::cin >> c;

  sf::RenderWindow window(
      sf::VideoMode(0.9 * sf::VideoMode::getDesktopMode().width,
                    0.9 * sf::VideoMode::getDesktopMode().height),
      "FLOCK SIMULATION");

  sf::Event event;

  sf::Clock clock1;
  sf::Clock clock2;

  sf::Texture texture;

  if (!texture.loadFromFile("cielo.jpg")) {
    std::cout << "Error loading the background\n";
  }

  sf::Sprite sprite;

  sprite.setTexture(texture);

  const float scaleX =
      static_cast<float>(window.getSize().x) /
      texture.getSize()
          .x;  // scale factor to adapt the background image to the window.
  const float scaleY =
      static_cast<float>(window.getSize().y) / texture.getSize().y;

  sprite.setScale(scaleX, scaleY);
  sprite.setPosition(0.f, 0.f);

  std::default_random_engine eng;
  std::normal_distribution<float> dist;

  pr::Flock flock{d, ds, s, a, c};

  while (window.isOpen()) {
    while (window.pollEvent(event)) {
      switch (event.type) {
        case sf::Event::Closed:
          window.close();

          break;

        case sf::Event::MouseButtonPressed: {
          switch (event.mouseButton.button) {
            case sf::Mouse::Left: {
              sf::Vector2i position = sf::Mouse::getPosition(window);
              float positionf_x = static_cast<float>(position.x);
              float positionf_y = static_cast<float>(position.y);
              pr::Vector2 position_f{positionf_x, positionf_y};

              pr::Vector2 speed{dist(eng), dist(eng)};

              pr::Boid boid{position_f, speed, 0.5f};
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
              sf::Vector2i position = sf::Mouse::getPosition(window);
              float positionf_x = static_cast<float>(position.x);
              float positionf_y = static_cast<float>(position.y);
              pr::Vector2 position_f{positionf_x, positionf_y};

              pr::Vector2 speed{dist(eng), dist(eng)};

              pr::Boid boid{position_f, speed, 0.5f};
              boid.setPosition(position_f);
              boid.setRotation();
              boid.setPointCount();
              boid.setRadius(15.f);
              boid.setScale(1.f, 1.5f);
              boid.setFillColor(sf::Color::Black);
              boid.setOrigin(12.99038106f, 10.0f);

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

    pr::Result res = flock.state();

if(flock.n_birds().size() >= 2 && time_passed.asSeconds() >= 1.f){ 
    std::cout << "Medium velocity: " << res.medium_velocity << " +/- "
              << res.err_velocity << ";       "
              << "Medium distance among boids: " << res.medium_distance
              << " +/- " << res.err_distance << ";\n";
              
              clock2.restart();}

    window.clear();

    window.draw(sprite);

    for (pr::Boid& x : flock.n_birds()) {
      window.draw(x.shape());
    }

    window.display();
  }
  return 0;
}
