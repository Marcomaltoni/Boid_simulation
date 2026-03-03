# 🦅 Boids Flocking Simulation

I presented this project for the exam of **"Programming for physics"** during my bachelor thesis in physics at the University of Bologna. Its goal is to simulate the
behaviour of a flock in a 2D space, described using "Reynolds algorithm", and the code is written in **C++**.


## 📋 Characteristics
- **Mathematical Model**: Implementation of three fundamental rules $\to$ *Separation*, *Allignment* e *Cohesion*.
- **2D vectors**: the physics of the objects is described through an optimized `Vector2` class.
- **Unit Testing**: integrated test suite to verify the correct behaviour of all the components.
- **Build System**: use of **Cmake** for cross-platform compilation.

## 🛠️ Coding tools
* **Language**: C++ 
* **Build System**: CMake
* **Test Framework**: [doctest](https://github.com/doctest/doctest)
* **External libraries**: SFML 

---

## 🚀 Compilation and execution (Ubuntu/Linux)

Make sure to have installed `cmake` and the graphical libraries ( SFML):
```bash
sudo apt-get update
sudo apt-get install cmake build-essential libsfml-dev
