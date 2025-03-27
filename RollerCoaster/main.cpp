#include <iostream>
#include <memory>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include "controller.h"
#include "model.h"
#include "renderer.h"
#include "scene.h"

namespace RollerCoaster {
// Constants
constexpr int WINDOW_WIDTH = 1024;
constexpr int WINDOW_HEIGHT = 1024;

std::unique_ptr<Scene> scene;
std::unique_ptr<Controller> controller;
std::unique_ptr<Renderer> renderer;

// GLFW callbacks
void framebufferSizeCallback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
  scene->resize(width, height);
  controller->resize(width, height);
  renderer->resize(width, height);
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT)
    return;
  controller->handleKeyPress(key);
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
  controller->handleMouseClick(button, xpos, ypos, action);
}

void cursorPosCallback(GLFWwindow *window, double xpos, double ypos) {
  controller->handleMouseMove(xpos, ypos);
}

void scrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
  controller->handleMouseScroll(xoffset, yoffset);
}

void printGLInfo() {
  std::cout << "GL_VENDOR: " << glGetString(GL_VENDOR) << '\n'
            << "GL_RENDERER: " << glGetString(GL_RENDERER) << '\n'
            << "GL_VERSION: " << glGetString(GL_VERSION) << '\n'
            << "GL_SHADING_LANGUAGE_VERSION: " << glGetString(GL_SHADING_LANGUAGE_VERSION)
            << std::endl;
}
} // namespace LearnPLY

int main(int argc, char **argv) {
  // Initialize GLFW
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    return -1;
  }

  // Configure GLFW
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // Create window
  GLFWwindow *window = glfwCreateWindow(RollerCoaster::WINDOW_WIDTH, RollerCoaster::WINDOW_HEIGHT,
                                        "RollerCoaster", nullptr, nullptr);
  if (!window) {
    std::cerr << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }

  glfwMakeContextCurrent(window);

  // Initialize GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  // Print OpenGL information
  RollerCoaster::printGLInfo();

  // Set callbacks
  glfwSetFramebufferSizeCallback(window, RollerCoaster::framebufferSizeCallback);
  glfwSetKeyCallback(window, RollerCoaster::keyCallback);
  glfwSetMouseButtonCallback(window, RollerCoaster::mouseButtonCallback);
  glfwSetCursorPosCallback(window, RollerCoaster::cursorPosCallback);
  glfwSetScrollCallback(window, RollerCoaster::scrollCallback);

  // Initialize OpenGL state
  glEnable(GL_DEPTH_TEST);

  // Initialize components
  auto model = std::make_unique<Model>();
  Spline* spline = model.get()->getSpline();
  std::vector<Eigen::Vector3f> points;
  points.emplace_back(-0.5f, 0.0f, -0.5f);
  points.emplace_back(-0.5f, 0.0f, 0.5f);
  points.emplace_back(0.5f, 0.0f, 0.5f);
  points.emplace_back(0.5f, 0.0f, -0.5f);
  spline->setAntribute(points, true);
  //
  RollerCoaster::scene =
      std::make_unique<Scene>(std::move(model), RollerCoaster::WINDOW_WIDTH, RollerCoaster::WINDOW_HEIGHT);
  RollerCoaster::renderer = std::make_unique<Renderer>(RollerCoaster::WINDOW_WIDTH, RollerCoaster::WINDOW_HEIGHT,
                                                  RollerCoaster::scene.get());
  RollerCoaster::controller = std::make_unique<Controller>(
      RollerCoaster::WINDOW_WIDTH, RollerCoaster::WINDOW_HEIGHT, window, RollerCoaster::scene.get());
  RollerCoaster::scene->updateCarts();
  RollerCoaster::scene->updateCurveRenderer();
  //
  // Main loop
  while (!glfwWindowShouldClose(window)) {
    // update function
    RollerCoaster::scene->update();

    // render
    RollerCoaster::renderer->render();
    RollerCoaster::controller->render(*RollerCoaster::renderer);

    // swap buffer & polling
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Cleanup is handled by smart pointers
  glfwTerminate();
  return 0;
}
