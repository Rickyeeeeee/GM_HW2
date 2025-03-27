#pragma once

#include "miscellaneous/camera.h"
#include "miscellaneous/trackball.h"
#include "scene.h"
#include <GLFW/glfw3.h>
#include <ImGUI/imgui.h>
#include <glad/glad.h>
#include <string>
#include "renderer.h"

class Controller {
public:
  // Enum
  enum class SelectType { None, Faces, Vertices, Edges };

  // Constructor and Destructor
  Controller(int width, int height, GLFWwindow *window, Scene *scenePtr);
  ~Controller() = default;

  // Window Management
  void resize(int width, int height);

  // Rendering Interface
  void render(Renderer& renderer);

  // Input Handling Interface
  void handleMouseClick(int button, double xpos, double ypos, int action);
  void handleMouseMove(double xpos, double ypos);
  void handleKeyPress(int key);
  void handleMouseScroll(double xoffset, double yoffset);

private:
  // Misc functions
  void saveScreenshot();
  void exportCameraPosition(const std::string &filename = "camera.ini");
  void importCameraPosition(const std::string &filename = "camera.ini");

  // Selection functions
  int selectPoint(const double &x, const double &y);

  // Utility functions for selection
  static Eigen::Vector3f calculateWorldPoint(const double &x, const double &y, int height,
                                             Camera *camera);

  void updateSpline();

private:
  int screenWidth;
  int screenHeight;
  // State
  bool isDragging;
  // Components
  Scene *scene;
  std::unique_ptr<Trackball> trackball;
  // window
  GLFWwindow *window;
};
