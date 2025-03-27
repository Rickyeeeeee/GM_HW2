#include "controller.h"
#include <cstddef>
#include <ImGUI/imgui_impl_glfw.h>
#include <ImGUI/imgui_impl_opengl3.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include "mesh/learnply.h"
#include "mesh/meshprocessor.h"
#include "curves/CurveProcessor.h"

#define VRAD 0.05f

// Constants
#define ZOOM_SPEED 0.1f
#define PAN_SPEED 0.1f
#define M_PI 3.14159265358979323846f
float deg2rad(float degrees) { return degrees * (M_PI / 180.0f); }

Controller::Controller(int width, int height, GLFWwindow *window, Scene *scenePtr)
    : screenWidth(width), screenHeight(height), isDragging(false), scene(scenePtr) {
  // Trackball initialization
  trackball = std::make_unique<Trackball>();
  trackball->setCamera(scene->getCamera());
  trackball->start(Trackball::Around);
  // GUI Initialization
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.FontGlobalScale = 2.0f;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 130");
  this->window = window;
}

void Controller::resize(int width, int height) {
  screenWidth = width;
  screenHeight = height;
}

void Controller::handleMouseClick(int button, double xpos, double ypos, int action) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    isDragging = (action == GLFW_PRESS);
    if (isDragging) {
      trackball->start(Trackball::Around);
    }
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    if (action == GLFW_PRESS) {
      selectPoint(xpos, ypos);
    } else if (action == GLFW_RELEASE) {
    }
  }
}

void Controller::handleMouseMove(double xpos, double ypos) {
  if (isDragging) {
    trackball->track(Eigen::Vector2i(xpos, ypos));
  }
}

void Controller::handleKeyPress(int key) {
  auto camera = scene->getCamera();
  if (key == GLFW_KEY_W) {
    camera->zoom(ZOOM_SPEED);
  } else if (key == GLFW_KEY_S) {
    camera->zoom(-ZOOM_SPEED);
  } else if (key == GLFW_KEY_A) {
    camera->localTranslate(Eigen::Vector3f(-PAN_SPEED, 0, 0));
  } else if (key == GLFW_KEY_D) {
    camera->localTranslate(Eigen::Vector3f(PAN_SPEED, 0, 0));
  } else if (key == GLFW_KEY_Z) {
    camera->localTranslate(Eigen::Vector3f(0, PAN_SPEED, 0));
  } else if (key == GLFW_KEY_X) {
    camera->localTranslate(Eigen::Vector3f(0, -PAN_SPEED, 0));
  }
}

void Controller::handleMouseScroll(double xoffset, double yoffset) {
  auto camera = scene->getCamera();
  if (camera) {
    camera->zoom(yoffset * ZOOM_SPEED);
  }
}

void Controller::render(Renderer& renderer) {
  // Render GUI
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGui::Begin("Control Panel", NULL, ImGuiWindowFlags_AlwaysAutoResize);

  Spline* spline = scene->getModel()->getSpline();
  // Misc controls
  if (ImGui::CollapsingHeader("Misc.")) {
    if (ImGui::Button("Screenshot"))
      saveScreenshot();
    if (ImGui::Button("Save Camera Position"))
      exportCameraPosition();
    if (ImGui::Button("Load Camera Position"))
      importCameraPosition();

    if (ImGui::Button("Save Spline")) {
        CurveProcessor::saveSpline(spline);
    }
    if (ImGui::Button("Load Spline")) {
        CurveProcessor::loadSpline(scene->getModel());
        scene->updateCurveRenderer();
    }
    ImGui::Separator();
  }

  if (ImGui::CollapsingHeader("Vis.")) {
      bool flag1 = scene->getShowPoints();
      if (ImGui::Checkbox("Show Points", &flag1)) {
          scene->setShowPoints(flag1);
      }
      bool flag2 = scene->getShowFarmes();
      if (ImGui::Checkbox("Show Frames", &flag2)) {
          scene->setShowFarmes(flag2);
      }
      bool flag3 = scene->getShowCurvatures();
      if (ImGui::Checkbox("Show Curvature", &flag3)) {
          scene->setShowCurvatures(flag3);
      }
	  static bool isRGB = false;
	  if (ImGui::Checkbox("RGB", &isRGB)) {
		  renderer.setRGB(isRGB);
	  }
      ImGui::Separator();
  }

  bool flag_convertType = false;
  int convertType = static_cast<int>(spline->getType());
  if (ImGui::CollapsingHeader("Spline")) {
    ImGui::Text("Curve Type");
    const char *items[] = {"Polyline", "BSpline", "Catmull-Rom" };
    if (ImGui::Combo("##CurveType", &convertType, items, IM_ARRAYSIZE(items))) {
        flag_convertType = true;
    }

    bool flag0 = spline->getLoop();
    if (ImGui::Checkbox("Loop", &flag0)) {
        spline->setLoop(flag0);
        updateSpline();
    }
    bool flag1 = scene->getModel()->getUseUntiSpeed();
    if (ImGui::Checkbox("Unit Speed", &flag1)) {
        scene->getModel()->setUseUntiSpeed(flag1);
    }
    bool flag2 = scene->getModel()->getUseBishop();
    if (ImGui::Checkbox("Bishop Frame", &flag2)) {
        scene->getModel()->setUseBishop(flag2);
        scene->updateCarts();
    }

    if (ImGui::Button("Add Point")) {
      spline->addPoint();
      updateSpline();
    }
    
    bool active = spline->isSelectedPoint();
    if (active) {
        Eigen::Vector3f point = spline->getSelectedPoint();
        // Add Delete button at the top of the window
        if (ImGui::Button("Delete Point")) {
            spline->removePoint();
            updateSpline();
        }
        ImGui::Separator(); 
        // CONTROL POINTS
        {
            // X coordinate control
            ImGui::Text("X:");
            ImGui::SameLine();
            if (ImGui::ArrowButton("##left_x", ImGuiDir_Left)) {
                point[0] -= 0.1f;
                spline->setSelectedPoint(point);
                updateSpline();
            }
            ImGui::SameLine();
            char buf[32];
            sprintf(buf, "%.3f", point[0]);
            if (ImGui::InputText("##x", buf, IM_ARRAYSIZE(buf), ImGuiInputTextFlags_EnterReturnsTrue)) {
                point[0] = std::stof(buf);
                spline->setSelectedPoint(point);
                updateSpline();
            }
            ImGui::SameLine();
            if (ImGui::ArrowButton("##right_x", ImGuiDir_Right)) {
                point[0] += 0.1f;
                spline->setSelectedPoint(point);
                updateSpline();
            }

            // Y coordinate control
            ImGui::Text("Y:");
            ImGui::SameLine();
            if (ImGui::ArrowButton("##left_y", ImGuiDir_Left)) {
                point[1] -= 0.1f;
                spline->setSelectedPoint(point);
                updateSpline();
            }
            ImGui::SameLine();
            sprintf(buf, "%.3f", point[1]);
            if (ImGui::InputText("##y", buf, IM_ARRAYSIZE(buf), ImGuiInputTextFlags_EnterReturnsTrue)) {
                point[1] = std::stof(buf);
                spline->setSelectedPoint(point);
                updateSpline();
            }
            ImGui::SameLine();
            if (ImGui::ArrowButton("##right_y", ImGuiDir_Right)) {
                point[1] += 0.1f;
                spline->setSelectedPoint(point);
                updateSpline();
            }

            // Z coordinate control
            ImGui::Text("Z:");
            ImGui::SameLine();
            if (ImGui::ArrowButton("##left_z", ImGuiDir_Left)) {
                point[2] -= 0.1f;
                spline->setSelectedPoint(point);
                updateSpline();
            }
            ImGui::SameLine();
            sprintf(buf, "%.3f", point[2]);
            if (ImGui::InputText("##z", buf, IM_ARRAYSIZE(buf), ImGuiInputTextFlags_EnterReturnsTrue)) {
                point[2] = std::stof(buf);
                spline->setSelectedPoint(point);
                updateSpline();
            }
            ImGui::SameLine();
            if (ImGui::ArrowButton("##right_z", ImGuiDir_Right)) {
                point[2] += 0.1f;
                spline->setSelectedPoint(point);
                updateSpline();
            }
        }
    }
  }

  ImGui::Separator(); 
  {
      if (scene->getAnimation())
      {
          std::string str("U: ");
          str += std::to_string(scene->getTimeElapsed());
          ImGui::Text(str.c_str());
      }
      else
      {
          float t = scene->getTimeElapsed();
          ImGui::Text("U:");
          char buf[32];
          sprintf(buf, "%.3f", t);
          ImGui::SameLine();
          if (ImGui::ArrowButton("##left_u", ImGuiDir_Left)) {
              t -= 0.1f;
              scene->setTimeElapsed(t);
              scene->updateCarts();
          }
          ImGui::SameLine();
          if (ImGui::InputText("##u", buf, IM_ARRAYSIZE(buf), ImGuiInputTextFlags_EnterReturnsTrue)) {
              scene->setTimeElapsed(std::stof(buf));
              scene->updateCarts();
          }
          ImGui::SameLine();
          if (ImGui::ArrowButton("##right_u", ImGuiDir_Right)) {
              t += 0.1f;
              scene->setTimeElapsed(t);
              scene->updateCarts();
          }
      }
  } 
  {
      std::string str("K: ");
      double curvature = spline->getCurvature(
          scene->getTimeElapsed(), 
          scene->getModel()->getUseUntiSpeed());
      str += std::to_string(curvature);
      ImGui::Text(str.c_str());
  }
  if (ImGui::Button("Toggle Animation")) {
      scene->toggleAnimation();
  }

  if (flag_convertType)
  {
      CurveProcessor::convertSplineType(scene->getModel(), static_cast<Spline::Type>(convertType));
      updateSpline();
  }

  ImGui::End();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

int Controller::selectPoint(const double &x, const double &y) {
  Eigen::Vector3f pt = calculateWorldPoint(x, y, screenHeight, scene->getCamera());
  Spline *spline = scene->getModel()->getSpline();
  std::vector<Eigen::Vector3f> &points = spline->getPoints();

  float longest = VRAD * VRAD * 4.0f;
  int index = -1;

  for (int i = 0; i < points.size(); i++) {
    float d = (points[i] - pt).squaredNorm();
    if (d < longest) {
      longest = d;
      index = i;
    }
  }

  spline->setSelectedIdx(index);

  return index;
}

Eigen::Vector3f Controller::calculateWorldPoint(const double &x, const double &y, int height,
                                                Camera *camera) {
  // Calculate depth
  float depth_buffer;
  glReadPixels(x, height - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_buffer);
  float near = camera->nearDist();
  float far = camera->farDist();
  depth_buffer = (2.0f * depth_buffer) - 1.0f;
  depth_buffer = (2.0f * near * far) / (far + near - depth_buffer * (far - near));

  // Calculate Point in 3D World
  Eigen::Vector2f point2D(x, height - y);
  Eigen::Vector3f pt = camera->unProject(point2D, depth_buffer);

  return pt;
}

void Controller::updateSpline()
{
    scene->updateCurveRenderer();
    scene->updateCarts();
}

void Controller::saveScreenshot() {
  std::vector<unsigned char> pixels(screenWidth * screenHeight * 3);
  glReadPixels(0, 0, screenWidth, screenHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "screenshot_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".ppm";

  std::ofstream file(ss.str(), std::ios::binary);
  if (!file) {
    std::cerr << "Failed to open file for writing: " << ss.str() << std::endl;
    return;
  }

  file << "P6\n" << screenWidth << " " << screenHeight << "\n255\n";

  for (int y = screenHeight - 1; y >= 0; y--) {
    for (int x = 0; x < screenWidth; x++) {
      int idx = (y * screenWidth + x) * 3;
      file.write(reinterpret_cast<char *>(&pixels[idx]), 3);
    }
  }

  file.close();
  std::cout << "Screenshot saved to: " << ss.str() << std::endl;
}

void Controller::exportCameraPosition(const std::string &filename) {
  if (!scene->getCamera()) {
    std::cerr << "Camera not initialized" << std::endl;
    return;
  }

  std::ofstream file(filename);
  if (!file) {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }

  Eigen::Vector3f position = scene->getCamera()->position();
  Eigen::Vector3f target = scene->getCamera()->target();
  float fovy = scene->getCamera()->fovY();

  file << "[Camera]\n";
  file << "position_x=" << position.x() << "\n";
  file << "position_y=" << position.y() << "\n";
  file << "position_z=" << position.z() << "\n";
  file << "target_x=" << target.x() << "\n";
  file << "target_y=" << target.y() << "\n";
  file << "target_z=" << target.z() << "\n";
  file << "fovy=" << fovy << "\n";

  file.close();
  std::cout << "Camera position saved to: " << filename << std::endl;
}

void Controller::importCameraPosition(const std::string &filename) {
  if (!scene->getCamera()) {
    std::cerr << "Camera not initialized" << std::endl;
    return;
  }

  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Failed to open file for reading: " << filename << std::endl;
    return;
  }

  Eigen::Vector3f position, target;
  float fovy;
  std::string line;
  std::string section;

  while (std::getline(file, line)) {
    // Trim whitespace
    line.erase(0, line.find_first_not_of(" \t"));
    line.erase(line.find_last_not_of(" \t") + 1);

    if (line.empty() || line[0] == ';')
      continue;

    if (line[0] == '[' && line[line.length() - 1] == ']') {
      section = line.substr(1, line.length() - 2);
      continue;
    }

    if (section == "Camera") {
      size_t delimPos = line.find('=');
      if (delimPos != std::string::npos) {
        std::string key = line.substr(0, delimPos);
        float value = std::stof(line.substr(delimPos + 1));

        if (key == "position_x")
          position.x() = value;
        else if (key == "position_y")
          position.y() = value;
        else if (key == "position_z")
          position.z() = value;
        else if (key == "target_x")
          target.x() = value;
        else if (key == "target_y")
          target.y() = value;
        else if (key == "target_z")
          target.z() = value;
        else if (key == "fovy")
          fovy = value;
      }
    }
  }

  scene->getCamera()->setPosition(position);
  scene->getCamera()->setTarget(target);
  scene->getCamera()->setFovY(fovy);

  file.close();
  std::cout << "Camera position loaded from: " << filename << std::endl;
}