#pragma once

#include "helpers/arrowhelper.h"
#include "helpers/axeshelper.h"
#include "helpers/boxhelper.h"
#include "helpers/vertexhelper.h"

#include "scene.h"

class Renderer {
public:
  Renderer(int width, int height, Scene *scenePtr);
  ~Renderer();

  void resize(int width, int height);
  void render();
  void setRGB(bool isRGB) { this->isRGB = isRGB; }

private:
  void setupShaders();
  void releaseShaders();

private:
  int screenWidth;
  int screenHeight;

  // State
  Scene *scene;

  // Shader
  Shader *plyShader;
  Shader* RGBplyShader;
  Shader *axesShader;
  Shader *colorShader;
  bool isRGB = false;

  // Helpers
  std::unique_ptr<VertexHelper> vertexHelper;
  std::unique_ptr<AxesHelper> axesHelper;
  std::unique_ptr<BoxHelper> boxHelper;
  std::unique_ptr<ArrowHelper> arrowHelper;
};
