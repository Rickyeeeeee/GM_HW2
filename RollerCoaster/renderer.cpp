#include "renderer.h"

Renderer::Renderer(int width, int height, Scene *scenePtr)
    : screenWidth(width), screenHeight(height), scene(scenePtr), plyShader(nullptr),
      axesShader(nullptr), colorShader(nullptr) {
  if (!scene) {
    throw std::runtime_error("Scene pointer is null");
  }
  if (!scene->getCamera()) {
    throw std::runtime_error("Camera is null");
  }

  setupShaders();

  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  glEnable(GL_DEPTH_TEST);

  vertexHelper = std::make_unique<VertexHelper>();
  vertexHelper->initialize(colorShader);

  axesHelper = std::make_unique<AxesHelper>();
  axesHelper->initialize(axesShader, 1.0f);

  boxHelper = std::make_unique<BoxHelper>();
  boxHelper->initialize(RGBplyShader);

  arrowHelper = std::make_unique<ArrowHelper>();
  arrowHelper->initialize(colorShader);
}

Renderer::~Renderer() {}

void Renderer::setupShaders() {
  plyShader = new Shader("../shaders/plyShader.vs.glsl", "../shaders/plyShader.fs.glsl"); // Default
  RGBplyShader = new Shader("../shaders/plyShader.vs.glsl", "../shaders/plyShaderRGB.fs.glsl"); 
  colorShader = new Shader("../shaders/colorShader.vs.glsl", "../shaders/colorShader.fs.glsl");
  axesShader = new Shader("../shaders/axesShader.vs.glsl", "../shaders/axesShader.fs.glsl");
}

void Renderer::releaseShaders() {
  if (plyShader != nullptr) {
    delete plyShader;
    plyShader = nullptr;
  }
  if (colorShader != nullptr) {
    delete colorShader;
    colorShader = nullptr;
  }

  if (axesShader != nullptr) {
    delete axesShader;
    axesShader = nullptr;
  }
}

void Renderer::render() {
  if (!scene)
    return;

  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // Take camera parameter
  const Camera *cam = scene->getCamera();
  const Eigen::Matrix4f &projection = cam->projectionMatrix();
  const Eigen::Matrix4f &view = cam->viewMatrix().matrix();
  const Eigen::Vector3f &camPos = cam->position();
  // Spline Points
  Spline *spline = scene->getModel()->getSpline();
  if (scene->getShowPoints()) {
    std::vector<Eigen::Vector3f> &points = spline->getPoints();
    vertexHelper->use(projection, view, camPos);
    for (int i = 0; i < points.size(); i++) {
      bool flag = (spline->getSelectedIdx() == i);
      vertexHelper->draw(points[i], flag);
    }
  }
  // Spline Pipe
  {
	  if (isRGB) {
		  RGBplyShader->use();
		  RGBplyShader->setMat4("projection_matrix", projection);
		  RGBplyShader->setMat4("view_matrix", view);
		  RGBplyShader->setVec3("viewPos", camPos);
		  RGBplyShader->setMat4("model_matrix", Eigen::Matrix4f::Identity());
		  scene->getModel()->getCurveRenderer()->draw();
	  }
	  else {
		  plyShader->use();
		  plyShader->setMat4("projection_matrix", projection);
		  plyShader->setMat4("view_matrix", view);
		  plyShader->setVec3("viewPos", camPos);
		  plyShader->setMat4("model_matrix", Eigen::Matrix4f::Identity());
		  scene->getModel()->getCurveRenderer()->draw();
	  }
  }
  // Carts
  {
    std::vector<Cart> &carts = scene->getModel()->getCarts();
    boxHelper->use(projection, view, camPos);
    for (int i = 0; i < (int)carts.size(); i++) {
      boxHelper->draw(carts[i].getPosition(), carts[i].getTangent(), carts[i].getNormal(), 0.1f);
    }
    if (scene->getShowFarmes()) {
      arrowHelper->use(projection, view, camPos);
      for (int i = 0; i < (int)carts.size(); i++) {
        const Eigen::Vector3f &t = carts[i].getTangent();
        const Eigen::Vector3f &n = carts[i].getNormal();
        Eigen::Vector3f b = n.cross(t);
        arrowHelper->setColor(0.5f, 0.5f, 1.0f);
        arrowHelper->draw(carts[i].getPosition(), t, 0.2f, 0.015f);
        arrowHelper->setColor(1.0f, 0.5f, 0.5f);
        arrowHelper->draw(carts[i].getPosition(), n, 0.2f, 0.015f);
        arrowHelper->setColor(0.5f, 1.0f, 0.5f);
        arrowHelper->draw(carts[i].getPosition(), b, 0.2f, 0.015f);
      }
    }
  }
  // Axes helper
  axesHelper->draw(view, projection,
                   static_cast<float>(screenWidth) / static_cast<float>(screenHeight));

  glUseProgram(0);
}

void Renderer::resize(int width, int height) {
  screenWidth = width;
  screenHeight = height;
}
