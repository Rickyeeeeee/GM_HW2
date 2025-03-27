#include "CurveProcessor.h"
#include "BSpline.h"
#include "CatmullRom.h"
#include "Polyline.h"
#include <fstream>

void CurveProcessor::samplePoints(Spline *spline, float segLen,
                                  std::vector<Eigen::Vector3f> &all_points,
                                  std::vector<Eigen::Vector3f> &all_tangents,
                                  std::vector<Eigen::Vector3f> &all_normals,
                                  std::vector<float> &all_curvatures) {
  std::vector<Curve> curves = spline->getCurves();
  Eigen::Vector3f binormal = Eigen::Vector3f::Zero();
  // Iterate for each Curves
  for (int i = 0; i < spline->getNumCurves(); i++) {
    Curve &c = spline->getCurves()[i];
    // Take the features point and tangents from the curves
    std::vector<Eigen::Vector3f> points;
    c.getPoints(segLen, points);
    std::vector<Eigen::Vector3f> tangents;
    c.getTangents(segLen, tangents);
    std::vector<float> curvatures;
    c.getCurvatures(segLen, curvatures);
    // Insert it to vector for spline
    all_points.insert(all_points.end(), points.begin(), points.end());
    all_tangents.insert(all_tangents.end(), tangents.begin(), tangents.end());
    all_curvatures.insert(all_curvatures.end(), curvatures.begin(), curvatures.end());

    //
    if ((spline->getType() != Spline::Type::Polyline) &&
        (spline->getLoop() || i < spline->getNumCurves() - 1)) {
      all_points.pop_back();
      all_tangents.pop_back();
      all_curvatures.pop_back();
    }
    // Calculate the binormal
    for (auto &tangent : tangents) {
      Eigen::Vector3f normal;
      // First initialize the last binormal, I'm afraid the starting tangent will be same as
      // upvector
      if (binormal == Eigen::Vector3f::Zero()) {
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        if (tangent.dot(up) > 0.99f)
          up = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        normal = tangent.cross(up).normalized();
        binormal = normal.cross(tangent).normalized();
      }
      // If it's not the first one, we can safely use the last binormal
      else {
        normal = tangent.cross(binormal).normalized();
        binormal = normal.cross(tangent).normalized();
      }
      // Push it
      all_normals.push_back(normal);
    }
  }
}

void CurveProcessor::sampleCurvature(Spline* spline, float segLen, std::vector<float>& all_curvatures)
{
    std::vector<Curve> curves = spline->getCurves();
    Eigen::Vector3f binormal = Eigen::Vector3f::Zero();
    // Iterate for each Curves
    for (int i = 0; i < spline->getNumCurves(); i++) {
        Curve& c = spline->getCurves()[i];
        // Take the features point and tangents from the curves
        std::vector<Eigen::Vector3f> points;
        c.getPoints(segLen, points);
        std::vector<Eigen::Vector3f> tangents;
        c.getTangents(segLen, tangents);
        std::vector<float> curvatures;
        c.getCurvatures(segLen, curvatures);
        // Insert it to vector for spline
        all_curvatures.insert(all_curvatures.end(), curvatures.begin(), curvatures.end());
        if ((spline->getType() != Spline::Type::Polyline) &&
            (spline->getLoop() || i < spline->getNumCurves() - 1)) {
            all_curvatures.pop_back();
        }
    }
}

/******************************************************************************
Transport the roation from t0 to t1 on u0 and return u1

Entry:
  u0 - vector u at last time step
  t0 - vector t at last time step
  t1 - vector t at current time step

Exit:
  u1 - vector u at current time step
******************************************************************************/
Eigen::Vector3f CurveProcessor::parallelTransport(Eigen::Vector3f &u0, 
                                                  Eigen::Vector3f &t0,
                                                  Eigen::Vector3f &t1) {
    /*
    TODO:Replace the following code
    1. Compute the axis of t0 and t1
    2. Compute the rotation angle phi from t0 to t1
    3. Rotate u0 on the axis with phi
    */
    // Compute the axis of rotation (cross product of t0 and t1)
    Eigen::Vector3f n = t0.cross(t1);

    // Check if t0 and t1 are nearly parallel
    if (n.norm() == 0.0f) {
        return u0; // No rotation needed
    }

    // Compute the rotation angle
    float phi = std::atan2(n.norm(), t0.dot(t1));

    // Normalize the rotation axis
    n.normalize();

    // Apply Rodrigues' rotation formula
    return u0 * std::cos(phi) + (n.cross(u0)) * std::sin(phi) + n * (n.dot(u0)) * (1 - std::cos(phi));
}

void CurveProcessor::saveSpline(Spline *spline) {
  if (!spline)
    return;

  std::vector<Eigen::Vector3f> &points = spline->getPoints();
  std::ofstream outFile("spline.txt");

  if (!outFile) {
    std::cerr << "Error writing file\n";
    return;
  }

  outFile << (int)(spline->getType()) << "\n";
  outFile << (int)(spline->getLoop()) << "\n";
  for (const Eigen::Vector3f &pt : points) {
    outFile << pt.x() << " " << pt.y() << " " << pt.z() << "\n";
  }

  outFile.close();
}

bool CurveProcessor::loadSpline(Model *model) {
  std::ifstream inFile("spline.txt");
  if (!inFile) {
    std::cerr << "Error reading file\n";
    return false;
  }

  int type, loop;
  std::vector<Eigen::Vector3f> points;
  float x, y, z;
  inFile >> type;
  inFile >> loop;
  while (inFile >> x >> y >> z) {
    points.push_back(Eigen::Vector3f(x, y, z));
  }
  inFile.close();
  ////
  switch ((Spline::Type)type) {
  case Spline::Type::Polyline:
    model->setSpline(new Polyline());
    break;
  case Spline::Type::BSpline:
    model->setSpline(new BSpline());
    break;
  }
  model->getSpline()->setAntribute(points, loop);

  return true;
}

bool CurveProcessor::convertSplineType(Model *model, Spline::Type type) {
  Spline *spline = model->getSpline();
  if (spline->getType() == type || type == Spline::Type::Error)
    return false;

  bool loop = spline->getLoop();
  std::vector<Eigen::Vector3f> points = spline->getPoints();

  switch (type) {
  case Spline::Type::Polyline:
    model->setSpline(new Polyline());
    break;
  case Spline::Type::BSpline:
    model->setSpline(new BSpline());
    break;
  case Spline::Type::CatmullRom:
    model->setSpline(new CatmullRom());
    break;
  }

  if (!points.empty()) {
    model->getSpline()->setAntribute(points, loop);
  } else {
    model->getSpline()->setLoop(loop);
  }
}
