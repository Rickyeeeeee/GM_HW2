#pragma once

#include <Eigen/Dense>
#include <vector>

class Curve
{
private:
  Eigen::Matrix<float, 4, 3> mMP;
  float length;

public:
  Curve(
    const Eigen::Vector3f& a,
    const Eigen::Vector3f& b,
    const Eigen::Vector3f& c,
    const Eigen::Vector3f& d,
    Eigen::Matrix4f& M
  );

  Eigen::Vector3f getPosition(float u);
  Eigen::Vector3f getTangent(float u);
  Eigen::Vector3f getTangentPrime(float u);
  float getCurvature(float u);
  void calculateFeatures(int segments = 100);

  void getPoints(float segLen, std::vector<Eigen::Vector3f>& points);
  void getTangents(float segLen, std::vector<Eigen::Vector3f>& tangent);
  void getCurvatures(float segLen, std::vector<float>& curvatures);
  inline Eigen::Matrix<float, 4, 3> getMP() const { return mMP; }
  inline float getLength() const { return length; }

};
