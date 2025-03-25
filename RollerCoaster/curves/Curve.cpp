#include "Curve.h"
#include <iostream>

// Pre Calculate M * P for the formula of C(u) = T * M * P
Curve::Curve(const Eigen::Vector3f& a, const Eigen::Vector3f& b,
      const Eigen::Vector3f& c, const Eigen::Vector3f& d,
      Eigen::Matrix4f& M) : length(0) {
  Eigen::Matrix<float, 4, 3> P;
  P <<
    a.transpose(),
    b.transpose(),
    c.transpose(),
    d.transpose();
  mMP = M * P;
}

// Calculate C(u) = T * M * P
Eigen::Vector3f Curve::getPosition(float u) {
  Eigen::Matrix<float, 1, 4> matU;
  matU << u * u * u, u * u, u, 1.0f;
  Eigen::Vector3f ret = (matU * mMP).transpose();
  return ret;
}

// Calculate the unit tangent vector at u
Eigen::Vector3f Curve::getTangent(float u) {
  Eigen::Matrix<float, 1, 4> matU;
  matU << 3.0f * u * u, 2.0f * u, 1.0f, 0.0f;
  Eigen::Vector3f ret = (matU * mMP).transpose();
  return ret;
}

// Calculate the unit tangent prime vector at u
Eigen::Vector3f Curve::getTangentPrime(float u) {
	Eigen::Matrix<float, 1, 4> matU;
	matU << 6.0f * u, 2.0f, 0.0f, 0.0f;
	Eigen::Vector3f ret = (matU * mMP).transpose();
	return ret;
}

/*
  TODO: Implement the function to compute the curvature.
*/
float Curve::getCurvature(float u)
{
	// Use the formula for curvature: k = |T'(u) x T''(u)| / |T'(u)|^3
	Eigen::Vector3f Tprime = getTangentPrime(u);
	Eigen::Vector3f T = getTangent(u);
	float k = (Tprime.cross(T)).norm() / pow(T.norm(), 3);
	return k;
}

// Calculate arc-length
void Curve::calculateFeatures(int segments) {
  length = 0.0f;
  Eigen::Vector3f p0 = getPosition(0.0f);
  for (int i = 1; i < segments; i++) {
    float u = (float)i / (float)segments;
    Eigen::Vector3f p1 = getPosition(u);
    length += (p1 - p0).norm();
    p0 = p1;
  }
}

// Aquire points per length
void Curve::getPoints(float segLen, std::vector<Eigen::Vector3f>& points)
{
    int numSeg = (int)(length / segLen);
    for (int i = 0; i < numSeg; i++) {
        float u = (float)i / (float)numSeg;
        points.push_back(getPosition(u));
    }
}

// Aquire tangent per length
void Curve::getTangents(float segLen, std::vector<Eigen::Vector3f>& tangents)
{
    int numSeg = (int)(length / segLen);
    for (int i = 0; i < numSeg; i++) {
        float u = (float)i / (float)numSeg;
        tangents.push_back(getTangent(u));
    }
}

// Aquire curvatures per length
void Curve::getCurvatures(float segLen, std::vector<float>& curvatures)
{
    int numSeg = (int)(length / segLen);
    for (int i = 0; i < numSeg; i++) {
        float u = (float)i / (float)numSeg;
        curvatures.push_back(getCurvature(u));
    }
}