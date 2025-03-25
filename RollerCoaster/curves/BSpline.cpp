#include "BSpline.h"
#include <algorithm>
#include <cmath>

void BSpline::build() {
  Spline::build();
  if (m_points.size() < 4) return;
  /*
  TODO: Implement the function to build the B-spline.
  Hint:
  1. Use addCurve to create the segments of the B-spline
  2. If m_loop is true, create the close B-spline with m_points
  3. Otherwise, create the open uniform B-spline with m_points
     Note, the open uniform B-spline goes through p0 and pn
  */
  Eigen::Matrix4f matM;
  matM <<   -1.0f, 3.0f, -3.0f, 1.0f,
            3.0f, -6.0f, 3.0f, 0.0f,
            -3.0f, 0.0f, 3.0f, 0.0f,
            1.0f, 4.0f, 1.0f, 0.0f;

  matM /= 6.0f;

  int n = (int)m_points.size();
  if (!m_loop) {
      for (int i = 0; i < n - 1; i++) {
          Eigen::Vector3f p0, p1, p2, p3;
            p1 = m_points[i];
            p2 = m_points[i + 1];
            p0 = i == 0 ? 2*p1-p2 : m_points[(i - 1 + n) % n];
            p3 = i == (n-2) ? 2*p2-p1 : m_points[(i + 2) % n];
          addCurve(p0, p1, p2, p3, matM);
      }
  }
  else {
      for (int i = 0; i < n; i++) {
          Eigen::Vector3f p0, p1, p2, p3;
          p0 = m_points[(i - 1 + n) % n];
          p1 = m_points[i];
          p2 = m_points[(i + 1) % n];
          p3 = m_points[(i + 2) % n];
          addCurve(p0, p1, p2, p3, matM);
      }
  }
}