#include "CatmullRom.h"
#include <algorithm>
#include <cmath>

void CatmullRom::build() {
    Spline::build();
    if (m_points.size() < 4) return;
    /*
      TODO: Implement the function to build the Catmull-Rom spline.
      Hint:
      1. Use addCurve to create the segments of the B-spline
      2. If m_loop is true, create the close Catmull-Rom spline with m_points
      3. Otherwise, create the open Catmull-Rom spline with m_points
         Note, the open Catmull-Rom spline goes through p0 and pn
    */
    Eigen::Matrix4f matM;
    matM << -1.0f, 3.0f, -3.0f, 1.0f,
            2.0f, -5.0f, 4.0f, -1.0f,
            -1.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 2.0f, 0.0f, 0.0f;

	matM /= 2.0f;

    int n = (int)m_points.size();
    if (!m_loop) {
        for (int i = 0; i < n - 1; i++) {
			Eigen::Vector3f p0, p1, p2, p3;
			if (i == 0) {
                p0 = m_points[i];
                p1 = m_points[i + 1];
				p2 = m_points[i + 2];
                Eigen::Matrix4f matMF;
                matMF <<    1.5f, -1.0f, 0.5f, 0.0f,
                            -1.5f, 1.0f, -0.5f, 0.0f,
					        -1.0f, 1.0f, 0.0f, 0.0f,
					        1.0f, 0.0f, 0.0f, 0.0f;
				addCurve(p0, p1, p2, Eigen::Vector3f::Zero(), matMF);
			}
			else if (i == n - 2) {
				p0 = m_points[i - 1];
				p1 = m_points[i];
				p2 = m_points[i + 1];
				Eigen::Matrix4f matML;
				matML <<    0.5f, 1.0f, -0.5f, 0.0f,
					        -1.0f, -2.0f, 1.0f, 0.0f,
					        0.5f, 0.0f, 0.5f, 0.0f,
					        0.0f, 1.0f, 0.0f, 0.0f;
				addCurve(p0, p1, p2, Eigen::Vector3f::Zero(), matML);
			}
            else {
                p0 = m_points[(i - 1 + n) % n];
                p1 = m_points[i];
			    p2 = m_points[i + 1];
                p3 = m_points[(i + 2) % n];
                addCurve(p0, p1, p2, p3, matM);
            }
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