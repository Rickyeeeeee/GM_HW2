#include "Spline.h"

/******************************************************************************
Build the spline with the given control points
******************************************************************************/
void Spline::build()
{
    m_curves.clear();
    preLength.clear();
    preLength.push_back(0.0f);
    arcLength = 0.0f;
}

/******************************************************************************
Add a control point
******************************************************************************/
void Spline::addPoint() {
  if (m_points.empty()) {
      addPoint(Eigen::Vector3f(0, 0, 0));
  }
  else {
    Eigen::Vector3f p = m_points[m_points.size() - 1] + Eigen::Vector3f(1, 0, 0);
    addPoint(p);
  }
}

/******************************************************************************
Add a control point at p

Entry:
 p - the position of the control point
******************************************************************************/
void Spline::addPoint(Eigen::Vector3f& p)
{
    m_points.push_back(p);
    build();
}

/******************************************************************************
Remove the last control point
******************************************************************************/
void Spline::removePoint() {
  if (selectedIdx < 0 || selectedIdx >= m_points.size())
    return;
  m_points.erase(m_points.begin() + selectedIdx);
  selectedIdx = -1;
  build();
}

/******************************************************************************
Add a polynoimal curve to this splines

Entry:
  a - the first vector (e.g., p0)
  b - the second vector (e.g., p1)
  c - the thrid vector (e.g., p2)
  d - the forth vector (e.g., p3)
  M - the matrix to compute coefficients 
******************************************************************************/
void Spline::addCurve(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& d, Eigen::Matrix4f& M)
{
    m_curves.emplace_back(a, b, c, d, M);
    m_curves.back().calculateFeatures();
    arcLength += m_curves.back().getLength();
    preLength.push_back(arcLength);
}

/******************************************************************************
Reparameterize t to (i, u)
******************************************************************************/
std::pair<int, float> Spline::parameterize(float t)
{
    float total = (float)m_curves.size();
    t = std::fmod(t, total);
    int idx = std::floorf(t);
    float u = t - (float)idx;
    return std::pair<int, double>(idx, u);
}

/******************************************************************************
Reparameterize t to (i, u) such that the curve is unit speed
******************************************************************************/
std::pair<int, float> Spline::parameterizeUnitSpeed(float s)
{
    /*
    TODO: Implement the function to reparameterize.
    Hint: We store the arc-length of each curve in preLength (s0-s1-s2-...-sn)
          Therefore, you can search s is in which interval of perLength and interpolate u
    */
	s = std::fmod(s, (float)m_curves.size());
    s /= (float)m_curves.size();
    s *= arcLength;
	if (m_curves.size() == 0) return std::pair<int, float>(0, 0.0f);
	for (int i = 0; i < preLength.size() - 1; i++) {
		if (s >= preLength[i] && s <= preLength[i + 1]) {
			float u = (s - preLength[i]) / (preLength[i + 1] - preLength[i]);
			return std::pair<int, float>(i, u);
		}
	}
    return std::pair<int, float>(0, 0.0f);
}

Eigen::Vector3f Spline::getPosition(float t, bool flagUS) {
    return flagUS ? getPositionS(t) : getPositionU(t);
}

Eigen::Vector3f Spline::getTangent(float t, bool flagUS) {
    return flagUS ? getTangentS(t) : getTangentU(t);
}

double Spline::getCurvature(float t, bool flagUS) {
    return flagUS ? getCurvatureS(t) : getCurvatureU(t);
}

Eigen::Vector3f Spline::getPositionU(float t)
{
    if (m_curves.size() == 0) return Eigen::Vector3f::Zero();
    std::pair<int, double> u = parameterize(t);
    return m_curves[u.first].getPosition(u.second);
}

Eigen::Vector3f Spline::getTangentU(float t)
{
    if (m_curves.size() == 0) return Eigen::Vector3f::Zero();
    std::pair<int, double> u = parameterize(t);
    return m_curves[u.first].getTangent(u.second);
}

double Spline::getCurvatureU(float t)
{
    if (m_curves.size() == 0) return 0.0f;
    std::pair<int, double> u = parameterize(t);
    return m_curves[u.first].getCurvature(u.second);
}

Eigen::Vector3f Spline::getPositionS(float s)
{
    if (m_curves.size() == 0) return Eigen::Vector3f::Zero();
    std::pair<int, double> u = parameterizeUnitSpeed(s);
    return m_curves[u.first].getPosition(u.second);
}

Eigen::Vector3f Spline::getTangentS(float s)
{
    if (m_curves.size() == 0) return Eigen::Vector3f::Zero();
    std::pair<int, double> u = parameterizeUnitSpeed(s);
    return m_curves[u.first].getTangent(u.second);
}

double Spline::getCurvatureS(float s)
{
    if (m_curves.size() == 0) return 0.0;
    std::pair<int, double> u = parameterizeUnitSpeed(s);
    return m_curves[u.first].getCurvature(u.second);
}

Eigen::Vector3f Spline::getSelectedPoint() const
{
    return  isSelectedPoint() ? m_points[selectedIdx] : Eigen::Vector3f::Zero();
}

void Spline::setSelectedPoint(Eigen::Vector3f& p)
{
    m_points[selectedIdx] = p;
    build();
}

bool Spline::isSelectedPoint() const
{
    return !(selectedIdx < 0 || selectedIdx >= m_points.size());
}


void Spline::setAntribute(std::vector<Eigen::Vector3f>& points, bool loop)
{
    m_loop = loop;
    m_points = points;
    build();
}

void Spline::setLoop(bool loop)
{
    m_loop = loop;
    build();
}

void Spline::setPoints(std::vector<Eigen::Vector3f>& points)
{
    m_points = points;
    build();
}
