#include <squirrel_8dof_planner/squirrel_8dof_planner_structures.h>

namespace SquirrelMotionPlanner
{

Cell2D::Cell2D()
{
}

Cell2D::Cell2D(const UInt &x, const UInt &y) :
    x(x), y(y)
{
}

bool Cell2D::operator==(const Cell2D &cell) const
{
  return x == cell.x && y == cell.y;
}

Tuple2D::Tuple2D()
{
}

Tuple2D::Tuple2D(Real x, Real y) :
    x(x), y(y)
{
}

Tuple2D Tuple2D::operator+(const Tuple2D &point) const
{
  return Tuple2D(x + point.x, y + point.y);
}

void Tuple2D::operator+=(const Tuple2D &point)
{
  x += point.x;
  y += point.y;
}

Tuple2D Tuple2D::operator-(const Tuple2D &point) const
{
  return Tuple2D(x - point.x, y - point.y);
}

Tuple2D Tuple2D::operator*(const Real &factor) const
{
  return Tuple2D(x * factor, y * factor);
}

void Tuple2D::operator*=(const Real &factor)
{
  x *= factor;
  y *= factor;
}

void Tuple2D::operator/=(const Real &divisor)
{
  x /= divisor;
  y /= divisor;
}

Tuple2D Tuple2D::operator/(const Real &divisor) const
{
  return Tuple2D(x / divisor, y / divisor);
}

Real Tuple2D::distance(const Tuple2D &point) const
{
  return sqrt((x - point.x) * (x - point.x) + (y - point.y) * (y - point.y));
}

LineSegment2D::LineSegment2D()
{

}

LineSegment2D::LineSegment2D(const Tuple2D &start, const Tuple2D &end) :
    start(start), end(end)
{
  length = end.distance(start);
  if (length < TINY_FLT)
  {
    length = 0.0;
    direction = Tuple2D(0.0, 0.0);
    angle = 0.0;
  }
  else
  {
    direction = (end - start) / length;
    angle = atan2(direction.y, direction.x);
  }
}

void LineSegment2D::setPoints(const Tuple2D &start, const Tuple2D &end)
{
  this->start = start;
  this->end = end;
  length = end.distance(start);
  if (length < TINY_FLT)
  {
    length = 0.0;
    direction = Tuple2D(0.0, 0.0);
  }
  else
    direction = (end - start) / length;
}

void LineSegment2D::clipStart(const Real &distance)
{
  if (distance >= length)
  {
    start = end;
    length = 0.0;
  }
  else
  {
    start = start + direction * distance;
    length -= distance;
  }
}

void LineSegment2D::clipEnd(const Real &distance)
{
  if (distance >= length)
  {
    end = start;
    length = 0.0;
  }
  else
  {
    end = end - direction * distance;
    length -= distance;
  }
}

void LineSegment2D::clipBoth(const Real &distance)
{
  if (distance * 2 >= length)
  {
    start = end = start + direction * 0.5 * length;
    length = 0.0;
  }
  else
  {
    start = start + direction * distance;
    end = end - direction * distance;
    length -= 2 * distance;
  }
}

Tuple2D LineSegment2D::getPointAbsolute(const Real &distanceAbsolute) const
{
  return start + direction * distanceAbsolute;
}

Tuple2D LineSegment2D::getPointRelative(const Real &distanceRelative) const
{
  return start + direction * distanceRelative * length;
}

Tuple2D LineSegment2D::getPointMiddle() const
{
  return getPointRelative(0.5);
}

ParametricFunctionCubic2D::ParametricFunctionCubic2D()
{
  xA = xB = xC = xD = yA = yB = yC = yD = length = 0;
  lengthRecip = std::numeric_limits<Real>::infinity();
}

ParametricFunctionCubic2D::ParametricFunctionCubic2D(const Tuple2D &pointStart, const Tuple2D &pointEnd, const Tuple2D &pointRef, const Real &smoothingFactor,
                                                     const UInt &lengthDiscretization)
{
  xA = (pointStart.x - pointEnd.x) * (2 - smoothingFactor);
  xB = 3 * (pointEnd.x - pointStart.x) + smoothingFactor * (2 * pointStart.x - pointEnd.x - pointRef.x);
  xC = smoothingFactor * (pointRef.x - pointStart.x);
  xD = pointStart.x;
  yA = (pointStart.y - pointEnd.y) * (2 - smoothingFactor);
  yB = 3 * (pointEnd.y - pointStart.y) + smoothingFactor * (2 * pointStart.y - pointEnd.y - pointRef.y);
  yC = smoothingFactor * (pointRef.y - pointStart.y);
  yD = pointStart.y;

  length = 0.0;
  const Real stepSize = 1.0 / lengthDiscretization;
  Real t = 0.0;
  for (UInt i = 0; i < lengthDiscretization; ++i, t += stepSize)
  {
    const Tuple2D point1 = getPointParametric(t);
    const Tuple2D point2 = getPointParametric(t + stepSize);
    const Real dx = point1.x - point2.x;
    const Real dy = point1.y - point2.y;
    length += sqrt(dx * dx + dy * dy);
  }

  if (length > 0)
    lengthRecip = 1 / length;
  else
    lengthRecip = std::numeric_limits<Real>::infinity();
}

Tuple2D ParametricFunctionCubic2D::getPointParametric(const Real &t) const
{
  return Tuple2D(xA * t * t * t + xB * t * t + xC * t + xD, yA * t * t * t + yB * t * t + yC * t + yD);
}

Tuple2D ParametricFunctionCubic2D::getPointAbsolute(const Real &distance) const
{
  const Real t = distance * lengthRecip;
  return Tuple2D(xA * t * t * t + xB * t * t + xC * t + xD, yA * t * t * t + yB * t * t + yC * t + yD);
}

Tuple2D ParametricFunctionCubic2D::getDirectionParametric(const Real &t) const
{
  return Tuple2D(3 * xA * t * t + 2 * xB * t + xC, 3 * yA * t * t + 2 * yB * t + yC);
}

Tuple2D ParametricFunctionCubic2D::getDirectionAbsolute(const Real &distance) const
{
  const Real t = distance * lengthRecip;
  return Tuple2D(3 * xA * t * t + 2 * xB * t + xC, 3 * yA * t * t + 2 * yB * t + yC);
}

Real ParametricFunctionCubic2D::getAngleAbsolute(const Real &distance) const
{
  const Tuple2D direction = getDirectionAbsolute(distance);
  return atan2(direction.y, direction.x);
}

} //namespace SquirrelMotionPlanner

