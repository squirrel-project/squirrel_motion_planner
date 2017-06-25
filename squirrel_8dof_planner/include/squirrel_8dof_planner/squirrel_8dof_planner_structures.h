#ifndef SQUIRREL_8DOF_PLANNER_STRUCTURES_H_
#define SQUIRREL_8DOF_PLANNER_STRUCTURES_H_

#include <ros/ros.h>

#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <stdint.h>

namespace SquirrelMotionPlanner
{

#define TINY_FLT 0.000001     ///< Definition of a small floating point number.

typedef int32_t Int;     ///< 32-bit signed integer definition.
typedef uint32_t UInt;     ///< 32-bit unsigned integer definition.
typedef double Real;     ///< 64-bit floating point definition.

/**
 * @brief A 2D cell structure with an x and y coordinate.
 * Used for representing positive integer of a 2D cell, e.g., indices of a 2D array.
 */
struct Cell2D
{
  UInt x;     ///< x coordinate of the 2D cell.
  UInt y;     ///< y coordinate of the 2D cell.

  /**
   * @brief Constructor, leaves x and y uninitialized.
   */
  Cell2D();

  /**
   * @brief Constructor, initializes x and y with the provided values.
   */
  Cell2D(const UInt &x, const UInt &y);

  /**
   * @brief Equal to operator, compares x and y with the x and y values of cell.
   * @param cell The cell to which the current x and y coordinates are to be compared.
   * @return Returns false if either x or y differ from the current values.
   */
  bool operator==(const Cell2D &cell) const;
};

/**
 * @brief A 2D tuple structure with an x and y value.
 * Used for representing a 2D floating point tuple, e.g., a metric 2D coordinate.
 */
struct Tuple2D
{
  Real x;     ///< x value of the 2D tuple.
  Real y;     ///< y value of the 2D tuple.

  /**
   * @brief Constructor, leaves x and y uninitialized.
   */
  Tuple2D();

  /**
   * @brief Constructor, initializes x and y with the provided values.
   */
  Tuple2D(Real x, Real y);

  /**
   * @brief Addition operator, that returns point with the added x and y values.
   * @param point Tuple2D to be added to the current x and y values.
   * @return Returns a Tuple2D with the added x and y values.
   */
  Tuple2D operator+(const Tuple2D &point) const;

  /**
   * @brief Addition operator, that adds a point to the current x and y values.
   * @param point Tuple2D to be added to the current x and y values.
   */
  void operator+=(const Tuple2D &point);

  /**
   * @brief Subtraction operator, that returns point with the subtracted x and y values.
   * @param point Tuple2D to be subtracted from the current x and y values.
   * @return Returns a Tuple2D with the subtracted x and y values.
   */
  Tuple2D operator-(const Tuple2D &point) const;

  /**
   * @brief Multiplication operator, that returns point with the multiplied x and y values.
   * @param factor A floating point value to be multiplied to the current x and y values.
   * @return Returns a Tuple2D with the multiplied x and y values.
   */
  Tuple2D operator*(const Real &factor) const;

  /**
   * @brief Multiplication operator, that multiplies the current x and y values by a given value.
   * @param factor A floating point value to be multiplied to the current x and y values.
   */
  void operator*=(const Real &factor);

  /**
   * @brief Division operator, that devides the current x and y values by a given value.
   * @param divisor A floating point value, that devides the current x and y values by a given value.
   */
  void operator/=(const Real &divisor);

  /**
   * @brief Division operator, that returns a Tuple2D with divided x and y values.
   * @param divisor A floating point value, that devides the current x and y values by a given value.
   * @return Returns a Tuple2D with the divided x and y values.
   */
  Tuple2D operator/(const Real &divisor) const;

  /**
   * @brief Finds the Euclidean distance to a given point.
   * @param point A Tuple2D to which the distance is found.
   * @return Returns a floating point value of the Euclidean distance to the given point.
   */
  Real distance(const Tuple2D &point) const;
};

/**
 * @brief A 2D line segment with a start and end point and a normalized direction.
 */
struct LineSegment2D
{
  Tuple2D start;     ///< Start point of the line segment.
  Tuple2D end;     ///< End point of the line segment.
  Tuple2D direction;     ///< Normalized direction of the line segment, pointing from start to end.

  Real length;     ///< Euclidean distance between start and end.
  Real angle;     ///< The angle to the x-axis, given in radians from -pi to pi.

  /**
   * @brief Constructor that leaves all members un-initialized.
   */
  LineSegment2D();

  /**
   * @brief Constructor that initializes all members.
   * @param start The start point of the line segment.
   * @param end The end point of the line segment.
   */
  LineSegment2D(const Tuple2D &start, const Tuple2D &end);

  /**
   * @brief Updates the start and end point of the line segment and recomputes all other members accordingly.
   * @param The start point of the line segment.
   * @param The end point of the line segment.
   */
  void setPoints(const Tuple2D &start, const Tuple2D &end);

  /**
   * @brief Clips the line segment from the start by a given distance and updates all other members accordingly.
   * If the distance is longer than the actual line segment, both start and end lie at the former end point.
   * @param distance The distance by which the line segment is to be shortened.
   */
  void clipStart(const Real &distance);

  /**
   * @brief Clips the line segment from the end by a given distance and updates all other members accordingly.
   * If the distance is longer than the actual line segment, both start and end lie at the former start point.
   * @param distance The distance by which the line segment is to be shortened.
   */
  void clipEnd(const Real &distance);

  /**
   * @brief Clips the line segment from both sides by a given distance and updates all other members accordingly.
   * If the distance is longer than the actual line segment, both start and end lie at the former mid point.
   * @param distance The distance by which the line segment is to be shortened on both sides.
   */
  void clipBoth(const Real &distance);

  /**
   * @brief Finds a point along the line segment a given absolute distance from the start.
   * @param distanceAbsolute The absolute distance from the start.
   * @return Returns a Tuple2D with the coordinates of the point that is the given absolute distance from the start.
   */
  Tuple2D getPointAbsolute(const Real &distanceAbsolute) const;

  /**
   * @brief Finds a point along the line segment a given relative distance between the start and end, i.e., 0 for start and 1 for end.
   * @param distanceAbsolute The relative distance from the start.
   * @return Returns a Tuple2D with the coordinates of the point that is the given relative distance from the start.
   */
  Tuple2D getPointRelative(const Real &distanceRelative) const;

  /**
   * @brief Finds the mid point along the line segment.
   * @return Returns a Tuple2D with the coordinates of the point that lies in the middle between start and end.
   */
  Tuple2D getPointMiddle() const;
};

/**
 * @brief Defines a 2D parametric cubic polynomial function.
 * The function is dependent on a parameter t, such that a start point lies at t = 0.0 and the end point at t = 1.0.
 */
struct ParametricFunctionCubic2D
{
  Real xA;     ///< Coefficient of t^3 in the x-coordinate.
  Real xB;     ///< Coefficient of t^2 in the x-coordinate.
  Real xC;     ///< Coefficient of t^1 in the x-coordinate.
  Real xD;     ///< Coefficient of t^0 in the x-coordinate.
  Real yA;     ///< Coefficient of t^3 in the y-coordinate.
  Real yB;     ///< Coefficient of t^2 in the y-coordinate.
  Real yC;     ///< Coefficient of t^1 in the y-coordinate.
  Real yD;     ///< Coefficient of t^0 in the y-coordinate.
  Real length;     ///< Approximate arc length for parameter t in the interval [0:1], interpolated between points on the function.
  Real lengthRecip;     ///< 1 / length, used for calculating absolute points and directions.

  /**
   * @brief Constructor. Sets all coefficients to 0.
   */
  ParametricFunctionCubic2D();

  /**
   * @brief Constructor. Determines all coefficients of the function, defined by a start and end point and an intermediate reference point.
   * @param pointStart The start point of the function, returned by t = 0.0.
   * @param pointEnd The end point of the function, returned by t = 1.0.
   * @param pointRef The intermediate reference point that is used to determine the outpoing and incoming directions at pointStart and pointEnd.
   * @param smoothingFactor Determines the smoothing value between pointStart and pointEnd.
   * Low values (< 1.0) correspond to little smoothing, large values (> 3.0) to large smoothing. Ideal values lie between 1.5 and 2.5.
   * Too large values can lead to unwanted loops in the interval t = [0:1].
   * @param lengthDiscretization The discretization of the interval t = [0:1] which is used during the computation of length.
   */
  ParametricFunctionCubic2D(const Tuple2D &pointStart, const Tuple2D &pointEnd, const Tuple2D &pointRef, const Real &smoothingFactor,
                            const UInt &lengthDiscretization);

  /**
   * @brief Finds a point along the 2D function for a given parameter t.
   * @param t Parameter value.
   * @return Returns a 2D point of the function for a given parameter t.
   */
  Tuple2D getPointParametric(const Real &t) const;

  /**
   * @brief Finds a point along the function for a given absolute distance from the initialized start point.
   * @param distance The distance from the start.
   * @return Returns a 2D point of the function a given distance along the function from the start point.
   */
  Tuple2D getPointAbsolute(const Real &distance) const;

  /**
   * @brief Finds the first 2D derivate along the function for a given parameter t.
   * @param t Parameter value.
   * @return Returns a 2D vector of the first 2D derivate of the function for a given parameter t.
   */
  Tuple2D getDirectionParametric(const Real &t) const;

  /**
   * @brief Finds the first 2D derivate along the function for a given absolute distance from the initialized start point.
   * @param distance The distance from the start.
   * @return Returns a 2D vector of the first 2D derivate of the function a given distance along the function from the start point.
   */
  Tuple2D getDirectionAbsolute(const Real &distance) const;

  /**
   * @brief Finds the angle between the direction of the function and the x-axis at a given absolute distance from the initialized start point.
   * @param distance The distance from the start.
   * @return Returns the angle in radians between the direction of the function and the x-axis.
   */
  Real getAngleAbsolute(const Real &distance) const;
};

/**
 * @brief A* node structure, used during a grid based A* search.
 */
struct AStarNode
{
  Real g;     ///< Cost that is necessary to reach the goal from the starting node.
  Real f;     ///< Sum of cost to reach the node and an admissable heuristic.
  Cell2D cell;     ///< 2D coordinates to the corresponding grid cell.
  Cell2D cellParent;     ///< 2D coordinate of the corresponding parent cell.
  bool open;     ///< Indicates if the node is currently in the open list.
  bool closed;     ///< Indicates if the node has already been expanded and is closed.
  bool occupied;     ///< Indicates if the node is occupied by an obstacle.
  UInt indexOpenList;     ///< The current index the node has in the priority queue.

  std::vector<std::pair<AStarNode*, Real> > neighbors;     ///< Vector of pointers to neighboring nodes along with the cost to reach the corresponding nodes.
};

/**
 * @brief A* path structure that contains a list of cells from start to goal in addition to corresponding metric 2D points.
 */
struct AStarPath2D
{
  std::vector<Cell2D> cells;     ///< Sorted vector of grid cell corrdinates of the path from the start to goal cell.
  std::vector<Tuple2D> points;     ///< Sorted vector of metric points that correspond to cells.
  bool valid;     ///< Indicates if the current path is valid or has been invalidated due to an impossible path or a map update.
};

} //namespace SquirrelMotionPlanner

#endif /* SQUIRREL_8DOF_PLANNER_STRUCTURES_H_ */
