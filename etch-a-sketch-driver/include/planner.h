
#ifndef PLANNER_H
#define PLANNER_H

namespace Planner
{

// Define a struct to hold a 2D position vector
struct PositionVector
{
  float x;
  float y;
};

struct VelocityVector
{
  float x;
  float y;
};

struct AccelerationVector
{
  float x;
  float y;
};

enum Direction
{
  FORWARD,
  BACKWARD
};
// Define a struct to hold the parameters of a trapezoidal profile
// These are the constraints used to plan the trajectory
struct TrapezoidTrajectoryParameters
{
  float x_initial;
  float y_initial;
  float v_initial;
  float x_final;
  float y_final;
  float v_final;
  float v_target;
  float a_target;

  // Backlash compensation parameters
  bool backlash_compensation = false;
  float left_right_backlash_compensation_distance = 0;
  float up_down_backlash_compensation_distance = 0;
  float backlash_compensation_velocity = 0;
  //   Previous direction of motion in the x and y directions
  //   Forward is defined as up and right
  Direction previous_left_right_direction = BACKWARD;
  Direction previous_up_down_direction = BACKWARD;
};

// Define a struct to hold the parameters of a trapezoidal profile
struct TrapezoidVelocityTrajectory
{
  // Start time of the profile
  unsigned long start_time_us = 0;
  // Time from start for backlash compensation
  unsigned long backlash_compensation_time_delta_us = 0;
  // Time from start for backlash pause - zero velocity to reset stiction
  unsigned long backlash_pause_time_delta_us = 0;
  //   Time from start for constant acceleration
  unsigned long acceleration_time_delta_us = 0;
  //   Time from start for constant velocity
  unsigned long coast_end_time_delta_us = 0;
  //   Time from start for constant deceleration
  unsigned long end_time_delta_us = 0;

  float dx = 0;
  float dy = 0;

  float angle;
  float v_initial;
  float v_target;
  float v_final;

  float x_initial;
  float y_initial;
  float v_backlash_left_right;
  float v_backlash_up_down;

  float a_target;

  Direction left_right_direction;
  Direction up_down_direction;
};

// Define a struct to hold the state of a trajectory
struct TrajectoryState
{
  bool backlash_compensation_phase = false;
  //   Whether or not we should use OL positon
  bool left_right_ol = false;
  bool up_down_ol = false;
  PositionVector p;
  VelocityVector v;
  AccelerationVector a;
  bool is_complete = false;
};

TrapezoidVelocityTrajectory generate_trapezoid_profile(
    TrapezoidTrajectoryParameters args, float error_tolerance);

TrajectoryState compute_trapezoid_velocity_vector(
    TrapezoidVelocityTrajectory& profile, unsigned long time_us,
    float OL_THRESHOLD);

}  // namespace Planner
#endif  // PLANNER_H
