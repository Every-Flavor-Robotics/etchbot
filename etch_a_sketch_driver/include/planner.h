
#ifndef PLANNER_H
#define PLANNER_H

namespace Planner
{
struct VelocityVector
{
  float x;
  float y;
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
};

// Define a struct to hold the parameters of a trapezoidal profile
struct TrapezoidVelocityTrajectory
{
  // Start time of the profile
  unsigned long start_time_us = 0;
  //   Time from start for constant acceleration
  unsigned long acceleration_time_delta_us;
  //   Time from start for constant velocity
  unsigned long coast_end_time_delta_us;
  //   Time from start for constant deceleration
  unsigned long end_time_delta_us;

  float angle;
  float v_initial;
  float v_target;
  float v_final;

  float a_target;
};

// Define a struct to hold the state of a trajectory
struct TrajectoryState
{
  VelocityVector v;
  bool is_complete = false;
};

TrapezoidVelocityTrajectory generate_trapezoid_profile(
    TrapezoidTrajectoryParameters args);

TrajectoryState compute_trapezoid_velocity_vector(
    TrapezoidVelocityTrajectory& profile, unsigned long time_us);

}  // namespace Planner
#endif  // PLANNER_H
