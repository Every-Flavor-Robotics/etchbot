
#ifndef PLANNER_H
#define PLANNER_H

namespace Planner
{

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
  float end_delay_us = 0;
};

struct BacklashCompensatedTrajectoryParameters
{
  // Trajectory parameters for the main profile
  //   These should contain the parameters directly from the gcode command,
  //   not the current state of the system
  TrapezoidTrajectoryParameters main_profile;

  bool backlash_compensation_enabled = false;

  //   Previous direction of motion in the x and y directions
  //   Forward is defined as up and right
  Direction left_right_direction_previous = BACKWARD;
  Direction up_down_direction_previous = BACKWARD;

  // This is the total distance that the motor must move to compensate for
  // backlash
  float left_right_backlash_distance;
  float up_down_backlash_distance;

  //   Current state of the robot
  //   These should contain the current state of the system
  //   Not the parameters from the gcode command
  float x_current;
  float y_current;
  float v_current;

  // The current backlash offset in the x and y directions
  float left_right_backlash_offset;
  float up_down_backlash_offset;

  float v_target_backlash;
  float a_target_backlash;
};

// Define a struct to hold the parameters of a trapezoidal profile
struct TrapezoidVelocityTrajectory
{
  // Start time of the profile
  unsigned long start_time_us = 0;
  //   Time from start for constant acceleration
  unsigned long acceleration_time_delta_us = 0;
  //   Time from start for constant velocity
  unsigned long coast_end_time_delta_us = 0;
  //   Time from start for constant deceleration
  unsigned long end_time_delta_us = 0;
  //   Additional time to wait after the profile is complete
  unsigned long end_delay_delta_us = 0;

  float dx = 0;
  float dy = 0;

  float angle;
  float v_initial;
  float v_target;
  float v_final;

  float a_target;
  float a_backlash_target;

  Direction left_right_direction;
  Direction up_down_direction;
};

struct BacklashCompensatedTrajectory
{
  TrapezoidVelocityTrajectory profile;
  TrapezoidVelocityTrajectory backlash_compensation_profile;

  Direction left_right_direction;
  Direction up_down_direction;
};

// Define a struct to hold the state of a trajectory
struct TrajectoryState
{
  bool backlash_compensation_phase = false;

  VelocityVector v;
  AccelerationVector a;
  bool is_complete = false;
};

TrapezoidVelocityTrajectory generate_trapezoid_profile(
    TrapezoidTrajectoryParameters args, float error_tolerance);

BacklashCompensatedTrajectory generate_backlash_compensated_profile(
    BacklashCompensatedTrajectoryParameters args, float error_tolerance);

TrajectoryState compute_trapezoid_velocity_vector(
    TrapezoidVelocityTrajectory& profile, unsigned long time_us,
    float OL_THRESHOLD);

TrajectoryState compute_backlash_compesated_trapezoid_velocity_vector(
    BacklashCompensatedTrajectory& profile, unsigned long time_us);

}  // namespace Planner
#endif  // PLANNER_H
