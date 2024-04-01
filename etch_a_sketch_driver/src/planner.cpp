#include "planner.h"

#include <Arduino.h>

Planner::TrapezoidVelocityTrajectory Planner::generate_trapezoid_profile(
    Planner::TrapezoidTrajectoryParameters args)
{
  // Calculate the distance between the initial and final points
  float dx = args.x_final - args.x_initial;
  float dy = args.y_final - args.y_initial;
  float d_total = sqrt(dx * dx + dy * dy);

  // Compute distance over constant acceleration/deceleration region
  // x_f = (v_f^2 - v_i^2) / (2 * a)
  float d_accel =
      0.5 * (args.v_target * args.v_target - args.v_initial * args.v_initial) /
      args.a_target;

  float d_decel =
      0.5 * (args.v_final * args.v_final - args.v_target * args.v_target) /
      -args.a_target;

  // Confirm that the distance is long enough to reach the target velocity
  if (d_total < d_accel + d_decel)
  {
    Serial.println("Distance is too short to reach target velocity");
    // Compute a new target velocity that can be reached
    // v_f = sqrt(2 * a * x_f + v_i^2)
    args.v_target =
        sqrt(2 * args.a_target * d_total + args.v_initial * args.v_initial +
             args.v_final * args.v_final) /
        2;

    // Recompute the distance over the constant acceleration/deceleration region
    d_accel =
        0.5 *
        (args.v_target * args.v_target - args.v_initial * args.v_initial) /
        args.a_target;

    d_decel = 0.5 *
              (args.v_final * args.v_final - args.v_target * args.v_target) /
              -args.a_target;
  }

  Planner::TrapezoidVelocityTrajectory profile;

  //   Compute the time to reach the target velocity
  // v_f = v_i + a * t
  float t_accel = (args.v_target - args.v_initial) / args.a_target;
  float t_decel = (args.v_final - args.v_target) / -args.a_target;

  float t_coast = (d_total - d_accel - d_decel) / args.v_target;

  profile.acceleration_time_delta_us = (long)t_accel * 1e6;
  profile.coast_end_time_delta_us =
      profile.acceleration_time_delta_us + (long)t_coast * 1e6;
  profile.end_time_delta_us =
      profile.coast_end_time_delta_us + (long)t_decel * 1e6;

  profile.v_initial = args.v_initial;
  profile.v_target = args.v_target;
  profile.v_final = args.v_final;

  profile.a_target = args.a_target;

  profile.angle = atan2(dy, dx);

  // Debug
  //   Print all the parameters
  //   Serial.println("Acceleration time: " + String(t_accel));
  //   Serial.println("Coast time: " + String(t_coast));
  //   Serial.println("Deceleration time: " + String(t_decel));
  //   Serial.println("Acceleration distance: " + String(d_accel));
  //   Serial.println("Coast distance: " + String(d_total - d_accel - d_decel));
  //   Serial.println("Deceleration distance: " + String(d_decel));
  //   Serial.println("Total distance: " + String(d_total));
  //   Serial.println("Initial velocity: " + String(args.v_initial));
  //   Serial.println("Target velocity: " + String(args.v_target));
  //   Serial.println("Final velocity: " + String(args.v_final));
  //   Serial.println("Target angle: " + String(profile.angle));
  //   Serial.println("Acceleration time delta: " +
  //                  String(profile.acceleration_time_delta_us));
  //   Serial.println("Coast end time delta: " +
  //                  String(profile.coast_end_time_delta_us));
  //   Serial.println("End time delta: " + String(profile.end_time_delta_us));

  return profile;
}

Planner::TrajectoryState Planner::compute_trapezoid_velocity_vector(
    Planner::TrapezoidVelocityTrajectory& profile, long time_us)
{
  TrajectoryState state;
  float v_target;
  //   Compute t relative to the start time
  float t_us = time_us - profile.start_time_us;
  float t = t_us / 1e6;

  // Trajectory has not started, set the start time and initial velocity
  if (profile.start_time_us == 0)
  {
    Serial.println("Starting profile");
    profile.start_time_us = time_us;
    v_target = profile.v_initial;
  }

  // Constant acceleration phase
  else if (t_us < profile.acceleration_time_delta_us)
  {
    //   Acceleration phase
    v_target = profile.v_initial + profile.a_target * t;
  }
  //   Constant velocity phase
  else if (t_us < profile.coast_end_time_delta_us)
  {
    //   Constant velocity phase
    v_target = profile.v_target;
  }
  //   Deceleration phase
  else if (t_us < profile.end_time_delta_us)
  {
    //   Deceleration phase
    v_target = profile.v_target -
               profile.a_target * (t - profile.coast_end_time_delta_us / 1e6);
  }
  else
  {
    //   Profile is complete
    v_target = profile.v_final;
    state.is_complete = true;
  }

  state.v.x = v_target * cos(profile.angle);
  state.v.y = v_target * sin(profile.angle);
  return state;
}