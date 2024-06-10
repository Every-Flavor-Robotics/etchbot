#include "planner.h"

#include <Arduino.h>

#include <chrono>

Planner::TrapezoidVelocityTrajectory Planner::generate_trapezoid_profile(
    Planner::TrapezoidTrajectoryParameters args, float error_tolerance)
{
  // Calculate the distance between the initial and final points
  float dx = args.x_final - args.x_initial;
  float dy = args.y_final - args.y_initial;
  float d_total = sqrt(dx * dx + dy * dy);

  //   Construct profile
  Planner::TrapezoidVelocityTrajectory profile;

  //   Serial.println("Distance: " + String(d_total, 5) + "rad");

  if (d_total < error_tolerance)
  {
    return Planner::TrapezoidVelocityTrajectory();
  }

  // Cap initial velocity to target velocity
  // When iterative replanning, this is necessary as the planner
  // cannot handle a higher initial velocity than the target velocity
  if (args.v_initial > args.v_target)
  {
    args.v_initial = args.v_target;
  }

  profile.angle = atan2(dy, dx);

  // Velocity mutliplier to ensure that at least one of the components
  // is travelling at the target velocity
  // Other component will be traveling at a lower velocity
  //   float velocity_multipler =
  //       1;  // / max(cos(profile.angle), sin(profile.angle));
  //   args.v_target *= velocity_multipler;

  // If we are too close to the goal and initial velocity is too high,
  // we only need to decelerate
  // x_f = (v_f^2 - v_i^2) / (2 * a)
  float d_only_decel =
      -0.5 * (args.v_final * args.v_final - args.v_initial * args.v_initial) /
      args.a_target;

  float t_accel = 0;
  float t_coast = 0;
  float t_decel = 0;

  // Handle the case where only deceleration is needed
  if (d_total < d_only_decel)
  {
    // Compute new acceleration target to reach the target velocity
    // in the remaining distance
    args.a_target =
        -(args.v_final * args.v_final - args.v_initial * args.v_initial) /
        (2 * d_total);
    // Set target velocity to initial velocity to correctly compute
    // deceleration time.
    args.v_target = args.v_initial;
    // Serial.println("Only deceleration needed");
    t_decel = (args.v_final - args.v_initial) / -args.a_target;
  }
  else
  {
    // Regular operation, compute the trapezoid profile

    // Compute distance over constant acceleration/deceleration region
    // x_f = (v_f^2 - v_i^2) / (2 * a)
    float d_accel =
        0.5 *
        (args.v_target * args.v_target - args.v_initial * args.v_initial) /
        args.a_target;

    float d_decel =
        -0.5 * (args.v_final * args.v_final - args.v_target * args.v_target) /
        args.a_target;

    // Confirm that the distance is long enough to reach the target velocity
    if (d_total < d_accel + d_decel)
    {
      //   Serial.println("Distance is too short to reach target velocity");
      // Compute a new target velocity that can be reached
      // v_t^2 = ad + 0.5 * v_i^2 + 0.5 * v_f^2
      args.v_target =
          sqrt(args.a_target * d_total + 0.5 * args.v_initial * args.v_initial +
               0.5 * args.v_final * args.v_final);

      // Recompute the distance over the constant acceleration/deceleration
      // region
      d_accel =
          0.5 *
          (args.v_target * args.v_target - args.v_initial * args.v_initial) /
          args.a_target;

      d_decel = -0.5 *
                (args.v_final * args.v_final - args.v_target * args.v_target) /
                args.a_target;
    }

    //   Compute the time to reach the target velocity
    // v_f = v_i + a * t
    t_accel = (args.v_target - args.v_initial) / args.a_target;
    t_decel = (args.v_final - args.v_target) / -args.a_target;

    t_coast = (d_total - d_accel - d_decel) / args.v_target;
  }

  profile.acceleration_time_delta_us = (t_accel * 1e6);
  profile.coast_end_time_delta_us =
      profile.acceleration_time_delta_us + (t_coast * 1e6);
  profile.end_time_delta_us = profile.coast_end_time_delta_us + (t_decel * 1e6);

  profile.v_initial = args.v_initial;
  profile.v_target = args.v_target;
  profile.v_final = args.v_final;

  profile.a_target = args.a_target;

  //   Serial.println("-------------- In planner -----------------");
  //   Serial.println("Initial position: (" + String(args.x_initial, 5) + ", " +
  //                  String(args.y_initial, 5) + ")");
  //   Serial.println("Final position: (" + String(args.x_final, 5) + ", " +
  //                  String(args.y_final, 5) + ")");
  //   Serial.println("Initial Total distance: " + String(d_total, 5) + "mm");
  //   Serial.println("Initial velocity: " + String(args.v_initial, 5) +
  //   "mm/s"); Serial.println("Target velocity: " + String(args.v_target, 5) +
  //   "mm/s"); Serial.println("Final velocity: " + String(args.v_final, 5) +
  //   "mm/s"); Serial.println("Acceleration time: " + String(t_accel, 5) +
  //   "s"); Serial.println("Coast time: " + String(t_coast, 5) + " s");
  //   Serial.println("Deceleration time: " + String(t_decel, 5) + " s");

  //   Serial.println("Total time: " + String(t_accel + t_coast + t_decel, 5) +
  //                  " us");

  //   Serial.println("Acceleration distance: " +
  //                  String(0.5 * args.a_target * t_accel * t_accel, 5) +
  //                  "mm");

  //   Serial.println("Coast distance: " + String(t_coast * args.v_target, 5) +
  //                  " mm");

  //   Serial.println("Deceleration distance: " +
  //                  String(0.5 * args.a_target * t_decel * t_decel, 5) +
  //                  "mm");

  //   Serial.println("Angle: " + String(profile.angle, 5) + "rad");

  //   Serial.println("-------------- In planner -----------------");

  return profile;
}

Planner::TrajectoryState Planner::compute_trapezoid_velocity_vector(
    Planner::TrapezoidVelocityTrajectory& profile, unsigned long time_us)
{
  TrajectoryState state;
  float v_target;
  float a_target;
  //   Compute t relative to the start time
  unsigned long t_us = time_us - profile.start_time_us;
  float t = t_us / 1e6;

  // Trajectory has not started, set the start time and initial velocity
  if (profile.start_time_us == 0)
  {
    // Serial.println("Starting profile");
    profile.start_time_us = time_us;
    v_target = profile.v_initial;
    a_target = 0;
  }

  // Constant acceleration phase
  else if (t_us <= profile.acceleration_time_delta_us)
  {
    //   Acceleration phase
    v_target = profile.v_initial + profile.a_target * t;
    a_target = profile.a_target;
  }
  //   Constant velocity phase
  else if (t_us <= profile.coast_end_time_delta_us)
  {
    //   Constant velocity phase
    v_target = profile.v_target;
    a_target = 0;
  }
  //   Deceleration phase
  else if (t_us <= profile.end_time_delta_us)
  {
    //   Deceleration phase
    v_target = profile.v_target -
               profile.a_target * (t - profile.coast_end_time_delta_us / 1e6);

    a_target = -profile.a_target;
  }
  else
  {
    //   Profile is complete
    v_target = profile.v_final;
    state.is_complete = true;
  }

  state.v.x = v_target * cos(profile.angle);
  state.v.y = v_target * sin(profile.angle);
  state.a.x = a_target * cos(profile.angle);
  state.a.y = a_target * sin(profile.angle);

  return state;
}