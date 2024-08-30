#include "planner.h"

#include <Arduino.h>

#include <chrono>
// void freq_println(String str, int freq)
// {
//   static unsigned long last_print_time = 0;
//   unsigned long now = millis();

//   if (now - last_print_time > 1000 / freq)
//   {
//     Serial.println(str);
//     last_print_time = now;
//   }
// }

Planner::TrapezoidVelocityTrajectory Planner::generate_trapezoid_profile(
    Planner::TrapezoidTrajectoryParameters args, float error_tolerance)
{
  // Calculate the distance between the initial and final points
  float dx = args.x_final - args.x_initial;
  float dy = args.y_final - args.y_initial;
  float d_total = sqrt(dx * dx + dy * dy);

  //   Construct profile
  Planner::TrapezoidVelocityTrajectory profile;

  if (d_total < error_tolerance)
  {
    return profile;
  }

  profile.dx = dx;
  profile.dy = dy;

  profile.angle = atan2(dy, dx);

  // Cap initial velocity to target velocity
  // When iterative replanning, this is necessary as the planner
  // cannot handle a higher initial velocity than the target velocity
  if (args.v_initial > args.v_target)
  {
    args.v_initial = args.v_target;
  }

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
  profile.end_delay_delta_us = profile.end_time_delta_us + args.end_delay_us;

  profile.v_initial = args.v_initial;
  profile.v_target = args.v_target;
  profile.v_final = args.v_final;

  profile.a_target = args.a_target;

  //   Serial.println("-------------- In planner -----------------");
  //   Serial.println("Backlash compensation: " +
  //                  String(args.backlash_compensation));
  //   Serial.println("Left right backlash compensation velocity: " +
  //                  String(profile.v_backlash_left_right) + " mm/s");
  //   Serial.println("Up down backlash compensation velocity: " +
  //                  String(profile.v_backlash_up_down) + " mm/s");
  //   Serial.println("Initial position: (" + String(args.x_initial, 5) + ", "
  //   +
  //                  String(args.y_initial, 5) + ")");
  //   Serial.println("Final position: (" + String(args.x_final, 5) + ", " +
  //                  String(args.y_final, 5) + ")");
  //   Serial.println("Initial Total distance: " + String(d_total, 5) + "mm");
  //   Serial.println("Initial velocity: " + String(args.v_initial, 5) +
  //   "mm/s"); Serial.println("Target velocity: " + String(args.v_target, 5)
  //   + "mm/s"); Serial.println("Final velocity: " + String(args.v_final, 5)
  //   + "mm/s"); Serial.println("Acceleration time: " + String(t_accel, 5) +
  //   "s"); Serial.println("Coast time: " + String(t_coast, 5) + " s");
  //   Serial.println("Deceleration time: " + String(t_decel, 5) + " s");

  //   Serial.println("Total time: " + String(t_accel + t_coast + t_decel, 5)
  //   +
  //                  " us");

  //   Serial.println("Acceleration distance: " +
  //                  String(0.5 * args.a_target * t_accel * t_accel, 5) +
  //                  "mm");

  //   Serial.println("Coast distance: " + String(t_coast * args.v_target, 5)
  //   +
  //                  " mm");

  //   Serial.println("Deceleration distance: " +
  //                  String(0.5 * args.a_target * t_decel * t_decel, 5) +
  //                  "mm");

  //   Serial.println("Angle: " + String(profile.angle, 5) + "rad");

  //   //   Print all time deltas
  //   Serial.println("Backlash compensation time: " +
  //                  String(profile.backlash_compensation_time_delta_us) +
  //                  "us");

  //   Serial.println("Backlash pause time: " +
  //                  String(profile.backlash_pause_time_delta_us) + "us");

  //   Serial.println("Acceleration time: " +
  //                  String(profile.acceleration_time_delta_us) + "us");

  //   Serial.println("Coast end time: " +
  //   String(profile.coast_end_time_delta_us) +
  //                  "us");

  //   Serial.println("End time: " + String(profile.end_time_delta_us) +
  //   "us");

  //   Serial.println("-------------- In planner -----------------");

  return profile;
}

Planner::BacklashCompensatedTrajectory
Planner::generate_backlash_compensated_profile(
    Planner::BacklashCompensatedTrajectoryParameters args,
    float error_tolerance)
{
  // Create a new trajectory profile to output
  Planner::BacklashCompensatedTrajectory compensated_profile;

  // Calculate the distance between the initial and final points
  float dx = args.main_profile.x_final - args.x_current;
  float dy = args.main_profile.y_final - args.y_current;
  float d_total = sqrt(dx * dx + dy * dy);

  // Robot is already at the target position
  if (d_total < error_tolerance)
  {
    // Nothing to be done, return an empty profile
    return compensated_profile;
  }

  float angle = atan2(dy, dx);

  //   Define directions from angle
  //   atan2 returns an angle in the range of -pi to pi
  //   Quadrant1: 0 to pi/2
  //   Quadrant2: pi/2 to pi
  //   Quadrant3: -pi to -pi/2
  //   Quadrant4: -pi/2 to 0
  //   If the angle is in the first or fourth quadrant, the x direction
  //   is forward
  if (angle >= -PI / 2 && angle <= PI / 2)
  {
    compensated_profile.left_right_direction = FORWARD;
  }
  else
  {
    compensated_profile.left_right_direction = BACKWARD;
  }
  //   If the angle is in the first or second quadrant, the y direction
  //   is forward
  if (angle >= 0)
  {
    compensated_profile.up_down_direction = FORWARD;
  }
  else
  {
    compensated_profile.up_down_direction = BACKWARD;
  }

  // If we want backlash compensation
  if (args.backlash_compensation_enabled)
  {
    // Create args for backlash planning
    TrapezoidTrajectoryParameters backlash_compensation_args;

    // The backlash offset is the initial position for this trajectory
    backlash_compensation_args.x_initial = args.left_right_backlash_offset;
    backlash_compensation_args.y_initial = args.up_down_backlash_offset;

    backlash_compensation_args.v_initial = args.v_current;
    backlash_compensation_args.v_target = args.v_target_backlash;
    backlash_compensation_args.v_final = args.main_profile.v_initial;
    backlash_compensation_args.a_target = args.a_target_backlash;

    // x_final and y_final depend on direction
    if (compensated_profile.left_right_direction == FORWARD)
    {
      backlash_compensation_args.x_final = args.left_right_backlash_distance;
    }
    else if (compensated_profile.left_right_direction == BACKWARD)
    {
      backlash_compensation_args.x_final = 0.0;
    }
    else
    {
      // Don't do any compensation, because we are under the error tolerance
      backlash_compensation_args.x_final = args.left_right_backlash_offset;
    }

    if (compensated_profile.up_down_direction == FORWARD)
    {
      backlash_compensation_args.y_final = args.up_down_backlash_distance;
    }
    else if (compensated_profile.up_down_direction == BACKWARD)
    {
      backlash_compensation_args.y_final = 0.0;
    }
    else
    {
      // Don't do any compensation, because we are under the error tolerance
      backlash_compensation_args.y_final = args.up_down_backlash_offset;
    }

    backlash_compensation_args.end_delay_us = 1e6;

    // Serial.println("Backlash compensation args:");
    // Serial.println("x_initial: " +
    //                String(backlash_compensation_args.x_initial));
    // Serial.println("y_initial: " +
    //                String(backlash_compensation_args.y_initial));
    // Serial.println("x_final: " + String(backlash_compensation_args.x_final));
    // Serial.println("y_final: " + String(backlash_compensation_args.y_final));
    // Serial.println("end_delay_us: " +
    //                String(backlash_compensation_args.end_delay_us));

    // Compute the backlash compensation profile
    compensated_profile.backlash_compensation_profile =
        generate_trapezoid_profile(backlash_compensation_args, error_tolerance);
  }

  // Now compute the main profile
  // If the backlash compensation profile is empty, the initial velocity
  // is the current velocity, otherwise it is the initial velocity of the
  // specified in the main profile
  if (compensated_profile.backlash_compensation_profile.end_time_delta_us == 0)
  {
    args.main_profile.v_initial = args.v_current;
  }

  args.main_profile.x_initial = args.x_current;
  args.main_profile.y_initial = args.y_current;

  args.main_profile.end_delay_us = 0;

  compensated_profile.profile =
      generate_trapezoid_profile(args.main_profile, error_tolerance);

  //   Add the backlash compensation time to the main profile
  compensated_profile.profile.acceleration_time_delta_us +=
      compensated_profile.backlash_compensation_profile.end_delay_delta_us;
  compensated_profile.profile.coast_end_time_delta_us +=
      compensated_profile.backlash_compensation_profile.end_delay_delta_us;
  compensated_profile.profile.end_time_delta_us +=
      compensated_profile.backlash_compensation_profile.end_delay_delta_us;
  compensated_profile.profile.end_delay_delta_us +=
      compensated_profile.backlash_compensation_profile.end_delay_delta_us;

  //   Serial.println("-------------- In planner -----------------");
  //   //   Print all details about the backlash compensation profile
  //   Serial.println("Backlash compensation profile:");
  //   Serial.println("Acceleration time: " +
  //                  String(compensated_profile.backlash_compensation_profile
  //                             .acceleration_time_delta_us) +
  //                  "us");
  //   Serial.println("Coast end time: " +
  //                  String(compensated_profile.backlash_compensation_profile
  //                             .coast_end_time_delta_us) +
  //                  "us");
  //   Serial.println(
  //       "End time: " +
  //       String(
  //           compensated_profile.backlash_compensation_profile.end_time_delta_us)
  //           +
  //       "us");
  //   Serial.println("End delay time: " +
  //                  String(compensated_profile.backlash_compensation_profile
  //                             .end_delay_delta_us) +
  //                  "us");

  //   //   Print all details about the main profile
  //   Serial.println("Main profile:");
  //   Serial.println(
  //       "Acceleration time: " +
  //       String(compensated_profile.profile.acceleration_time_delta_us) +
  //       "us");
  //   Serial.println("Coast end time: " +
  //                  String(compensated_profile.profile.coast_end_time_delta_us)
  //                  + "us");
  //   Serial.println("End time: " +
  //                  String(compensated_profile.profile.end_time_delta_us) +
  //                  "us");
  //   Serial.println("End delay time: " +
  //                  String(compensated_profile.profile.end_delay_delta_us) +
  //                  "us");
  //   Serial.println("-------------- In planner -----------------");

  return compensated_profile;
}

Planner::TrajectoryState Planner::compute_trapezoid_velocity_vector(
    Planner::TrapezoidVelocityTrajectory& profile, unsigned long time_us,
    float OL_THRESHOLD)
{
  TrajectoryState state;

  state.v.x = 0;
  state.v.y = 0;
  state.a.x = 0;
  state.a.y = 0;

  float p_target;
  float v_target;
  float a_target;

  //   Compute t relative to the start time
  unsigned long t_us = time_us - profile.start_time_us;
  float t = t_us / 1e6;

  //   Check if degenerate case
  if (profile.end_time_delta_us == 0)
  {
    state.is_complete = true;
    return state;
  }

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

Planner::TrajectoryState
Planner::compute_backlash_compesated_trapezoid_velocity_vector(
    Planner::BacklashCompensatedTrajectory& profile, unsigned long time_us)
{
  TrajectoryState state;

  state.v.x = 0;
  state.v.y = 0;
  state.a.x = 0;
  state.a.y = 0;

  float v_target;
  float a_target;

  //   Compute t relative to the start time
  unsigned long t_us =
      time_us - profile.backlash_compensation_profile.start_time_us;
  float t = t_us / 1e6;

  //   Check if degenerate case
  if (profile.profile.end_time_delta_us == 0)
  {
    state.is_complete = true;
    return state;
  }

  // Trajectory has not started, set the start time and initial velocity
  if (profile.backlash_compensation_profile.start_time_us == 0)
  {
    // Serial.println("Starting profile");
    profile.backlash_compensation_profile.start_time_us = time_us;
    v_target = 0;
    a_target = 0;
  }

  TrapezoidVelocityTrajectory& cur_profile =
      t_us <= profile.backlash_compensation_profile.end_delay_delta_us
          ? profile.backlash_compensation_profile
          : profile.profile;

  if (t_us <= cur_profile.acceleration_time_delta_us)
  {
    //   Acceleration phase
    v_target = cur_profile.v_initial + cur_profile.a_target * t;
    a_target = cur_profile.a_target;
    state.backlash_compensation_phase = false;
  }
  //   Constant velocity phase
  else if (t_us <= cur_profile.coast_end_time_delta_us)
  {
    float acceleration_time = cur_profile.acceleration_time_delta_us / 1e6;

    //   Constant velocity phase
    v_target = cur_profile.v_target;
    a_target = 0;
    state.backlash_compensation_phase = false;
  }
  //   Deceleration phase
  else if (t_us <= cur_profile.end_time_delta_us)
  {
    //   Deceleration phase
    v_target =
        cur_profile.v_target -
        cur_profile.a_target * (t - cur_profile.coast_end_time_delta_us / 1e6);

    a_target = -cur_profile.a_target;

    state.backlash_compensation_phase = true;
  }
  else if (t_us <= cur_profile.end_delay_delta_us)
  {
    v_target = 0;
    a_target = 0;
    state.is_complete = true;
    state.backlash_compensation_phase = true;
  }
  //   Can only reach this point if the the second profile is complete
  else
  {
    //   Profile is complete
    v_target = cur_profile.v_final;
    state.is_complete = true;
    state.backlash_compensation_phase = false;
  }

  state.v.x = v_target * cos(cur_profile.angle);
  state.v.y = v_target * sin(cur_profile.angle);

  state.a.x = a_target * cos(cur_profile.angle);
  state.a.y = a_target * sin(cur_profile.angle);

  return state;
}