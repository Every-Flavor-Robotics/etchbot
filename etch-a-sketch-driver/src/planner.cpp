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

  //   if (abs(dx) < 0.25)
  //   {
  //     dx = 0;
  //   }

  //   if (abs(dy) < 0.25)
  //   {
  //     dy = 0;
  //   }

  //   String str = "dx: " + String(dx, 5) + " dy: " + String(dy, 5);
  //   freq_println(str, 100);

  float d_total = sqrt(dx * dx + dy * dy);

  //   Construct profile
  Planner::TrapezoidVelocityTrajectory profile;

  profile.dx = dx;
  profile.dy = dy;
  profile.x_initial = args.x_initial;
  profile.y_initial = args.y_initial;

  //   Serial.println("Initial position: (" + String(args.x_initial, 5) + ", " +
  //                  String(args.y_initial, 5) + ")");

  //   Serial.println("Distance: " + String(d_total, 5) + "rad");

  if (d_total < error_tolerance)
  {
    //   Serial.println("Distance is less than error tolerance");
    profile.left_right_direction = args.previous_left_right_direction;
    profile.up_down_direction = args.previous_up_down_direction;
    profile.v_backlash_left_right = 0;
    profile.v_backlash_up_down = 0;

    // Serial.println("Distance is less than error tolerance");
    // // Print x, x_final, dx
    // Serial.println("Initial x: " + String(args.x_initial, 5));
    // Serial.println("Final x: " + String(args.x_final, 5));
    // Serial.println("dx: " + String(dx, 5));

    // Serial.println("-------------- In planner -----------------");
    // Serial.println("Backlash compensation: " +
    //                String(args.backlash_compensation));
    // Serial.println("Backlash compensation time: " +
    //                String(profile.backlash_compensation_time_delta_us) +
    //                "us");
    // Serial.println("Left right backlash compensation velocity: " +
    //                String(profile.v_backlash_left_right) + " mm/s");
    // Serial.println("Up down backlash compensation velocity: " +
    //                String(profile.v_backlash_up_down) + " mm/s");
    // Serial.println("Initial position: (" + String(args.x_initial, 5) + ", " +
    //                String(args.y_initial, 5) + ")");
    // Serial.println("Final position: (" + String(args.x_final, 5) + ", " +
    //                String(args.y_final, 5) + ")");
    // Serial.println("Initial Total distance: " + String(d_total, 5) + "mm");
    // Serial.println("Initial velocity: " + String(args.v_initial, 5) +
    // "mm/s"); Serial.println("Target velocity: " + String(args.v_target, 5) +
    // "mm/s"); Serial.println("Final velocity: " + String(args.v_final, 5) +
    // "mm/s");

    // Serial.println("Angle: " + String(profile.angle, 5) + "rad");

    // Serial.println("-------------- In planner -----------------");

    return profile;
  }

  // Cap initial velocity to target velocity
  // When iterative replanning, this is necessary as the planner
  // cannot handle a higher initial velocity than the target velocity
  if (args.v_initial > args.v_target)
  {
    args.v_initial = args.v_target;
  }

  profile.angle = atan2(dy, dx);

  //   Define directions from angle
  //   atan2 returns an angle in the range of -pi to pi
  //   Quadrant1: 0 to pi/2
  //   Quadrant2: pi/2 to pi
  //   Quadrant3: -pi to -pi/2
  //   Quadrant4: -pi/2 to 0
  //   If the angle is in the first or fourth quadrant, the x direction
  //   is forward
  if (profile.angle >= -PI / 2 && profile.angle <= PI / 2)
  {
    profile.left_right_direction = FORWARD;
  }
  else
  {
    profile.left_right_direction = BACKWARD;
  }

  //   If the angle is in the first or second quadrant, the y direction
  //   is forward
  if (profile.angle >= 0)
  {
    profile.up_down_direction = FORWARD;
  }
  else
  {
    profile.up_down_direction = BACKWARD;
  }

  // If we want backlash compensation
  if (args.backlash_compensation)
  {
    float left_right_backlash_time = 0.0;
    float left_right_distance = 0.0;
    // Check if we need to compensate for backlash in the x direction
    if (profile.left_right_direction != args.previous_left_right_direction &&
        abs(dx) > 0.001)
    {
      // Compute time in seconds to reach the backlash compensation
      left_right_backlash_time =
          args.left_right_backlash_compensation_distance /
          args.backlash_compensation_velocity;

      left_right_distance = args.left_right_backlash_compensation_distance;
    }

    float up_down_backlash_time = 0.0;
    float up_down_distance = 0.0;
    // Check if we need to compensate for backlash in the y direction
    if (profile.up_down_direction != args.previous_up_down_direction &&
        abs(dy) > 0.001)
    {
      // Compute time in microseconds to reach the backlash compensation
      up_down_backlash_time = args.up_down_backlash_compensation_distance /
                              args.backlash_compensation_velocity;

      up_down_distance = args.up_down_backlash_compensation_distance;
    }

    // Choose the larger of the two backlash compensation times
    float backlash_compensation_time_delta =
        max(left_right_backlash_time, up_down_backlash_time);

    if (backlash_compensation_time_delta == 0)
    {
      profile.v_backlash_left_right = 0;
      profile.v_backlash_up_down = 0;
    }

    // Set the speeds for each direction
    profile.v_backlash_left_right =
        left_right_distance / backlash_compensation_time_delta;

    // Negate direction if we are moving backwards
    if (profile.left_right_direction == BACKWARD)
    {
      profile.v_backlash_left_right = -profile.v_backlash_left_right;
    }

    profile.v_backlash_up_down =
        up_down_distance / backlash_compensation_time_delta;

    // Negate direction if we are moving backwards
    if (profile.up_down_direction == BACKWARD)
    {
      profile.v_backlash_up_down = -profile.v_backlash_up_down;
    }

    profile.backlash_compensation_time_delta_us =
        (backlash_compensation_time_delta) * 1e6;

    if (profile.backlash_compensation_time_delta_us > 0)
    {
      profile.backlash_pause_time_delta_us = 5e5;
    }
    else
    {
      profile.v_backlash_left_right = 0;
      profile.v_backlash_up_down = 0;
    }
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

  profile.backlash_pause_time_delta_us =
      profile.backlash_compensation_time_delta_us +
      profile.backlash_pause_time_delta_us;
  profile.acceleration_time_delta_us =
      (t_accel * 1e6) + profile.backlash_pause_time_delta_us;
  profile.coast_end_time_delta_us =
      profile.acceleration_time_delta_us + (t_coast * 1e6);
  profile.end_time_delta_us = profile.coast_end_time_delta_us + (t_decel * 1e6);

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

  //   Serial.println("End time: " + String(profile.end_time_delta_us) + "us");

  //   Serial.println("-------------- In planner -----------------");

  return profile;
}

Planner::TrajectoryState Planner::compute_trapezoid_velocity_vector(
    Planner::TrapezoidVelocityTrajectory& profile, unsigned long time_us,
    float OL_THRESHOLD)
{
  TrajectoryState state;
  state.left_right_ol = abs(profile.dx) < OL_THRESHOLD && profile.dx != 0;
  state.up_down_ol = abs(profile.dy) < OL_THRESHOLD && profile.dy != 0;

  state.p.x = 0;
  state.p.y = 0;
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

  else if (t_us <= profile.backlash_compensation_time_delta_us)
  {
    //   Backlash compensation phase
    state.backlash_compensation_phase = true;
    state.up_down_ol = true;
    state.left_right_ol = true;

    // Skip remaining calculations, directly set backlash compensation
    state.p.x = profile.v_backlash_left_right * t + profile.x_initial;
    state.p.y = profile.v_backlash_up_down * t + profile.y_initial;

    // Serial.println(profile.x_initial);

    return state;
  }

  //   Backlash pause phase
  else if (t_us <= profile.backlash_pause_time_delta_us)
  {
    state.backlash_compensation_phase = true;
    state.up_down_ol = true;
    state.left_right_ol = true;

    // Switch back to regular operation, set velocity to 0
    float backlash_final_time =
        profile.backlash_compensation_time_delta_us / 1e6;

    state.p.x =
        profile.v_backlash_left_right * backlash_final_time + profile.x_initial;
    state.p.y =
        profile.v_backlash_up_down * backlash_final_time + profile.y_initial;

    return state;
  }

  // Constant acceleration phase
  else if (t_us <= profile.acceleration_time_delta_us)
  {
    p_target = 0.5 * profile.a_target * t * t + profile.v_initial * t;
    //   Acceleration phase
    v_target = profile.v_initial + profile.a_target * t;
    a_target = profile.a_target;
  }
  //   Constant velocity phase
  else if (t_us <= profile.coast_end_time_delta_us)
  {
    float acceleration_time = profile.acceleration_time_delta_us / 1e6;

    p_target = 0.5 * profile.a_target * acceleration_time * acceleration_time +
               profile.v_initial * acceleration_time +
               profile.v_target * (t - acceleration_time);

    //   Constant velocity phase
    v_target = profile.v_target;
    a_target = 0;
  }
  //   Deceleration phase
  else if (t_us <= profile.end_time_delta_us)
  {
    float acceleration_time = profile.acceleration_time_delta_us / 1e6;
    float constant_velocity_time = profile.coast_end_time_delta_us / 1e6;

    p_target =
        0.5 * profile.a_target * acceleration_time * acceleration_time +
        profile.v_initial * acceleration_time +
        profile.v_target * constant_velocity_time +
        profile.v_target * (t - constant_velocity_time - acceleration_time) -
        0.5 * profile.a_target *
            (t - constant_velocity_time - acceleration_time) *
            (t - constant_velocity_time - acceleration_time);

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

  if (state.left_right_ol)
  {
    state.v.x = 0;
    state.p.x = profile.v_backlash_left_right * t + profile.x_initial +
                p_target * cos(profile.angle);
  }
  else
  {
    state.v.x = v_target * cos(profile.angle);
  }

  if (state.up_down_ol)
  {
    state.v.y = 0;
    state.p.y = profile.v_backlash_up_down * t + profile.y_initial +
                p_target * sin(profile.angle);
  }
  else
  {
    state.v.y = v_target * sin(profile.angle);
  }
  state.a.x = a_target * cos(profile.angle);
  state.a.y = a_target * sin(profile.angle);

  return state;
}