#include "motorgo_mini.h"

const MotorGo::MotorConfiguration OGMotor(11,        // pole_pairs
                                          500,       // kv
                                          0.1f,      // phase_resistance
                                          NOT_SET,   // phase_inductance
                                          5,         // voltage_limit
                                          10.0f,     // current_limit
                                          10000.0f,  // velocity_limit
                                          0.2f       // calibration_voltage
);

const MotorGo::MotorConfiguration GARTTLeftTronix(11,        // pole_pairs
                                                  338.85,    // kv
                                                  0.1088f,   // phase_resistance
                                                  NOT_SET,   // phase_inductance
                                                  4.5,       // voltage_limit
                                                  6.0f,      // current_limit
                                                  10000.0f,  // velocity_limit
                                                  1.1f  // calibration_voltage
);

const MotorGo::MotorConfiguration GARTTRightTronix(11,       // pole_pairs
                                                   374.59,   // kv
                                                   0.1088f,  // phase_resistance
                                                   NOT_SET,  // phase_inductance
                                                   4.5,      // voltage_limit
                                                   6.0f,     // current_limit
                                                   10000.0f,  // velocity_limit
                                                   1.1f  // calibration_voltage
);

// PID PARAMETERS
//  OG Motor

// left_right_velocity_pid_params.p = 0.26;
// left_right_velocity_pid_params.i = 0.0;
// left_right_velocity_pid_params.d = 0.0;
// left_right_velocity_pid_params.lpf_time_constant = 0.00;
// left_right_ff_accel_gain = 0.03 / 100;
// left_right_ff_velocity_gain = 0.025;

// left_right_velocity_pid.P = left_right_velocity_pid_params.p;
// left_right_velocity_pid.I = 0;
// left_right_velocity_pid.D = 0;
// left_right_velocity_lpf.Tf =
// left_right_velocity_pid_params.lpf_time_constant;

// up_down_velocity_pid_params.p = 0.26;
// up_down_velocity_pid_params.i = 0.0;
// up_down_velocity_pid_params.d = 0.0;
// up_down_velocity_pid_params.lpf_time_constant = 0.00;
// up_down_ff_accel_gain = 0.03 / 100;
// up_down_ff_velocity_gain = 0.025;

// up_down_velocity_pid.P = up_down_velocity_pid_params.p;
// up_down_velocity_pid.I = 0;
// up_down_velocity_pid.D = 0;
// up_down_velocity_lpf.Tf = up_down_velocity_pid_params.lpf_time_constant;
