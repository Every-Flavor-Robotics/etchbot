#include <Arduino.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include <atomic>
#include <chrono>
#include <vector>

#include "FastAccelStepper.h"
#include "HTTPClient.h"
#include "gcode.h"
#include "planner.h"
#include "wifi_gcode_stream.h"

//  Conversion from mm to steps
#define MICROSTEPS 16
#define GEAR_RATIO (1.0f) // Motor to knob
#define RAD_PER_MM (0.1858f / GEAR_RATIO)
#define MMPERMIN_TO_RADPERSEC (RAD_PER_MM / 60.0)
#define STEPS_PER_RAD (200.0f / (2.0f * PI)) * MICROSTEPS
#define STEPS_PER_MM (STEPS_PER_RAD * RAD_PER_MM)

#define ACCELERATION 512000     // steps/s^2
#define MAX_ACCELERATION 750000 // steps/s^2
#define UP_DOWN_BACKLASH_RAD (2.0 * STEPS_PER_MM)
#define LEFT_RIGHT_BACKLASH_RAD (1.8 * STEPS_PER_MM)
#define BACKLASH_COMPENSATION_RADPERSEC 100.0f

#define ERROR_TOLERANCE (0.01 * STEPS_PER_MM)

#define X_LIM 130
#define Y_LIM 89.375

// MotorGo Plink GPIO header (ESP32-S3)
#define UP_DOWN_STEP 39
#define UP_DOWN_DIR 37
#define LEFT_RIGHT_STEP 38
#define LEFT_RIGHT_DIR 40

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *left_right = nullptr;
FastAccelStepper *up_down = nullptr;

TaskHandle_t motor_task_handle;
void motor_task(void *pvParameters);

// Target velocities in rad/s (will be converted to steps/s in motor_task)
std::atomic<float> left_right_velocity_target(0.0);
std::atomic<float> up_down_velocity_target(0.0);

bool enable_flag = false;
bool disable_flag = false;
bool motors_enabled = false;

void freq_println(String str, int freq)
{
    static unsigned long last_print_time = 0;
    unsigned long now = millis();

    if (now - last_print_time > 1000 / freq)
    {
        Serial.println(str);
        last_print_time = now;
    }
}

void home()
{
    Serial.println("[home] Returning to (0,0)");

    // Stop the velocity-based target tracking
    motors_enabled = false;
    left_right_velocity_target.store(0);
    up_down_velocity_target.store(0);

    // Use FastAccelStepper's absolute move to return to 0 steps exactly
    if (left_right)
    {
        left_right->setSpeedInHz(3200); // Moderate speed for homing
        left_right->moveTo(0);
    }
    if (up_down)
    {
        up_down->setSpeedInHz(3200);
        up_down->moveTo(0);
    }

    // Wait for move to complete
    while ((left_right && left_right->isRunning()) || (up_down && up_down->isRunning()))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("[home] At origin");
}

// GCode objects
GCode::GCodeParser *parser;
GCode::WifiGCodeStream *stream;

void draw_pre_setup() {}

void draw_setup()
{
    Serial.println("[init] FastAccelStepper engine starting...");
    engine.init();
    left_right = engine.stepperConnectToPin(LEFT_RIGHT_STEP);
    if (left_right)
    {
        left_right->setDirectionPin(LEFT_RIGHT_DIR);
        left_right->setAcceleration(ACCELERATION);
    }
    else
    {
        Serial.println("[ERROR] Could not attach Left/Right stepper");
    }

    up_down = engine.stepperConnectToPin(UP_DOWN_STEP);
    if (up_down)
    {
        up_down->setDirectionPin(UP_DOWN_DIR);
        up_down->setAcceleration(ACCELERATION);
    }
    else
    {
        Serial.println("[ERROR] Could not attach Up/Down stepper");
    }

    stream = new GCode::WifiGCodeStream(HOST, 50);
    parser = new GCode::GCodeParser(stream, 2000);
    GCode::start_parser(*parser);

    xTaskCreatePinnedToCore(motor_task, "Motor Task", 4096, NULL, 1,
                            &motor_task_handle, 1);
}

void motor_task(void *pvParameters)
{
    while (true)
    {
        if (enable_flag)
        {
            // FastAccelStepper handles enable internally if pin is set,
            // but for now we just track state
            motors_enabled = true;
            enable_flag = false;
        }
        if (disable_flag)
        {
            left_right->stopMove();
            up_down->stopMove();
            motors_enabled = false;
            disable_flag = false;
        }

        if (motors_enabled)
        {
            float lr_vel_rad = left_right_velocity_target.load();
            float ud_vel_rad = up_down_velocity_target.load();

            int32_t lr_steps_s = (int32_t)(lr_vel_rad * STEPS_PER_RAD);
            int32_t ud_steps_s = (int32_t)(ud_vel_rad * STEPS_PER_RAD);

            if (lr_steps_s == 0)
            {
                left_right->stopMove();
            }
            else if (lr_steps_s > 0)
            {
                left_right->setSpeedInHz(lr_steps_s);
                left_right->runForward();
            }
            else
            {
                left_right->setSpeedInHz(-lr_steps_s);
                left_right->runBackward();
            }

            if (ud_steps_s == 0)
            {
                up_down->stopMove();
            }
            else if (ud_steps_s > 0)
            {
                up_down->setSpeedInHz(ud_steps_s);
                up_down->runForward();
            }
            else
            {
                up_down->setSpeedInHz(-ud_steps_s);
                up_down->runBackward();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        esp_task_wdt_reset();
    }
}

Planner::BacklashCompensatedTrajectoryParameters profile;
Planner::BacklashCompensatedTrajectory cur_trajectory;
float left_right_backlash_offset = 0;
float up_down_backlash_offset = 0;

GCode::MotionCommand command1;
GCode::MotionCommand command2;

GCode::MotionCommand *current_command = &command1;
bool next_command_ready = false;
GCode::MotionCommand *next_command = &command2;
GCode::MotionCommand *temp_command;

float previous_position_x = 0;
float previous_position_y = 0;
float previous_velocity_x = 0;
float previous_velocity_y = 0;
unsigned long previous_loop_time = 0;
bool first = true;
bool complete = false;

bool draw_loop()
{
    long lr_pos_steps = left_right->getCurrentPosition();
    long ud_pos_steps = up_down->getCurrentPosition();

    // Safety conditions
    if (lr_pos_steps > (X_LIM + 8) * STEPS_PER_MM ||
        lr_pos_steps < -8 * STEPS_PER_MM ||
        ud_pos_steps > (Y_LIM + 8) * STEPS_PER_MM ||
        ud_pos_steps < -8 * STEPS_PER_MM)
    {
        disable_flag = true;
    }

    if (complete)
        return true;

    if (next_command_ready && (first || Planner::compute_backlash_compesated_trapezoid_velocity_vector(cur_trajectory, micros()).is_complete))
    {
        if (first)
        {
            // Delay to allow buffer to fill
            vTaskDelay(pdMS_TO_TICKS(3000));
            first = false;
            motors_enabled = true;
        }

        // Swap the pointers
        temp_command = current_command;
        current_command = next_command;
        next_command = temp_command;
        next_command_ready = false;

        profile.main_profile.x_initial = (float)lr_pos_steps / STEPS_PER_RAD - left_right_backlash_offset / STEPS_PER_RAD;
        profile.main_profile.y_initial = (float)ud_pos_steps / STEPS_PER_RAD - up_down_backlash_offset / STEPS_PER_RAD;

        profile.main_profile.x_final = constrain(current_command->x * STEPS_PER_MM / STEPS_PER_RAD, 0, X_LIM * STEPS_PER_MM / STEPS_PER_RAD);
        profile.main_profile.y_final = constrain(current_command->y * STEPS_PER_MM / STEPS_PER_RAD, 0, Y_LIM * STEPS_PER_MM / STEPS_PER_RAD);

        profile.main_profile.v_initial = 0;
        profile.main_profile.v_target = current_command->feedrate * MMPERMIN_TO_RADPERSEC;
        profile.main_profile.v_final = 0;
        profile.main_profile.a_target = ACCELERATION / STEPS_PER_RAD;

        if (current_command->home)
        {
            home();
            complete = true;
        }
        else
        {
            profile.x_current = profile.main_profile.x_initial;
            profile.y_current = profile.main_profile.y_initial;
            profile.v_current = 0;
            profile.left_right_backlash_offset = left_right_backlash_offset / STEPS_PER_RAD;
            profile.up_down_backlash_offset = up_down_backlash_offset / STEPS_PER_RAD;
            profile.left_right_backlash_distance = LEFT_RIGHT_BACKLASH_RAD / STEPS_PER_RAD;
            profile.up_down_backlash_distance = UP_DOWN_BACKLASH_RAD / STEPS_PER_RAD;
            profile.v_target_backlash = BACKLASH_COMPENSATION_RADPERSEC;
            profile.a_target_backlash = ACCELERATION / STEPS_PER_RAD;
            profile.backlash_compensation_enabled = true;

            cur_trajectory = Planner::generate_backlash_compensated_profile(profile, ERROR_TOLERANCE / STEPS_PER_RAD);
            cur_trajectory.backlash_compensation_profile.start_time_us = micros();
            cur_trajectory.profile.start_time_us = micros(); // Will be offset in compute function
        }
    }

    unsigned long now = micros();
    Planner::TrajectoryState state = Planner::compute_backlash_compesated_trapezoid_velocity_vector(cur_trajectory, now);

    left_right_velocity_target.store(state.v.x);
    up_down_velocity_target.store(state.v.y);

    // Update backlash offsets (in steps, but stored as rad-equivalent here for simplicity with the logic)
    float cur_pos_x = (float)lr_pos_steps / STEPS_PER_RAD;
    float cur_pos_y = (float)ud_pos_steps / STEPS_PER_RAD;

    // Actually, let's keep backlash in steps for clarity if we are using steps
    // but the planner works in whatever units we give it.
    // For consistency with main_draw.h, let's keep units as radians for the targets.

    if (!next_command_ready && parser->is_available())
    {
        GCode::MotionCommandResult result = parser->pop_command_buffer();
        if (result.success)
        {
            next_command->x = result.command.x;
            next_command->y = result.command.y;
            next_command->z = result.command.z;
            next_command->feedrate = result.command.feedrate;
            next_command->home = result.command.home;
            next_command_ready = true;
        }
    }

    return complete;
}
