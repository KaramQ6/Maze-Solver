/* Exercise the real ARDUINO driver/controller paths against deterministic hardware IO. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "controller.h"
#include "motors.h"

unsigned long test_time_ms = 0;
int test_pins[32] = {};
int test_pwm[32] = {};
int test_channels[32] = {};
static float heading = 0.0f;
static float front_mm = 500.0f;
static bool gyro_ok = true;
static bool range_ok = true;
static bool stalled = false;

static int wheel_command(int in1, int in2, int pwm_pin) {
    int duty = 0;
    for (int channel = 0; channel < 32; channel++) {
        if (test_channels[channel] == pwm_pin) duty = test_pwm[channel];
    }
    if (test_pins[in1] == test_pins[in2]) return 0;
    return test_pins[in1] == HIGH ? duty : -duty;
}

void test_advance_time(unsigned long duration_ms) {
    int left = wheel_command(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_PWMA);
    int right = wheel_command(PIN_MOTOR_IN3, PIN_MOTOR_IN4, PIN_MOTOR_PWMB);
    if (!stalled) {
        front_mm -= (left + right) * (float)duration_ms / 1000.0f;
        heading += (left - right) * (float)duration_ms / 600.0f;
    }
    test_time_ms += duration_ms;
}

bool sensors_init(void) { return true; }
bool sensors_update_gyro(void) { return gyro_ok; }
float sensors_get_heading_deg(void) { return heading; }
void sensors_reset_heading(float value) { heading = value; }
bool sensors_read_distances(SensorDistances *dist) {
    if (!range_ok) return false;
    dist->front_mm = front_mm;
    dist->left_mm = dist->right_mm = WALL_TARGET_SIDE_MM;
    dist->wall_front = front_mm < WALL_DETECT_THRESHOLD_MM;
    dist->wall_left = dist->wall_right = true;
    return true;
}

static void assert_stopped() {
    assert(wheel_command(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_PWMA) == 0);
    assert(wheel_command(PIN_MOTOR_IN3, PIN_MOTOR_IN4, PIN_MOTOR_PWMB) == 0);
}

int main() {
    static_assert(!ROBOT_HAS_ENCODERS, "C3 test must not require encoders");
    motors_init();
    motors_set_speed(100, -120);
    assert(wheel_command(10, 20, 21) == 100);
    assert(wheel_command(8, 7, 6) == -120);
    motors_brake();
    assert(test_pins[10] == HIGH && test_pins[20] == HIGH);
    assert(test_pins[8] == HIGH && test_pins[7] == HIGH);
    assert_stopped();
    motors_stop();
    assert(test_pins[10] == LOW && test_pins[20] == LOW);
    assert_stopped();

    controller_init();
    front_mm = 500.0f;
    assert(controller_step_forward(SEARCH_BASE_PWM));
    assert(fabsf(500.0f - front_mm - CELL_PITCH_MM) <= CELL_ARRIVAL_TOLERANCE_MM);
    assert_stopped();

    stalled = true;
    front_mm = 500.0f;
    assert(!controller_step_forward(SEARCH_BASE_PWM)); /* Timeout is NOT a cell crossing. */
    assert(front_mm == 500.0f);
    assert_stopped();
    stalled = false;

    front_mm = FRONT_REFERENCE_MAX_MM + 1.0f;
    assert(!controller_step_forward(SEARCH_BASE_PWM));
    assert_stopped();

    range_ok = false;
    assert(!controller_step_forward(SEARCH_BASE_PWM));
    assert_stopped();
    range_ok = true;

    gyro_ok = false;
    front_mm = 500.0f;
    assert(!controller_step_forward(SEARCH_BASE_PWM));
    assert(!controller_turn_90(false));
    assert_stopped();
    gyro_ok = true;

    controller_init();
    assert(controller_turn_90(false));
    assert(fabsf(heading - 90.0f) <= 1.2f);
    assert(controller_turn_90(true));
    assert(fabsf(heading) <= 1.2f);
    assert_stopped();
    puts("C3 TB6612 direction, brake, ToF arrival, stall, and gyro fault tests passed.");
}
