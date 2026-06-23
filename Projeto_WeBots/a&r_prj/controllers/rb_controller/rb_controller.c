#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 16 
#define RAD_TO_DEG (180.0 / M_PI)

#define KP 15.0  
#define KI 80.0    
#define KD 0.05    

#define SETPOINT 182.0 
#define MAX_SPEED 45.0 

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("motor1");
  WbDeviceTag right_motor = wb_robot_get_device("motor2");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu, TIME_STEP);

  double integral = 0.0;
  double last_error = 0.0;
  double dt = (double)TIME_STEP / 1000.0; 

  while (wb_robot_step(TIME_STEP) != -1) {
    
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double input = (rpy[1] * RAD_TO_DEG) + 180.0; 

    // Recovery Guard Gate
    if (input > 130.0 && input < 230.0) {
      
      double error = SETPOINT - input;
      
      double p_term = KP * error;
      
      // Integral (with anti-windup clamping)
      integral += error * dt;
      if (integral > 5.0) integral = 5.0;
      if (integral < -5.0) integral = -5.0;
      double i_term = KI * integral;
      
      double derivative = (error - last_error) / dt;
      double d_term = KD * derivative;
      
      double output = p_term + i_term + d_term;
      last_error = error;

      // Map output (-255 to 255) - Webots motor space
      double target_speed = (output / 255.0) * MAX_SPEED;

      if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
      if (target_speed < -MAX_SPEED) target_speed = -MAX_SPEED;

      // velocities inverted to drive into the direction of the fall
      wb_motor_set_velocity(left_motor, -target_speed);
      wb_motor_set_velocity(right_motor, -target_speed); 
      
      printf("Input Deg: %0.2f | Output: %0.2f | Motor Speed: %0.2f\n", input, output, target_speed);

    } else {
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);
      integral = 0.0;
      last_error = 0.0;
      printf("Bot fell over! Input Deg: %0.2f (Motors Stopped)\n", input);
    }
  };

  wb_robot_cleanup();
  return 0;
}