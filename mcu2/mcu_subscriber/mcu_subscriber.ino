// TODO change the sd.setCurrentMilliamps36v4(1000); to the right one for our system
// TODO make sure to test which direction is ccw and which direction is cw
// TODO handle the case when the node receives a callback while its still trying to set the stepper motor

// Note: must have the following libraries installed in arduino: 
//       HighPowerSteppperDriver
//       micro_ros_arduino-2.0.7-humble

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <SPI.h>
#include <HighPowerStepperDriver.h>


rcl_subscription_t rudder_subscriber;
std_msgs__msg__Float32* rudder_message;
float current_rudder_angle;
#define acceptable_rudder_error 0.1
#define rudder_max_current 1

rcl_subscription_t mast_subscriber;
std_msgs__msg__Float32* mast_message;
float current_mast_angle;
#define acceptable_mast_error 0.1
#define mast_max_current 1

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

const uint8_t rudder_dir_pin = 2;
const uint8_t rudder_step_pin = 3;
const uint8_t rudder_chip_select_pin = 4;

const uint8_t mast_dir_pin = 5;
const uint8_t mast_step_pin = 6;
const uint8_t mast_chip_select_pin = 7;

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const uint16_t step_period_us = 2000;

HighPowerStepperDriver rudder_stepper_driver;
HighPowerStepperDriver mast_stepper_driver;

#define ccw_direction  1
#define cw_direction  0

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Sends a pulse on the STEP pin to tell the driver to take one step, and also
//delays to control the speed of the motor.
void step(int step_pin)
{
  // The STEP minimum high pulse width is 1.9 microseconds.
  digitalWrite(step_pin, HIGH);
  delayMicroseconds(3);
  digitalWrite(step_pin, LOW);
  delayMicroseconds(3);
}

// Writes a high or low value to the direction pin to specify what direction to
// turn the motor.
void setDirection(int dir_pin, bool dir)
{
  // The STEP pin must not change for at least 200 nanoseconds before and after
  // changing the DIR pin.
  delayMicroseconds(1);
  digitalWrite(dir_pin, dir);
  delayMicroseconds(1);
}


void step_rudder_motor_cw() {
  setDirection(rudder_dir_pin, cw_direction);
  step(rudder_step_pin);
  delayMicroseconds(step_period_us);
}

void step_rudder_motor_ccw() {
  setDirection(rudder_dir_pin, ccw_direction);
  step(rudder_step_pin);
  delayMicroseconds(step_period_us);
}

void step_mast_motor_cw() {
  setDirection(mast_dir_pin, cw_direction);
  step(mast_step_pin);
  delayMicroseconds(step_period_us);
}

void step_mast_motor_ccw() {
  setDirection(mast_dir_pin, ccw_direction);
  step(mast_step_pin);
  delayMicroseconds(step_period_us);
}

// set mast stepper to given (absolute) angle
void set_mast_stepper_angle(float angle) {
  // #TODO
}

// set rudder stepper to given (absolute) angle
void set_rudder_stepper_angle(float angle) {
  // #TODO
}

void rudder_callback(const void * msgin) {  
  rudder_message = (std_msgs__msg__Float32 *)msgin;
  float desired_rudder_angle = rudder_message->data;

  set_rudder_stepper_angle(desired_rudder_angle);
  
  float rudder_error = current_rudder_angle - desired_rudder_angle;
  
  while (abs(rudder_error) > acceptable_rudder_error) {
    if (((int)rudder_error % 360) > 0 && ((int)rudder_error % 360) < 180) 
      step_rudder_motor_ccw();
    
    else 
      step_rudder_motor_cw();
    
    rudder_error = current_rudder_angle - desired_rudder_angle;
  }
}

void mast_callback(const void * msgin) {
  mast_message = (std_msgs__msg__Float32 *)msgin;
  float desired_mast_angle = mast_message->data;

  set_mast_stepper_angle(desired_mast_angle);
  
  float mast_error = current_mast_angle - desired_mast_angle;
  while (abs(mast_error) > acceptable_mast_error) {
    if (((int)mast_error % 360) > 0 && ((int)mast_error % 360) < 180) 
      step_mast_motor_ccw();
    
    else 
      step_mast_motor_cw();
    
    mast_error = current_mast_angle - desired_mast_angle;
  }
}

// TODO: we want to set different step mode for different steppers. consider taking it as parameter, not hardcoded value.
void init_stepper_driver(HighPowerStepperDriver stepper_driver, uint8_t chip_select_pin, uint8_t step_pin, uint8_t dir_pin, uint16_t max_current) {

  stepper_driver.setChipSelectPin(chip_select_pin);

  // Drive the STEP and DIR pins low initially.
  pinMode(step_pin, OUTPUT);
  digitalWrite(step_pin, LOW);
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, LOW);

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver to its default settings and clear latched status
  // conditions.
  stepper_driver.resetSettings();
  stepper_driver.clearStatus();

  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  stepper_driver.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  stepper_driver.setCurrentMilliamps36v4(max_current);

  // Set the number of microsteps that correspond to one full step.
  stepper_driver.setStepMode(HPSDStepMode::MicroStep32);

  // Enable the motor outputs.
  stepper_driver.enableDriver();
}



void setup() {
  set_microros_transports();
  
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

  // create rudder subscriber
  // rclc_subscription_init_default(
  //   &rudder_subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, rudder_message, Float32),
  //   "/actions/rudder_angle");

  // create mast subscriber
  // rclc_subscription_init_default(
  //   &mast_subscriber,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, mast_message, Float32),
  //   "/actions/mast_angle");

  // create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  // rclc_executor_add_subscription(&executor, &rudder_subscriber, &rudder_message, &rudder_callback, ON_NEW_DATA);
  // rclc_executor_add_subscription(&executor, &mast_subscriber, &mast_message, &mast_callback, ON_NEW_DATA);

  SPI.begin();

  // init_stepper_driver(rudder_stepper_driver, rudder_chip_select_pin, rudder_step_pin, rudder_dir_pin, rudder_max_current);
  // init_stepper_driver(mast_stepper_driver, mast_chip_select_pin, mast_step_pin, mast_dir_pin, mast_max_current);
}


void loop() {
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
