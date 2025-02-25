#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

const int THROTTLE_RPWM_PIN = 10;
const int THROTTLE_LPWM_PIN = 11;
const int BRAKE_RPWM_PIN = 3;
const int BRAKE_LPWM_PIN = 5;
const int REAR_WHEEL_HALL_PIN = 4;
const int STEERING_SERVO_PIN = 9;

const int STEERING_FREQ = 200;
const int STEERING_MIN_PULSE = 500;
const int STEERING_MAX_PULSE = 2500;
const int STEERING_NEUTRAL_PULSE = 1500;

const int RC_BRAKE_PIN = 13;
const int RC_THROTTLE_PIN = 12;
const int BRAKE_ENCODER_PIN = 2;
const int THROTTLE_ENCODER_PIN = 8;

const int MIN_PWM_VALUE = -1030;
const int MAX_PWM_VALUE = 1030;
const int THRESH_EDGE = 10;
const int MIN_RC_PULSE = 1000 + THRESH_EDGE;
const int MAX_RC_PULSE = 2000 - THRESH_EDGE;
const int AMP_RC_PULSE = 500;
const int DEADBAND = 50;

const float BRAKE_ACTUATOR_LENGTH = 4.0;
const float THROTTLE_ACTUATOR_LENGTH = 2.0;
const float ENCODER_COUNTS_PER_INCH = 374.0 / 2.0;
const float BRAKE_ENCODER_MIN_COUNTS = 250;
const float THROTTLE_ENCODER_MIN_COUNTS = 700;

const float brakeMaxCounts = ENCODER_COUNTS_PER_INCH * BRAKE_ACTUATOR_LENGTH;
const float brakePositionConversion = brakeMaxCounts / (MAX_RC_PULSE - MIN_RC_PULSE);
const float throttleMaxCounts = ENCODER_COUNTS_PER_INCH * THROTTLE_ACTUATOR_LENGTH;
const float throttlePositionConversion = throttleMaxCounts / (MAX_RC_PULSE - MIN_RC_PULSE);

int brakePosition = 0;
int throttlePosition = 0;
bool brakeDirection = true;
bool throttleDirection = true;

const float meter_per_hall_tick = 0.05233333333;
unsigned long last = 0;

// Motor command node
rcl_node_t motor_node;
rcl_publisher_t throttle_pub;
rcl_publisher_t brake_pub;
std_msgs__msg__Float32 throttle_msg;
std_msgs__msg__Float32 brake_msg;

// Steering command node
rcl_node_t steering_node;
rcl_publisher_t steering_pub;
std_msgs__msg__Float32 steering_msg;

// Serial node
rcl_node_t serial_node;
rcl_subscription_t cmd_sub;
geometry_msgs__msg__Twist cmd_msg;

void controlMotor(float goalPosition, int& currentPosition, int rpwmPin, int lpwmPin, bool& direction);
void moveMotorForward(int rpwmPin, int lpwmPin, float goalPosition, int currentPosition);
void moveMotorBackward(int rpwmPin, int lpwmPin, float goalPosition, int currentPosition);
void stopMotor(int rpwmPin, int lpwmPin);
void zeroMotors();
void stopMotors();
void handleBrakeEncoder();
void handleThrottleEncoder();
void handleRearHall();
void setPWM(int pin, int pulseWidth);
void setSteeringAngle(float steeringCmd);

void motor_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  throttle_msg.data = msg->linear.x;
  brake_msg.data = msg->linear.y;
  
  rcl_publish(&throttle_pub, &throttle_msg, NULL);
  rcl_publish(&brake_pub, &brake_msg, NULL);
}

void steering_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  steering_msg.data = msg->angular.z;
  
  rcl_publish(&steering_pub, &steering_msg, NULL);
}

void setup() {
  pinMode(RC_BRAKE_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(THROTTLE_RPWM_PIN, OUTPUT);
  pinMode(THROTTLE_LPWM_PIN, OUTPUT);
  pinMode(BRAKE_RPWM_PIN, OUTPUT);
  pinMode(BRAKE_LPWM_PIN, OUTPUT);
  pinMode(STEERING_SERVO_PIN, OUTPUT);
  pinMode(REAR_WHEEL_HALL_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(BRAKE_ENCODER_PIN), handleBrakeEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_ENCODER_PIN), handleThrottleEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(REAR_WHEEL_HALL_PIN), handleRearHall, RISING);

  zeroMotors();
  delay(9000);
  stopMotors();

  brakePosition = 0;
  throttlePosition = 0;

  Serial.begin(115200);

  // Initialize micro-ROS
  rclc_support_t support;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create motor command node
  rclc_node_init_default(&motor_node, "motor_node", "", &support);
  rclc_publisher_init_default(&throttle_pub, &motor_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "throttle_cmd");
  rclc_publisher_init_default(&brake_pub, &motor_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "brake_cmd");

  // Create steering command node
  rclc_node_init_default(&steering_node, "steering_node", "", &support);
  rclc_publisher_init_default(&steering_pub, &steering_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "steering_cmd");

  // Create serial node
  rclc_node_init_default(&serial_node, "serial_node", "", &support);
  rclc_subscription_init_default(&cmd_sub, &serial_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel", motor_callback);
}

void loop() {
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &motor_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &steering_callback, ON_NEW_DATA);

  rclc_executor_spin(&executor);

  float steeringAngle = steering_msg.data;
  setSteeringAngle(steeringAngle);

  float brakeGoalPosition = brake_msg.data * brakeMaxCounts;
  float throttleGoalPosition = throttle_msg.data * throttleMaxCounts;

  brakeGoalPosition = (brakeGoalPosition > BRAKE_ENCODER_MIN_COUNTS ? BRAKE_ENCODER_MIN_COUNTS : brakeGoalPosition);
  controlMotor(brakeGoalPosition, brakePosition, BRAKE_RPWM_PIN, BRAKE_LPWM_PIN, brakeDirection);
  controlMotor(throttleGoalPosition, throttlePosition, THROTTLE_RPWM_PIN, THROTTLE_LPWM_PIN, throttleDirection);
}

void controlMotor(float goalPosition, int& currentPosition, int rpwmPin, int lpwmPin, bool& direction) {
  if (abs(goalPosition - currentPosition) < 5) {
    stopMotor(rpwmPin, lpwmPin);
  } else {
    if (goalPosition > currentPosition) {
      direction = true;
      moveMotorForward(rpwmPin, lpwmPin, goalPosition, currentPosition);
    } else {
      direction = false;
      moveMotorBackward(rpwmPin, lpwmPin, goalPosition, currentPosition);
    }
  }
}

void moveMotorForward(int rpwmPin, int lpwmPin, float goalPosition, int currentPosition) {
  int signal = 255;
  if (abs(goalPosition - currentPosition) < 10) {
    signal = 100;
  }
  analogWrite(rpwmPin, signal);
  analogWrite(lpwmPin, 0);
}

void moveMotorBackward(int rpwmPin, int lpwmPin, float goalPosition, int currentPosition) {
  int signal = -255;
  if (abs(goalPosition - currentPosition) < 10) {
    signal = -100;
  }
  analogWrite(rpwmPin, 0);
  analogWrite(lpwmPin, -signal);
}

void stopMotor(int rpwmPin, int lpwmPin) {
  analogWrite(rpwmPin, 0);
  analogWrite(lpwmPin, 0);
}

void zeroMotors() {
  analogWrite(THROTTLE_RPWM_PIN, 0);
  analogWrite(THROTTLE_LPWM_PIN, 255);
  analogWrite(BRAKE_RPWM_PIN, 0);
  analogWrite(BRAKE_LPWM_PIN, 255);
  setSteeringAngle(0.5);
}

void stopMotors() {
  analogWrite(THROTTLE_RPWM_PIN, 0);
  analogWrite(THROTTLE_LPWM_PIN, 0);
  analogWrite(BRAKE_RPWM_PIN, 0);
  analogWrite(BRAKE_LPWM_PIN, 0);
}

void handleBrakeEncoder() {
  if (brakeDirection == true) {
    brakePosition++;
  } else {
    brakePosition--;
  }
}

void handleThrottleEncoder() {
  if (throttleDirection == true) {
    throttlePosition++;
  } else {
    throttlePosition--;
  }
}

void handleRearHall() {
  Serial.println("Hall!");
  unsigned long now = millis();
  float rear_vel_m_per_s = meter_per_hall_tick / ((now - last) / 1000);
  last = now;
}

void setPWM(int pin, int pulseWidth) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
  delayMicroseconds(4000 - pulseWidth);
}

void setSteeringAngle(float steeringCmd) {
  int pulseWidth = map(1000 * steeringCmd, -1000, 1000, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  unsigned long startTime = millis();
  while (millis() - startTime < 20) {
    setPWM(STEERING_SERVO_PIN, pulseWidth);
  }
}
