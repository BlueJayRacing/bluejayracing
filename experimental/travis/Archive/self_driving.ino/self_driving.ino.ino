#include "serial.h"
#include <algorithm>

const int THROTTLE_RPWM_PIN = 10;
const int THROTTLE_LPWM_PIN = 11;
const int BRAKE_RPWM_PIN    = 3;
const int BRAKE_LPWM_PIN    = 5;

const int REAR_WHEEL_HALL_PIN = 4;

const int STEERING_SERVO_PIN = 9; // Choose an appropriate PWM pin

const int STEERING_FREQ          = 200;  // PWM frequency in Hz
const int STEERING_MIN_PULSE     = 500;  // Minimum pulse width in microseconds
const int STEERING_MAX_PULSE     = 2500; // Maximum pulse width in microseconds
const int STEERING_NEUTRAL_PULSE = 1500; // Neutral pulse width in microseconds

const int RC_BRAKE_PIN    = 13;
const int RC_THROTTLE_PIN = 12;

const int BRAKE_ENCODER_PIN    = 2;
const int THROTTLE_ENCODER_PIN = 8;

const int MIN_PWM_VALUE = -1030;
const int MAX_PWM_VALUE = 1030;
const int THRESH_EDGE   = 10;
const int MIN_RC_PULSE  = 1000 + THRESH_EDGE;
const int MAX_RC_PULSE  = 2000 - THRESH_EDGE;
const int AMP_RC_PULSE  = 500;
const int DEADBAND      = 50;

const float BRAKE_ACTUATOR_LENGTH       = 4.0;
const float THROTTLE_ACTUATOR_LENGTH    = 2.0;
const float ENCODER_COUNTS_PER_INCH     = 374.0 / 2.0;
const float BRAKE_ENCODER_MIN_COUNTS    = 250; // more is more brake
const float THROTTLE_ENCODER_MIN_COUNTS = 700;

const float brakeMaxCounts          = ENCODER_COUNTS_PER_INCH * BRAKE_ACTUATOR_LENGTH;
const float brakePositionConversion = brakeMaxCounts / (MAX_RC_PULSE - MIN_RC_PULSE);

const float throttleMaxCounts          = ENCODER_COUNTS_PER_INCH * THROTTLE_ACTUATOR_LENGTH;
const float throttlePositionConversion = throttleMaxCounts / (MAX_RC_PULSE - MIN_RC_PULSE);

int brakePosition    = 0;
int throttlePosition = 0;

bool brakeDirection    = true;
bool throttleDirection = true;

const float meter_per_hall_tick = 0.05233333333;
unsigned long last              = 0;

void setup()
{
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
    brakePosition    = 0;
    throttlePosition = 0;
    Serial.begin(115200);
}

float ran    = 0;
int pos      = -1;
float step_s = 0.1;

void loop()
{
    for (int i = 0; i < 10; i++) {
        chr = Serial.read();
        checkSerial(chr);
    }

    float steeringAngle = (steering_cmd - 0.5) * 2.0; // Convert steering command to range [-1, 1]
    setSteeringAngle(steeringAngle);
    if (!rosEnabled) {
        int brakePulse    = pulseIn(RC_BRAKE_PIN, HIGH, 25000);
        int throttlePulse = pulseIn(RC_THROTTLE_PIN, HIGH, 25000);

        float brakeGoalPosition    = calculateGoalPositionBrake(brakePulse);
        float throttleGoalPosition = calculateGoalPositionThrottle(throttlePulse);
        throttleGoalPosition =
            (throttleGoalPosition > THROTTLE_ENCODER_MIN_COUNTS ? THROTTLE_ENCODER_MIN_COUNTS : throttleGoalPosition);
        controlMotor(brakeGoalPosition, brakePosition, BRAKE_RPWM_PIN, BRAKE_LPWM_PIN, brakeDirection);
        controlMotor(throttleGoalPosition, throttlePosition, THROTTLE_RPWM_PIN, THROTTLE_LPWM_PIN, throttleDirection);
    } else {
        float brakeGoalPosition    = brake_cmd * brakeMaxCounts;
        float throttleGoalPosition = throttle_cmd * throttleMaxCounts;

        brakeGoalPosition =
            (brakeGoalPosition > BRAKE_ENCODER_MIN_COUNTS ? BRAKE_ENCODER_MIN_COUNTS : brakeGoalPosition);
        controlMotor(brakeGoalPosition, brakePosition, BRAKE_RPWM_PIN, BRAKE_LPWM_PIN, brakeDirection);
        controlMotor(throttleGoalPosition, throttlePosition, THROTTLE_RPWM_PIN, THROTTLE_LPWM_PIN, throttleDirection);

        //    ran = ran + step_s*pos ;
        //    if (ran>1 || ran <-1) {
        //      ran = std::clamp(ran, -1.0f, 1.0f);
        //      pos = pos*-1;
        //    }
        //    steeringAngle = ran;
    }
}

void controlMotor(float goalPosition, int& currentPosition, int rpwmPin, int lpwmPin, bool& direction)
{
    //  Serial.print("Goal Position: ");
    //  Serial.println(goalPosition);
    //  Serial.print("Current Position: ");
    //  Serial.println(currentPosition);
    //  Serial.print("Position Diff: ");
    //  Serial.println(abs(goalPosition - currentPosition));

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

float calculateGoalPositionBrake(int brakePulse)
{
    float brakeGoalPosition = (MAX_RC_PULSE - brakePulse - AMP_RC_PULSE - THRESH_EDGE) * brakePositionConversion;
    return (brakeGoalPosition > BRAKE_ENCODER_MIN_COUNTS ? BRAKE_ENCODER_MIN_COUNTS : brakeGoalPosition);
}

float calculateGoalPositionThrottle(int throttlePulse)
{
    float throttleGoalPosition = (throttlePulse - MIN_RC_PULSE) * throttlePositionConversion;
    return throttleGoalPosition;
}

void moveMotorForward(int rpwmPin, int lpwmPin, float goalPosition, int currentPosition)
{
    int signal = 255;
    if (abs(goalPosition - currentPosition) < 10) {
        signal = 100;
    }
    analogWrite(rpwmPin, signal);
    analogWrite(lpwmPin, 0);
}

void moveMotorBackward(int rpwmPin, int lpwmPin, float goalPosition, int currentPosition)
{
    int signal = -255;
    if (abs(goalPosition - currentPosition) < 10) {
        signal = -100;
    }
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -signal);
}

void stopMotor(int rpwmPin, int lpwmPin)
{
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
}

void zeroMotors()
{
    analogWrite(THROTTLE_RPWM_PIN, 0);
    analogWrite(THROTTLE_LPWM_PIN, 255);
    analogWrite(BRAKE_RPWM_PIN, 0);
    analogWrite(BRAKE_LPWM_PIN, 255);
    setSteeringAngle(0.5);
}

void stopMotors()
{
    analogWrite(THROTTLE_RPWM_PIN, 0);
    analogWrite(THROTTLE_LPWM_PIN, 0);
    analogWrite(BRAKE_RPWM_PIN, 0);
    analogWrite(BRAKE_LPWM_PIN, 0);
}

void handleBrakeEncoder()
{
    if (brakeDirection == true) {
        brakePosition++;
    } else {
        brakePosition--;
    }
}

void handleThrottleEncoder()
{
    if (throttleDirection == true) {
        throttlePosition++;
    } else {
        throttlePosition--;
    }
}

void handleRearHall()
{
    Serial.println("Hall!");
    unsigned long now      = millis();
    float rear_vel_m_per_s = meter_per_hall_tick / ((now - last) / 1000);
    rearVel                = rear_vel_m_per_s;
    last                   = now;
}

void setPWM(int pin, int pulseWidth)
{
    //  if (pulseWidth > 1500) {
    //    digitalWrite(pin, HIGH);
    //  } else {
    //    digitalWrite(pin, LOW);
    //  }
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(pin, LOW);
    delayMicroseconds(4000 - pulseWidth);
    //    analogWrite(pin, (pulseWidth/20000)*255);
}

void setSteeringAngle(float steeringCmd)
{
    int pulseWidth          = map(1000 * steeringCmd, -1000, 1000, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
    unsigned long startTime = millis();
    while (millis() - startTime < 20) {
        setPWM(STEERING_SERVO_PIN, pulseWidth);
    }
}

void printDebugInfo(int brakePulse, int brakePosition)
{
    Serial.print("RC BRAKE Pulse: ");
    Serial.println(brakePulse);
    Serial.print("LIN AC Pos: ");
    Serial.println(int(brakePosition));
    Serial.println("");
}
