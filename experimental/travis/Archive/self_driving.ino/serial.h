#include "commands.h"

// A pair of varibles to help parse serial commands
int arg   = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// Command variables
float throttle_cmd = 0;
float brake_cmd    = 0;
float steering_cmd = 0.5;
float rearVel      = 0;

// ROS enable
bool rosEnabled = false;

void resetCommand()
{
    cmd = NULL;
    memset(argv1, 0, sizeof(argv1));
    memset(argv2, 0, sizeof(argv2));
    arg1  = 0;
    arg2  = 0;
    arg   = 0;
    index = 0;
}

int runCommand()
{
    int i   = 0;
    char* p = argv1;
    char* str;
    int pid_args[4];
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);

    Serial.print(cmd);
    Serial.print(" ");
    Serial.print(arg1);
    Serial.print(" ");
    Serial.println(arg2);

    switch (cmd) {
    case GET_BAUDRATE:
        // Get the baud rate
        Serial.println(115200);
        break;
    case ROS_ENABLE:
        // Enable ROS
        rosEnabled = true;
        Serial.println("OK");
        break;
    case ROS_DISABLE:
        // Disable ROS
        rosEnabled = false;
        Serial.println("OK");
        break;
    case SET_THROTTLE:
        // Set the throttle
        if (arg2 == 0)
            arg2 = .01;
        throttle_cmd = (float)arg1 / (float)arg2;
        Serial.println("OK");
        break;
    case GET_THROTTLE:
        // Get the throttle
        Serial.println(throttle_cmd);
        break;
    case GET_BRAKE:
        // Get the brake
        Serial.println(brake_cmd);
        break;
    case SET_BRAKE:
        // Set the brake
        if (arg2 == 0)
            arg2 = .01;
        brake_cmd = (float)arg1 / (float)arg2;
        Serial.println("OK");
        break;
    case SET_STEERING:
        // Set the steering
        if (arg2 == 0)
            arg2 = .01;
        steering_cmd = (float)arg1 / (float)arg2;
        Serial.println("OK");
        break;
    case SET_STOP:
        // Stop the car
        throttle_cmd = 0;
        brake_cmd    = 0;
        steering_cmd = 0;
        Serial.println("OK");
        break;
    }
}

void checkSerial(char chr)
{
    if (chr == -1) {
        return;
    }
    if (chr == 13) {
        if (arg == 1)
            argv1[index] = NULL;
        else if (arg == 2)
            argv2[index] = NULL;
        runCommand();
        resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
        // Step through the arguments
        if (arg == 0)
            arg = 1;
        else if (arg == 1) {
            argv1[index] = NULL;
            arg          = 2;
            index        = 0;
        }
    } else {
        if (arg == 0) {
            // The first arg is the single-letter command
            cmd = chr;
        } else if (arg == 1) {
            // Subsequent arguments can be more than one character
            argv1[index] = chr;
            index++;
        } else if (arg == 2) {
            argv2[index] = chr;
            index++;
        }
    }
}
