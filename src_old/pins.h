#ifndef PINS_H
#define PINS_H

// REGBOT_HW4 will be defined in makefile, if Teensy 3.5 is selected.

#define OLD_PIN_LINE_LED        32 // For version < 3
#define OLD_PIN_START_BUTTON    11 // For version 1

// Pin defines for Regbot v3 and older

#ifndef REGBOT_HW4
// Main pins
#define PIN_LED_DEBUG           13 // (LED_BUILTIN)
#define PIN_START_BUTTON        6
#define PIN_LINE_LED_HIGH       18
#define PIN_LINE_LED_LOW        25
#define PIN_DISABLE2            11
#define PIN_POWER_IR            32
#define PIN_POWER_ROBOT         33

// ADC pins
#define ADC_NUM_IR_SENSORS      2
#define ADC_NUM_NO_LS           5
#define ADC_NUM_ALL             (ADC_NUM_NO_LS + 8)
#define PIN_IR_RAW_1            A1
#define PIN_IR_RAW_2            A0
#define PIN_BATTERY_VOLTAGE     A9
#define PIN_LEFT_MOTOR_CURRENT  A10
#define PIN_RIGHT_MOTOR_CURRENT A11
#define PIN_LINE_SENSOR_0       A12
#define PIN_LINE_SENSOR_1       A13
#define PIN_LINE_SENSOR_2       A15   // A19
#define PIN_LINE_SENSOR_3       A16   // A15
#define PIN_LINE_SENSOR_4       A17   // A16
#define PIN_LINE_SENSOR_5       A18   // A20
#define PIN_LINE_SENSOR_6       A19   // A17
#define PIN_LINE_SENSOR_7       A20   // A18

// Motor Controller pins
#define PIN_LEFT_DIR            2
#define PIN_LEFT_PWM            3
#define PIN_RIGHT_PWM           4
#define PIN_RIGHT_DIR           12
#define PIN_LEFT_ENCODER_A      20
#define PIN_LEFT_ENCODER_B      19
#define PIN_RIGHT_ENCODER_A     22
#define PIN_RIGHT_ENCODER_B     21
#define M1DIR           PIN_LEFT_DIR // M1IN2 - Teensy Digital 2 (direction)
#define M1PWM           PIN_LEFT_PWM // M1IN1 - Teensy Digital 3 (PWM)
#define M2DIR3          PIN_RIGHT_DIR // M2IN2 - Teensy Digital 8 (direction) hardware 3 only
#define M2PWM3          PIN_RIGHT_PWM // M2IN1 - Teensy Digital 4 (PWM) hardware 3
#define M12DIS          PIN_DISABLE2 // M1+M2 D2  enable both motors - hardware 3 only
#define M1ENC_A         PIN_LEFT_ENCODER_A // Encoder A - Teensy Digital 20
#define M1ENC_B         PIN_LEFT_ENCODER_B // Encoder B - Teensy Digital 19
#define M2ENC_A         PIN_RIGHT_ENCODER_A // Encoder A - Teensy Digital 22
#define M2ENC_B         PIN_RIGHT_ENCODER_B // Encoder B - Teensy Digital 21
// Legacy pins (only used < 3)
#define M1DIS1          4 // M1D2  - Teensy Digital 4 (disable (hiz))) - hardware < 3
#define M2DIS1          10 // M2D2  - Teensy Digital 10 (disable (hi-z)) - hardware < 3
#define M2PWM1          9 // M2IN1 - Teensy Digital 9 (PWM) hardware < 3
#define SLEW            7 // SLEW  - Teensy Digital 7  -  hardware 1 og 2 only, hardware 3 fixed high
#define M2DIR1          8 // M2IN2 - Teensy Digital 8 (direction) hardware < 3 only

// Servo pins
#define PIN_SERVO1      5 // Teensy pin for servo 1
#define PIN_SERVO2      9 // Teensy pin for servo 2 - not valid on version 3.0 or earlier (used for motor enable)
#define PIN_SERVO3      10 // Teensy pin for servo 3 - not valid on version 3.0 or earlier (used for motor enable)
#define SERVO_PIN_0     A14 // IO pin on servo board
#define SERVO_PIN_1     24  // IO pin on servo board

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Pin Definitions for Regbot v4.1 2018
#elif defined(REGBOT_HW4)
// Main pins
#define PIN_LED_STATUS          11
#define PIN_LED_DEBUG           13
#define PIN_START_BUTTON        26
#define PIN_LINE_LED_HIGH       12
#define PIN_LINE_LED_LOW        25
#define PIN_DISABLE2            27
#define PIN_POWER_IR            24
#define PIN_POWER_ROBOT         28

// ADC pins
#define ADC_NUM_IR_SENSORS      2
#define ADC_NUM_NO_LS           5
#define ADC_NUM_ALL             (ADC_NUM_NO_LS + 8)
//
#define PIN_IR_RAW_1            A1
#define PIN_IR_RAW_2            A0
#define PIN_BATTERY_VOLTAGE     A14
#define PIN_LEFT_MOTOR_CURRENT  A2
#define PIN_RIGHT_MOTOR_CURRENT A3
//
#define PIN_LINE_SENSOR_0       A12
#define PIN_LINE_SENSOR_1       A13
#define PIN_LINE_SENSOR_2       A15
#define PIN_LINE_SENSOR_3       A16
#define PIN_LINE_SENSOR_4       A17
#define PIN_LINE_SENSOR_5       A18
#define PIN_LINE_SENSOR_6       A19
#define PIN_LINE_SENSOR_7       A20

// Motor Controller pins
#define PIN_LEFT_DIR            6
#define PIN_LEFT_PWM            3
#define PIN_RIGHT_PWM           4
#define PIN_RIGHT_DIR           2

#define PIN_LEFT_ENCODER_A      23
#define PIN_LEFT_ENCODER_B      22
#define PIN_RIGHT_ENCODER_A     20
#define PIN_RIGHT_ENCODER_B     21
#define M1DIR           PIN_LEFT_DIR // M1IN2 - Teensy Digital (direction)
#define M1PWM           PIN_LEFT_PWM // M1IN1 - Teensy Digital (PWM)
#define M2DIR3          PIN_RIGHT_DIR // M2IN2 - Teensy Digital (direction) hardware 3 only
#define M2PWM3          PIN_RIGHT_PWM // M2IN1 - Teensy Digital (PWM) hardware 3
#define M12DIS          PIN_DISABLE2 // M1+M2 D2  enable both motors - hardware 3 only
#define M1ENC_A         PIN_LEFT_ENCODER_A // Encoder A - Teensy Digital
#define M1ENC_B         PIN_LEFT_ENCODER_B // Encoder B - Teensy Digital
#define M2ENC_A         PIN_RIGHT_ENCODER_A // Encoder A - Teensy Digital
#define M2ENC_B         PIN_RIGHT_ENCODER_B // Encoder B - Teensy Digital
// Legacy pins (only used < 3)
#define M1DIS1          4 // M1D2  - Teensy Digital 4 (disable (hiz))) - hardware < 3
#define M2DIS1          10 // M2D2  - Teensy Digital 10 (disable (hi-z)) - hardware < 3
#define M2PWM1          9 // M2IN1 - Teensy Digital 9 (PWM) hardware < 3
#define SLEW            7 // SLEW  - Teensy Digital 7  -  hardware 1 og 2 only, hardware 3 fixed high
#define M2DIR1          8 // M2IN2 - Teensy Digital 8 (direction) hardware < 3 only

// Servo pins
#define PIN_SERVO1      5 // Teensy pin for servo 1
#define PIN_SERVO2      9 // Teensy pin for servo 2
#define PIN_SERVO3      10 // Teensy pin for servo 3
#define PIN_SERVO4      29 // Teensy pin for servo 4
#define PIN_SERVO5      30 // Teensy pin for servo 5 (Shared with Buzzer!)
#define SERVO_PIN_0     PIN_SERVO4 // Deprecated!
#define SERVO_PIN_1     PIN_SERVO5 // Deprecated!
#define PIN_BUZZER		30 // Shared with SERVO5!

#endif

#endif
