#ifndef MOTORS_H
#define MOTORS_H

#include <AccelStepper.h>

/**
 * @brief Total number of motors in the system
 * Modify this value when adding or removing motors
 */
#define NUM_MOTORS 11

/**
 * @brief Motor indices for easier referencing
 */
enum MotorIndex {
    IDX_EPU, IDX_EPD,  // Elbow Pitch Up/Down
    IDX_EYR, IDX_EYL,  // Elbow Yaw Right/Left
    IDX_WPU, IDX_WPD,  // Wrist Pitch Up/Down
    IDX_RJL, IDX_LJR,  // Jaw cables
    IDX_LJL, IDX_RJR,
    IDX_ROLL       // Roll motor
};

/**
 * @brief Pin definitions for all stepper motors
 */
// Elbow Pitch Motor Pins
#define PIN_EPU_STEP 41
#define PIN_EPU_DIR  40
#define PIN_EPD_STEP 49
#define PIN_EPD_DIR  48

// Elbow Yaw Motor Pins
#define PIN_EYL_STEP 39
#define PIN_EYL_DIR  38
#define PIN_EYR_STEP 53
#define PIN_EYR_DIR  52

// Wrist Pitch Motor Pins
#define PIN_WPU_STEP 29
#define PIN_WPU_DIR  28
#define PIN_WPD_STEP 35
#define PIN_WPD_DIR  34

// Jaw Motor Pins
#define PIN_RJL_STEP 47
#define PIN_RJL_DIR  46
#define PIN_LJR_STEP 51
#define PIN_LJR_DIR  50
#define PIN_LJL_STEP 45
#define PIN_LJL_DIR  44
#define PIN_RJR_STEP 43
#define PIN_RJR_DIR  42

// Roll Motor Pins
#define PIN_ROLL_STEP 37
#define PIN_ROLL_DIR  36

// Motor parameters
const float DEFAULT_MAX_SPEED = 700.0;
const float DEFAULT_ACCELERATION = 1000000.0;
const float TENSION_SPEED_COARSE = 600.0;
const float TENSION_SPEED_FINE = 50.0;

// Conversion constants
#define MM_TO_STEPS(x) ((int)((x/0.5)*200))

// Motor instances declarations
// Remove enum value conflicts by renaming variables with a suffix _motor
extern AccelStepper EPU_motor;
extern AccelStepper EPD_motor;
extern AccelStepper EYL_motor;
extern AccelStepper EYR_motor;
extern AccelStepper WPD_motor;
extern AccelStepper WPU_motor;
extern AccelStepper RJL_motor;
extern AccelStepper LJR_motor;
extern AccelStepper LJL_motor;
extern AccelStepper RJR_motor;
extern AccelStepper ROLL_motor;

// Array of all motors
extern AccelStepper* motors[NUM_MOTORS];

/**
 * @brief Initialize all motors with default settings
 */
void initializeMotors();

/**
 * @brief Tension a motor with specified speed
 * @param motor Pointer to the motor to tension
 * @param coarse If true, uses coarse tensioning speed, otherwise uses fine tensioning
 */
void tensionMotor(AccelStepper* motor, bool coarse);

/**
 * @brief Detension a motor
 * @param motor Pointer to the motor to detension
 */
void detensionMotor(AccelStepper* motor);

/**
 * @brief Get motor by its name
 * @param name String name of the motor (e.g., "EPU", "EPD", etc.)
 * @return Pointer to the AccelStepper motor instance, or nullptr if not found
 */
AccelStepper* getMotorByName(const String& name);

/**
 * @brief Get motor name by index
 * @param index Index in the motors array
 * @return const char* name of the motor
 */
const char* getMotorName(int index);

#endif // MOTORS_H