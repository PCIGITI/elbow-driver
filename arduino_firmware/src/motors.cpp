#include "motors.h"

// Motor instances definitions
AccelStepper EPU_motor(AccelStepper::DRIVER, PIN_EPU_STEP, PIN_EPU_DIR);
AccelStepper EPD_motor(AccelStepper::DRIVER, PIN_EPD_STEP, PIN_EPD_DIR);
AccelStepper EYL_motor(AccelStepper::DRIVER, PIN_EYL_STEP, PIN_EYL_DIR);
AccelStepper EYR_motor(AccelStepper::DRIVER, PIN_EYR_STEP, PIN_EYR_DIR);
AccelStepper WPD_motor(AccelStepper::DRIVER, PIN_WPD_STEP, PIN_WPD_DIR);
AccelStepper WPU_motor(AccelStepper::DRIVER, PIN_WPU_STEP, PIN_WPU_DIR);
AccelStepper RJL_motor(AccelStepper::DRIVER, PIN_RJL_STEP, PIN_RJL_DIR);
AccelStepper LJR_motor(AccelStepper::DRIVER, PIN_LJR_STEP, PIN_LJR_DIR);
AccelStepper LJL_motor(AccelStepper::DRIVER, PIN_LJL_STEP, PIN_LJL_DIR);
AccelStepper RJR_motor(AccelStepper::DRIVER, PIN_RJR_STEP, PIN_RJR_DIR);
AccelStepper ROLL_motor(AccelStepper::DRIVER, PIN_ROLL_STEP, PIN_ROLL_DIR);

// Motor array definition using enum order
AccelStepper* motors[NUM_MOTORS] = {
    &EPU_motor, &EPD_motor,
    &EYR_motor, &EYL_motor, // Corrected (ensure this order matches your MotorIndex enum and MOTOR_NAMES)
    &WPU_motor, &WPD_motor, // Corrected
    &RJL_motor, &LJR_motor, // Corrected
    &LJL_motor, &RJR_motor, // Corrected
    &ROLL_motor             // Corrected
};

// Lookup table for motor names
const char* const MOTOR_NAMES[NUM_MOTORS] = {
    "EPU", "EPD",
    "EYR", "EYL",
    "WPU", "WPD",
    "RJL", "LJR",
    "LJL", "RJR",
    "ROLL"
};

void initializeMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i]->setAcceleration(DEFAULT_ACCELERATION);
        motors[i]->setCurrentPosition(0);
        motors[i]->setMaxSpeed(DEFAULT_MAX_SPEED);
    }
}

/*void tensionMotor(AccelStepper* motor, bool coarse) {
    float tensionSpeed = coarse ? TENSION_SPEED_COARSE : TENSION_SPEED_FINE;
    Serial.println(coarse ? "Coarse tensioning selected" : "Fine tensioning selected");
    Serial.println("Tensioning motor... Press any key to stop");
    
    float currentMaxSpeed = motor->maxSpeed();
    float currentAccel = motor->acceleration();
    
    motor->setMaxSpeed(tensionSpeed);
    motor->setAcceleration(DEFAULT_ACCELERATION);
    motor->setSpeed(tensionSpeed);
    
    while (!Serial.available()) {
        motor->runSpeed();
    }
    
    while (Serial.available()) {
        Serial.read();
    }
    
    motor->setMaxSpeed(currentMaxSpeed);
    motor->setAcceleration(currentAccel);
    motor->setSpeed(0);
    Serial.println("Tensioning stopped");
}

void detensionMotor(AccelStepper* motor) {
    Serial.println("Detensioning motor... Press any key to stop");
    
    float currentMaxSpeed = motor->maxSpeed();
    float currentAccel = motor->acceleration();
    
    motor->setMaxSpeed(TENSION_SPEED_COARSE);
    motor->setAcceleration(DEFAULT_ACCELERATION);
    motor->setSpeed(-TENSION_SPEED_COARSE);
    
    while (!Serial.available()) {
        motor->runSpeed();
    }
    
    while (Serial.available()) {
        Serial.read();
    }
    
    motor->setMaxSpeed(currentMaxSpeed);
    motor->setAcceleration(currentAccel);
    motor->setSpeed(0);
    Serial.println("Detensioning stopped");
}*/

AccelStepper* getMotorByName(const String& name) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (name == MOTOR_NAMES[i]) {
            return motors[i];
        }
    }
    Serial.println("ERROR: Invalid motor name");
    return nullptr;
}

const char* getMotorName(int index) {
    if (index >= 0 && index < NUM_MOTORS) {
        return MOTOR_NAMES[index];
    }
    return "Unknown Motor";
}