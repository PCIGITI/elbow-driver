#include <AccelStepper.h>
#include "motors.h"


bool tension_active = false;
float tension_speed = 0;
AccelStepper* tension_motor = nullptr;

bool detension_active = false;
float detension_speed = 0;
AccelStepper* detension_motor = nullptr;

bool test_q_active = false;
int test_q_joint_num = 0;
int test_q_dir = 1;


bool verbose_output = true;
bool test_mode_active = false;
int dir = 1; 
AccelStepper* current_test_motor = nullptr; // Will update to motors[0] below
AccelStepper* handleSelectMotor(String name);
void stepTest(int jointNum);


void setup() {  
    Serial.begin(9600);
    initializeMotors();
}

int handleTestMotorCommand(const String& command) {
    Serial.print("Handling Test Motor Command");
    Serial.println(command);
    if (command == "START_TEST_MOTORS") {
        test_mode_active = true;
        current_test_motor = handleSelectMotor("EPU");
    } 
    else if (command.startsWith("SELECT_MOTOR:")) {
        String name = command.substring(13);
        current_test_motor = handleSelectMotor(name);
        // Always stop tensioning when switching motors
        tension_active = false;
        detension_active = false;
    } 
    else if (command == "FINE_TENSION") {
        if(tension_active){
            tension_active = false;
            if (current_test_motor != nullptr) {
            current_test_motor->setSpeed(0);
            }
        }    

        else{
            tension_motor = current_test_motor;
            tension_speed = TENSION_SPEED_FINE;
            tension_motor->setMaxSpeed(tension_speed);
            tension_motor->setAcceleration(DEFAULT_ACCELERATION);
            tension_motor->setSpeed(tension_speed);
            tension_active = true;
            detension_active = false; // safety
            Serial.println("FINE_TENSION_RUNNING");
        }
    } 
    else if (command == "COARSE_TENSION") {
        if(tension_active){
            tension_active = false;
            if (current_test_motor != nullptr) {
            current_test_motor->stop();
        }
        Serial.println("TENSION_STOPPED");
        }
        else{
            tension_motor = current_test_motor;
            tension_speed = TENSION_SPEED_COARSE;
            tension_motor->setMaxSpeed(tension_speed);
            tension_motor->setAcceleration(DEFAULT_ACCELERATION);
            tension_motor->setSpeed(tension_speed);
            tension_active = true;
            detension_active = false; // safety
            Serial.println("COARSE_TENSION_RUNNING");
        }
    } 
    else if (command == "DETENSION") {
        if(detension_active){
            detension_active = false;
            if (current_test_motor != nullptr) {
            current_test_motor->stop();
        }
        Serial.println("TENSION_STOPPED");
        }
        else{
            detension_motor = current_test_motor;
            detension_speed = TENSION_SPEED_COARSE;
            detension_motor->setMaxSpeed(detension_speed);
            detension_motor->setAcceleration(DEFAULT_ACCELERATION);
            detension_motor->setSpeed(-detension_speed);
            detension_active = true;
            tension_active = false; // safety
            Serial.println("DETENSION_RUNNING");
        }
    } 
    else if (command == "STOP_TENSION") {
        tension_active = false;
        detension_active = false;
        if (current_test_motor != nullptr) {
            current_test_motor->stop();
        }
        Serial.println("TENSION_STOPPED");
    } 
    else if (command.startsWith("STEP_MOTOR_BY")) {
        String stepsStr = command.substring(14);
        int steps = stepsStr.toInt();
        Serial.print("STEPPING BY ");
        Serial.println(steps);
        current_test_motor->move(steps);
        while (current_test_motor->distanceToGo() != 0) {
            current_test_motor->run();
        }
        Serial.println("STEP_MOTOR_BY_DONE");
    }
    else if (command == "SET_HOME") {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i]->setCurrentPosition(0);
        }
        Serial.println("HOME_SET");
    }
    else if (command.startsWith("START_TEST_Q:")) {
        String jointStr = command.substring(14);
        test_q_joint_num = jointStr.toInt();
        test_q_active = true;
        test_q_dir = test_q_dir * -1; // Flip direction each time
        Serial.println("START_TEST_Q_RUNNING");
    }
    else if (command == "STOP_TEST_Q") {
        test_q_active = false;
        Serial.println("STOP_TEST_Q_DONE");
    }
    else if (command == "EXIT_TEST") {
        test_mode_active = false;
        tension_active = false;
        detension_active = false;
        test_q_active = false;
        Serial.println("TEST_MOTORS_EXITED");
    }
    else {
        return 0;
    }
    return 1;
}

AccelStepper* handleSelectMotor(String name){
    AccelStepper* motor = getMotorByName(name);
    if (motor != nullptr) {
        Serial.print("TEST_MOTOR_SELECTED:");
        Serial.println(name);
        return motor;
    } 
    Serial.print("ERROR: ATTEMPTED TO SELECT NULL MOTOR");
    return nullptr;
}

void stepTest(int jointNum){
    dir = dir * -1;
    int motor_pair[2] = {0, 0}; // Declare motor_pair as an array of 2 integers

    if (jointNum == 4) {
        //motor_pair[0] = 6;
        //motor_pair[1] = 9;
        motor_pair[0] = 7;
        motor_pair[1] = 8;

    } else if (jointNum == 3) {
        motor_pair[0] = 4;
        motor_pair[1] = 5; // Make sure there's a semicolon here
    }
    else if (jointNum == 2) {
        motor_pair[0] = 2;
        motor_pair[1] = 3; // Make sure there's a semicolon here
    }
    else if (jointNum == 1) {
        motor_pair[0] = 0;
        motor_pair[1] = 1; // Make sure there's a semicolon here
    }
        long startPosLJL = motors[motor_pair[0]]->currentPosition();
        long startPosLJR = motors[motor_pair[1]]->currentPosition();

        motors[motor_pair[0]]->setSpeed(dir*30);
        motors[motor_pair[1]]->setSpeed(dir*-30);

        Serial.println("START_TEST_Q_RUNNING");

        unsigned long startTime = millis();

        while (Serial.available() == 0) {
            motors[motor_pair[0]]->runSpeed();
            motors[motor_pair[1]]->runSpeed();
            delay(1);
        }

        unsigned long endTime = millis();
        long endPosLJL = motors[motor_pair[0]]->currentPosition();
        long endPosLJR = motors[motor_pair[1]]->currentPosition();

        motors[motor_pair[0]]->setSpeed(0);
        motors[motor_pair[1]]->setSpeed(0);

        long stepsLJL = endPosLJL - startPosLJL;
        long stepsLJR = endPosLJR - startPosLJR;
        unsigned long duration = endTime - startTime;

        Serial.print("TEST_Q_DONE;duration_ms:");
        Serial.print(duration);
        Serial.print(";steps_LJL:");
        Serial.print(stepsLJL);
        Serial.print(";steps_LJR:");
        Serial.println(stepsLJR);    
}

void handleMoveAllMotors(String cmd) {
    long motorSteps[NUM_MOTORS] = {0};
    cmd.trim();

    if (cmd.startsWith("MOVE_ALL_MOTORS:")) {
        String stepsData = cmd.substring(cmd.indexOf(':') + 1);
        char dataBuffer[stepsData.length() + 1];
        stepsData.toCharArray(dataBuffer, sizeof(dataBuffer));
        
        char* token = strtok(dataBuffer, ",");
        int motorIndex = 0;
        
        while (token != NULL && motorIndex < NUM_MOTORS) {
            motorSteps[motorIndex] = atol(token);
            motorIndex++;
            token = strtok(NULL, ",");
        }

        if (motorIndex == NUM_MOTORS) {
            if (verbose_output) {
                Serial.println("Moving motors with steps:");
                for (int i = 0; i < NUM_MOTORS; i++) {
                    Serial.print("Motor ");
                    Serial.print(getMotorName(i));
                    Serial.print(": ");
                    Serial.println(motorSteps[i]);
                }
            }

            // Command all motors to move
            for (int i = 0; i < NUM_MOTORS; i++) {
                // Calculate the new ABSOLUTE target position for this motor
                long newTarget = motors[i]->currentPosition() + motorSteps[i];
                
                // Command the motor to go to that exact new target position.
                // This ignores any previous/stale targets.
                motors[i]->moveTo(newTarget);
            }

            // Run all motors until movement is complete
            bool stillMoving = true;
            //unsigned long lastPrintTime = millis();
            //Serial.println("check 1");
            while (stillMoving) {
                //Serial.println("Check 2");
                stillMoving = false;
                for (int i = 0; i < NUM_MOTORS; i++) {
                    motors[i]->run();
                    if (motors[i]->distanceToGo() != 0) {
                        stillMoving = true;
                        if (motors[i]->distanceToGo() != 0) {
                        //Serial.print("Still waiting for motor to finish:");
                        //Serial.print(getMotorName(i));
                        //Serial.println(motors[i]->distanceToGo());
                        stillMoving = true;
                        }
                    }
                }
            }
            Serial.println("Motor movement commands applied.");
        }
    }
}

void runTestQMotor() {
    int motor_pair[2] = {0, 0};

    if (test_q_joint_num == 4) {
        motor_pair[0] = 7;
        motor_pair[1] = 8;
    } else if (test_q_joint_num == 3) {
        motor_pair[0] = 4;
        motor_pair[1] = 5;
    } else if (test_q_joint_num == 2) {
        motor_pair[0] = 2;
        motor_pair[1] = 3;
    } else if (test_q_joint_num == 1) {
        motor_pair[0] = 0;
        motor_pair[1] = 1;
    }

    motors[motor_pair[0]]->setSpeed(test_q_dir * 30);
    motors[motor_pair[1]]->setSpeed(test_q_dir * -30);

    motors[motor_pair[0]]->runSpeed();
    motors[motor_pair[1]]->runSpeed();
}


String serialBuffer = "";

void loop() {
    while (Serial.available() > 0) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n') {
            // Full command received
            serialBuffer.replace("\r", "");
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                if (verbose_output){
                    Serial.print("Arduino Recieved:");
                    Serial.println(serialBuffer);
                }

                if (test_mode_active || serialBuffer == "START_TEST_MOTORS") {
                    if(handleTestMotorCommand(serialBuffer) == 0){
                        Serial.println("Command not processed because test mode is active or command is unrecognized. Turning test motors off:");
                        test_mode_active = false;
                    }
                }
                else if(serialBuffer.startsWith("MOVE_ALL_MOTORS:")) {
                        handleMoveAllMotors(serialBuffer);
                    }
                else if (serialBuffer == "TOGGLE_VERBOSE") {
                    verbose_output = !verbose_output;
                    Serial.print("VERBOSE_STATE:");
                    Serial.println(verbose_output ? "1" : "0");
                }
                else if (serialBuffer.startsWith("START_TEST_Q")){
                    stepTest(1);
                }
                else{
                    Serial.println("UNRECOGNIZED COMMAND");
                }
                
                serialBuffer = ""; // Clear buffer for next command
            }
        }
            
        else {
            serialBuffer += incomingChar;
        }
    }

    // Run tension motor if active
    if (tension_active && tension_motor != nullptr) {
        tension_motor->runSpeed();
    }

    // Run detension motor if active
    if (detension_active && detension_motor != nullptr) {
        detension_motor->runSpeed();
    }

    // Run test Q motor if active
    if (test_q_active) {
        runTestQMotor();
    }
}

