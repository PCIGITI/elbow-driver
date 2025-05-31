#include <AccelStepper.h>
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define MM_TO_STEPS(x) ((int)((x/0.5)*200))  // For non-EP/EY motors: 0.5mm per rev, 200 steps per rev

bool verbose_output = true;

AccelStepper EPU(AccelStepper::DRIVER, 41, 40);
AccelStepper EPD(AccelStepper::DRIVER, 49, 48);
AccelStepper EYL(AccelStepper::DRIVER, 39, 38); 
AccelStepper EYR(AccelStepper::DRIVER, 53, 52);
AccelStepper WPD(AccelStepper::DRIVER, 29, 28); 
AccelStepper WPU(AccelStepper::DRIVER, 35, 34); 
AccelStepper RJL(AccelStepper::DRIVER, 47, 46);
AccelStepper LJR(AccelStepper::DRIVER, 51, 50);
AccelStepper LJL(AccelStepper::DRIVER, 45, 44);
AccelStepper RJR(AccelStepper::DRIVER, 43, 42);
AccelStepper ROLL(AccelStepper::DRIVER, 37, 36);

float ey_pos = 90;  // Initial position in degrees
float ep_pos = 90;  // Initial position in degrees
float wp_pos = 90;  // Initial wrist pitch position in degrees

//THIS IS THE ORDER THAT THE MOTORS ARE ALWAYS REFERRED TO IN
//IT IS [Q1- Q1+ Q2- Q2+ Q3+ Q3- Q4+ Q4- Q5+ Q5- ROLL]
AccelStepper* motors[] = {&EPU, &EPD, &EYR, &EYL, &WPD, &WPU, &RJL, &LJR, &LJL, &RJR, &ROLL};

// Function declarations
//void moveLink(AccelStepper* m1, AccelStepper* m2, float steps);
void moveEY(float theta_deg);
void moveEP(float theta_deg);
void moveLJ(float theta_deg);
void moveRJ(float theta_deg);
float getEYAuxSteps(float theta_deg);
int getEYSteps(float delta_theta);
void moveWP(float delta_theta);
int getWPSteps(float delta_theta);
int getJSteps(float delta_theta);
void tensionMotor(AccelStepper* motor, bool coarse);
void detensionMotor(AccelStepper* motor);
int getEPSteps(float delta_theta);
void getEYLength(float theta_deg, float& shorter, float& longer);
void getEPLength(float theta_deg, float& shorter, float& longer);
unsigned long startTime;
unsigned long endTime;
void moveEPandEY(float pitch_delta, float yaw_delta);


//MOVING MOTORS ON LEAD SCREW CLOCKWISE (+VE STEPS) RESULTS IN CABLE LENGTH SHORTENING
//MOVING MOTORS ON LEAD SCREW COUNTERCLOCKWISE (-VE STEPS) RESULTS IN CABLE LENGTH LENGTHENING


void setup()
{  
  Serial.begin(9600);

  // Set acceleration for all motors
  for (int i = 0; i < 11; i++) {
    motors[i]->setAcceleration(1000000.0);  // Effectively no ramping
    motors[i]->setCurrentPosition(0);
    motors[i]->setMaxSpeed(700);
  }
}

// --- Test Motor State ---
bool test_mode_active = false;
int test_motor_index = 0;
const int NUM_TEST_MOTORS = 10;
const char* test_motor_names[NUM_TEST_MOTORS] = {"RJL", "RJR", "LJL", "LJR", "WPU", "WPD", "EYR", "EYL", "EPD", "EPU"};
AccelStepper* test_motors[NUM_TEST_MOTORS] = {&RJL, &RJR, &LJL, &LJR, &WPU, &WPD, &EYR, &EYL, &EPD, &EPU};

int handleTestMotorCommand(const String& command) {
  if (command == "START_TEST_MOTORS") {
    test_mode_active = true;
    test_motor_index = 9; 
    Serial.print("TEST_MOTOR_SELECTED:");
    Serial.println(test_motor_names[test_motor_index]);
  } else if (command.startsWith("SELECT_MOTOR:")) {
    String name = command.substring(13);
    for (int i = 0; i < NUM_TEST_MOTORS; i++) {
      if (name == test_motor_names[i]) {
        test_motor_index = i;
        Serial.print("TEST_MOTOR_SELECTED:");
        Serial.println(test_motor_names[test_motor_index]);
        break;
      }
    }
  } else if (command == "NEXT_MOTOR") {
    test_motor_index = (test_motor_index + 1) % NUM_TEST_MOTORS;
    Serial.print("TEST_MOTOR_SELECTED:");
    Serial.println(test_motor_names[test_motor_index]);
  } else if (command == "TEST_MOTOR_STEP") {
    // Move 5 steps clockwise
    test_motors[test_motor_index]->move(5);
    while (test_motors[test_motor_index]->distanceToGo() != 0) {
      test_motors[test_motor_index]->run();
    }
    Serial.println("TEST_MOTOR_STEP_DONE");
  } else if (command == "FINE_TENSION") {
    // Fine tension: move a small number of steps
    tensionMotor(test_motors[test_motor_index], false);
    Serial.println("FINE_TENSION_DONE");
  } else if (command == "COARSE_TENSION") {
    // Coarse tension: move a larger number of steps
    tensionMotor(test_motors[test_motor_index], true);
    Serial.println("COARSE_TENSION_DONE");
  } else if (command == "DETENSION") {
    detensionMotor(test_motors[test_motor_index]);
    Serial.println("DETENSION_DONE");
  } else if (command.startsWith("STEP_MOTOR_BY")){
    String stepsStr = command.substring(14); // Length of "STEP_MOTOR_BY:"
    long steps = stepsStr.toInt(); // Use long for steps, can be negative
    Serial.print("STEPPING BY");
    Serial.println(steps);
    test_motors[test_motor_index]->move(steps);
    while (test_motors[test_motor_index]->distanceToGo() != 0) {
        test_motors[test_motor_index]->run();
    }
    Serial.println("STEP_MOTOR_BY_DONE");
  }
  else if (command == "SET_HOME"){
    //set all the motors current position to be 0 
    for (int i = 0; i < NUM_TEST_MOTORS; i++) {
      test_motors[i]->setCurrentPosition(0);
    }
  }
  else if (command == "EXIT_TEST") {
    test_mode_active = false;
    Serial.println("TEST_MOTORS_EXITED");
  }
  else {
    return 0;
  }
  return 1;
}

void handleMoveAllMotors(String cmd){
  int NUM_MOTORS = 11;
  long parsedMotorSteps[NUM_MOTORS];
  cmd.trim(); // Remove any leading/trailing whitespace (like \r)
  Serial.print("Received: [");
  Serial.print(cmd);
  Serial.println("]");

  if (cmd.startsWith("MOVE_ALL_MOTORS:")) {
    // Extract the part of the string after "MOVE_ALL_MOTORS:"
    String stepsData = cmd.substring(cmd.indexOf(':') + 1);

    // Prepare for parsing using strtok_r (reentrant version of strtok)
    char dataBuffer[stepsData.length() + 1];
    stepsData.toCharArray(dataBuffer, sizeof(dataBuffer));

    char* token;
    char* context = NULL; // Context pointer for strtok_r
    int motorIndex = 0;

    // Get the first token
    token = strtok_r(dataBuffer, ",", &context);

    while (token != NULL && motorIndex < NUM_MOTORS) {
      parsedMotorSteps[motorIndex] = atol(token); // Convert string token to long integer
      motorIndex++;
      token = strtok_r(NULL, ",", &context); // Get the next token
    }

    // Check if we successfully parsed all expected values
    if (motorIndex == NUM_MOTORS) {
      Serial.println("Successfully parsed steps for all motors:");
      String motorNames[] = {"EPU", "EPD", "EYR", "EYL", "WPD", "WPU", "RJL", "LJR", "LJL", "RJR", "ROLL"};
      for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print("  Motor ");
        Serial.print(i < 11 ? motorNames[i] : "Unknown"); // Print name if available
        Serial.print(": ");
        Serial.println(parsedMotorSteps[i]);

        // --- Command the motor to move ---
        motors[i]->move(parsedMotorSteps[i]); // Tell motor to move by this many steps (relative)
        // If you wanted to move to an absolute position:
      }
      bool stillMoving = true;
      while(stillMoving){
        stillMoving = false;
        for (int i = 0; i < NUM_MOTORS; i++) {
          motors[i]->run(); // THIS IS CRUCIAL - it generates the steps
          if (motors[i]->isRunning()) {
            stillMoving = true; // At least one motor is still moving
          }
        }
      }
      Serial.println("Motor movement commands applied.");
    } else {
      Serial.print("Error: Failed to parse all motor steps. Parsed ");
      Serial.print(motorIndex);
      Serial.print(" values, expected ");
      Serial.println(NUM_MOTORS);
    }
  } else if (cmd.length() > 0) { // Handle other potential commands or unrecognized input
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }

}

void loop()
{
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (test_mode_active || command == "START_TEST_MOTORS") {
      if(handleTestMotorCommand(command) == 1){
        return;
      }
    }
    if (command.startsWith("MOVE_EP_EY_REL:")) {
      int commaIdx = command.indexOf(",", 15);
      if (commaIdx > 0) {
        float pitch_delta = command.substring(15, commaIdx).toFloat();
        float yaw_delta = command.substring(commaIdx + 1).toFloat();
        moveEPandEY(pitch_delta, yaw_delta);
        Serial.println("OK");
      } else {
        Serial.println("ERROR: Invalid MOVE_EP_EY_REL format");
      }
      return;
    }
    if (command.startsWith("MOVE_EP_REL:")) {
      float angle = command.substring(12).toFloat();
      moveEP(angle);
      Serial.println("OK");
    }
    else if (command.startsWith("MOVE_EY_REL:")) {
      float angle = command.substring(12).toFloat();
      moveEY(angle);
      Serial.println("OK");
    }
    else if (command.startsWith("MOVE_WP_REL:")) { // <<< IMPLEMENTED THIS
      float angle = command.substring(12).toFloat();
      moveWP(angle); // You need to define moveWP(float angle)
      Serial.println("OK");
      return;
    }
    else if (command.startsWith("MOVE_LJ_REL:")) { // <<< IMPLEMENTED THIS
      float angle = command.substring(12).toFloat();
      moveLJ(angle); // You need to define moveLJ(float angle)
      Serial.println("OK");
      return;
    }
    else if (command.startsWith("MOVE_RJ_REL:")) { // <<< IMPLEMENTED THIS
      float angle = command.substring(12).toFloat();
      moveRJ(angle); 
      Serial.println("OK");
      return;
    }
    else if (command == "SET_VERBOSE:1") {
      verbose_output = true;
      Serial.println("OK");
    }
    else if (command == "SET_VERBOSE:0") {
      verbose_output = false;
      Serial.println("OK");
    }
    else if (command.startsWith("MOVE_ALL_MOTORS:")){
      handleMoveAllMotors(command);
    }
    else {
      Serial.println("ERROR: Unknown command");
    }
  }
}

void moveEP(float theta_deg){
  float K = 1.4; //open loop gain
  float theta_rad = DEG_TO_RAD(theta_deg);  // Calculate change in angle
  float delta_s = theta_rad*3.24;
  int steps = MM_TO_STEPS(delta_s)*K;
  
  if (verbose_output){
    Serial.println("\nMovement values for verification:");
    Serial.print("Delta theta: ");
    Serial.println(theta_deg);
    Serial.print("Steps: ");
    Serial.println(steps);
  }
  
  EPU.move(steps);
  EPD.move(-steps);

  // Run motors until movement is complete
  while((EPU.isRunning() || EPD.isRunning())){
    EPU.run();
    EPD.run();
  }
  
  ep_pos += theta_deg;  // Update the stored position
}

void moveEY(float theta_deg){

  //yaw left --> lengthen jaw right cables, shorten jaw left cables
  //yaw right --> lengthen jaw left cables, shorten jaw right cables
  float K = 1.4; //open loop gain
  float theta_rad = DEG_TO_RAD(theta_deg);  // Calculate change in angle
  float delta_s = theta_rad*3.1;
  int steps = MM_TO_STEPS(delta_s)*K;
  int aux_steps = getEYAuxSteps(theta_deg);
  
  if (verbose_output){
    Serial.println("\nMovement values for verification:");
    Serial.print("Delta theta: ");
    Serial.println(theta_deg);
    Serial.print("Steps: ");
    Serial.println(steps);

    Serial.print("Aux steps ");
    Serial.println(aux_steps);
    
    delay(1500);
  }
  
  EYR.move(steps);
  EYL.move(-steps);
  LJR.move(aux_steps);
  RJR.move(aux_steps);
  LJL.move(-aux_steps);
  LJR.move(-aux_steps);

  // Run motors until movement is complete
  while((EYR.isRunning() || EYL.isRunning())){
    EYR.run();
    EYL.run();
    LJR.run();
    RJR.run();
    LJL.run();
    LJR.run();
  }
  
  ey_pos += theta_deg;  // Update the stored position
}

float getEYAuxSteps(float delta_theta){
  Serial.println("\nDebug getShorterPathEY:");
  Serial.print("Input theta (degrees): ");
  
  // Convert to radians at the start  
  float path_length_change = 0.6738*DEG_TO_RAD(delta_theta); //gives path length change in MM

  int steps = MM_TO_STEPS(path_length_change);

  //this is the legit function but we're gonna linearize it float result = 0.96*sin(theta-1.16)+1.9 + 0.222;
  return steps;
}

int getEYSteps(float delta_theta){
  Serial.println("\nDebug getEYSteps:");
  Serial.print("Input delta_theta (degrees): ");
  Serial.println(delta_theta);
  
  float delta_theta_rad = DEG_TO_RAD(delta_theta);
  Serial.print("delta_theta in radians: ");
  Serial.println(delta_theta_rad, 6);
  
  float delta_s = delta_theta_rad/1.1; //get delta_s in mm from radius and delta_theta in radians
  Serial.print("delta_s (mm): ");
  Serial.println(delta_s, 6);
  
  float revs = delta_s/23.40; //23.40mm per rev
  Serial.print("revolutions: ");
  Serial.println(revs, 6);
  
  int steps = (int)(revs * 1600 * 1.3);  // Convert revolutions to steps (1600 steps per rev)
  Serial.print("final steps: ");
  Serial.println(steps);

  return steps;
}

void moveWP(float delta_theta){
  float wp_steps = getWPSteps(delta_theta);  // Convert degrees to mm
  // Move all motors

  //to move the wrist when the jaw is tensioned we must also move the jaws:
  //for every one degree of wrist movement, cables change by delta_theta_rad*0.89 = 0.01553 mm 
  //for every 0.02967 mm of wrist pl change, jaw cables change by 0.01553 mm
  //for every 11.86 wrist steps i need 6.212 jaw steps

  //pitch up (+ve) --> lengthen right jaw cables
  //pitch down --> lengthen left jaw cables

  int jaw_steps = int(wp_steps * (6.212/11.86));

  WPD.move(-wp_steps);
  WPU.move(wp_steps);

  LJL.move(jaw_steps);
  LJR.move(jaw_steps);
  RJL.move(-jaw_steps);
  RJR.move(-jaw_steps);

  Serial.println(wp_steps);
  Serial.println(jaw_steps);
  // Run all motors until movement is complete
  while(WPD.isRunning() || WPU.isRunning()){
    WPD.run();
    WPU.run();
    LJL.run();
    LJR.run();
    RJL.run();
    RJR.run();
    // Add dependent cable motors here
  }

  wp_pos = wp_pos+delta_theta;  // Update the stored position after movement is complete
}

// Placeholder function to convert degrees to mm for wrist pitch
int getWPSteps(float delta_theta){
  float effective_radius = 1.7; //mm
  float delta_s = effective_radius*DEG_TO_RAD(delta_theta);
  int steps = MM_TO_STEPS(delta_s);
  // TODO: Implement actual mathematical function to convert degrees to mm
  //    THEN MAP MM TO STEPS!
  return steps;
}

void moveLJ(float delta_theta){
  float steps = getJSteps(delta_theta);  // Convert degrees to mm
  // Move all motors
  LJR.move(steps);
  LJL.move(-steps);
  // Add any dependent cable motors here when we know which ones they are

  // Run all motors until movement is complete
  while(LJL.isRunning() || LJR.isRunning()){
    LJR.run();
    LJL.run();
    // Add dependent cable motors here
  }
}

void moveRJ(float steps){
    // Move all motors
    RJR.move(steps);
    RJL.move(-steps);
    // Run all motors until movement is complete
    while(RJL.isRunning() || RJR.isRunning()){
      RJL.run();
      RJR.run();
    }
}

int getJSteps(float delta_theta){
  float effective_radius = 1.35; //mm
  float delta_s = effective_radius*DEG_TO_RAD(delta_theta);
  int steps = MM_TO_STEPS(delta_s);
  // TODO: Implement actual mathematical function to convert degrees to mm
  //    THEN MAP MM TO STEPS!
  return steps;
}

void tensionMotor(AccelStepper* motor, bool coarse) {
  float tensionSpeed;
  if (coarse) {
    tensionSpeed = 600.0;
    Serial.println("Coarse tensioning selected");
  } else {
    tensionSpeed = 50.0;
    Serial.println("Fine tensioning selected");
  }
  
  Serial.println("Tensioning motor... Press any key to stop");
  
  // Save current settings
  float currentMaxSpeed = motor->maxSpeed();
  float currentAccel = motor->acceleration();
  
  // Set up for tensioning
  motor->setMaxSpeed(tensionSpeed);
  motor->setAcceleration(1000000.0);  // Effectively no ramping
  motor->setSpeed(tensionSpeed);
  
  while (!Serial.available()) {
    motor->runSpeed();  // Run motor at constant speed
  }
  
  // Clear the serial buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  // Restore original settings
  motor->setMaxSpeed(currentMaxSpeed);
  motor->setAcceleration(currentAccel);
  motor->setSpeed(500);  // Stop the motor
  Serial.println("Tensioning stopped");
}

void detensionMotor(AccelStepper* motor) {
  Serial.println("Detensioning motor... Press any key to stop");
  
  // Save current settings
  float currentMaxSpeed = motor->maxSpeed();
  float currentAccel = motor->acceleration();
  
  // Set up for detensioning (opposite direction of tensioning)
  motor->setMaxSpeed(600.0);
  motor->setAcceleration(1000000.0);  // Effectively no ramping
  motor->setSpeed(-600.0);  // Negative speed for opposite direction
  
  while (!Serial.available()) {
    motor->runSpeed();  // Run motor at constant speed
  }
  
  // Clear the serial buffer
  while (Serial.available()) {
    Serial.read();
  }
  
  // Restore original settings
  motor->setMaxSpeed(currentMaxSpeed);
  motor->setAcceleration(currentAccel);
  motor->setSpeed(500);  // Stop the motor
  Serial.println("Detensioning stopped");
}

void getEPLength(float theta_deg, float& shorter, float& longer){
  float theta;
  if (theta_deg>90){
    theta = DEG_TO_RAD(180-theta_deg);
  }
  else{
    theta = DEG_TO_RAD(theta_deg);
  }
  
  float x1 = -1.45+3.185*cos(theta-0.6689);
  float y1 = 3.3+3.185*sin(theta-0.6689);
  shorter = sqrt(pow(x1,2)+pow(y1,2));
  
  float x2 = 1.45+1.89*cos(theta+1.43);
  float y2 = 3.3+1.89*sin(theta+1.43);
  longer = sqrt(pow(x2,2)+pow(y2,2)) + 2.2324;
}

int getEPSteps(float delta_theta){

  float delta_theta_rad = DEG_TO_RAD(delta_theta);
  float delta_s = delta_theta_rad/1.1; //get delta_s in mm from radius and delta_theta in radians
  Serial.print("delta_s (mm): ");
  Serial.println(delta_s, 6);
  
  float revs = delta_s/23.40; //23.40mm per rev
  Serial.print("revolutions: ");
  Serial.println(revs, 6);
  
  int steps = (int)(revs * 1600 * 1.3);  // Convert revolutions to steps (1600 steps per rev)
  Serial.print("final steps: ");
  Serial.println(steps);
  
  return steps;
}

void getEYLength(float theta_deg, float& shorter, float& longer){
  float theta;
  if (theta_deg>90){
    theta = DEG_TO_RAD(180-theta_deg);
  }
  else{
    theta = DEG_TO_RAD(theta_deg);
  }
  if(theta_deg<32){
    float x1 = -1.4+1.28*(cos(theta-0.2793));
    float y1 = 1.9+1.28*(sin(theta-0.2793));
    shorter = sqrt(pow(x1,2)+pow(y1,2));
  }
  else{
    float rc = 1.56/2;
    float rs = 0.5;
    float l_1 = sqrt(pow(1.4,2)+pow(1.9,2));
    float x_a = rc/2*(cos(theta-0.872))-1.4;
    float y_a = rc/2*(sin(theta-0.872))+1.9;
    float h = sqrt(pow(x_a,2)+pow(y_a,2));

    float len_line = sqrt(pow(h,2)+pow(rs,2));

    float beta = atan(1.9/-1.4)-atan(y_a/x_a);
    float lambda = asin((sin(beta)*l_1)/rc);
    float alpha = asin(rs/len_line)-lambda;
    float arc_length = alpha*rs;
    shorter = arc_length+len_line+0.1947;
  }

  longer = 1.1*(1.5708-theta)+2.5099;
}

void moveEPandEY(float pitch_delta, float yaw_delta) {
  // Move both pitch and yaw simultaneously
  float K = 1.4;
  float theta_rad_ep = DEG_TO_RAD(pitch_delta);
  float theta_rad_ey = DEG_TO_RAD(yaw_delta);
  float delta_s_ep = theta_rad_ep * 3.24;
  float delta_s_ey = theta_rad_ey * 3.1;
  int steps_ep = MM_TO_STEPS(delta_s_ep) * K;
  int steps_ey = MM_TO_STEPS(delta_s_ey) * K;

  // Pitch: EPU/EPD, Yaw: EYR/EYL
  EPU.move(steps_ep);
  EPD.move(-steps_ep);
  EYR.move(steps_ey);
  EYL.move(-steps_ey);

  // Run all motors until movement is complete
  while (EPU.isRunning() || EPD.isRunning() || EYR.isRunning() || EYL.isRunning()) {
    EPU.run();
    EPD.run();
    EYR.run();
    EYL.run();
  }
  ep_pos += pitch_delta;
  ey_pos += yaw_delta;
}

