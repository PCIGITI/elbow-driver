#include <AccelStepper.h>
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define MM_TO_STEPS(x) ((int)((x/0.5)*200))  // For non-EP/EY motors: 0.5mm per rev, 200 steps per rev

bool verbose_output = true;

//motor 1 --> elbow pitch down, pins 41, 40
//motor 2 --> elbow pitch up, pins 49, 48
//motor 3 --> elbow yaw right, pins 39, 38
//motor 4 --> elbow yaw left, pins 53, 54


AccelStepper EPD(AccelStepper::DRIVER, 41, 40);
AccelStepper EPU(AccelStepper::DRIVER, 49, 48);
AccelStepper EYR(AccelStepper::DRIVER, 39, 38);
AccelStepper EYL(AccelStepper::DRIVER, 53, 52);
AccelStepper WPU(AccelStepper::DRIVER, 29, 28); //STILL HAVE TO CONNECT THIS
AccelStepper WPD(AccelStepper::DRIVER, 35, 34); //STILL HAVE TO CONNECT THIS
AccelStepper LJR(AccelStepper::DRIVER, 45, 44);
AccelStepper LJL(AccelStepper::DRIVER, 51, 50);
AccelStepper RJR(AccelStepper::DRIVER, 47, 46);
AccelStepper RJL(AccelStepper::DRIVER, 43, 42);
AccelStepper ROLL(AccelStepper::DRIVER, 37, 36);

float ey_pos = 90;  // Initial position in degrees
float ep_pos = 90;  // Initial position in degrees
float wp_pos = 90;  // Initial wrist pitch position in degrees
AccelStepper* motors[] = {&EPD, &EPU, &EYR, &EYL, &WPU, &WPD, &LJR, &LJL, &RJR, &RJL, &ROLL};

// Function declarations
//void moveLink(AccelStepper* m1, AccelStepper* m2, float steps);
void moveEY(float theta_deg);
void moveEP(float theta_deg);
void moveEYTo(float abs_theta);
void moveEPTo_EZ(float abs_theta);
float getLongerPathEY(float theta_deg);
float getShorterPathEY(float theta_deg);
int getEYSteps(float delta_theta);
void moveWPTo(float abs_theta);
float getWPSteps(float delta_theta);
float getLongerPathWP(float theta);
float getShorterPathWP(float theta);
void detension();
void testMotors();
void tensionMotor(AccelStepper* motor);
void detensionMotor(AccelStepper* motor);
void moveEPToTest(float abs_theta);
int getEPSteps(float delta_theta);
void moveEYToTest(float abs_theta);
void getEYLength(float theta_deg, float& shorter, float& longer);
void getEPLength(float theta_deg, float& shorter, float& longer);
void manualControl();
void elbowTest();
unsigned long startTime;
unsigned long endTime;

//MOVING MOTORS ON LEAD SCREW CLOCKWISE (+VE STEPS) RESULTS IN CABLE LENGTH SHORTENING
//MOVING MOTORS ON LEAD SCREW COUNTERCLOCKWISE (-VE STEPS) RESULTS IN CABLE LENGTH LENGTHENING
void setup()
{  
  Serial.begin(9600);
  
  // Set speeds for all motors
  EPD.setMaxSpeed(300.0);
  EPU.setMaxSpeed(300.0);
  EYR.setMaxSpeed(300.0);
  EYL.setMaxSpeed(300.0);
  WPU.setMaxSpeed(300.0);
  WPD.setMaxSpeed(300.0);
  RJL.setMaxSpeed(300.0);
  RJR.setMaxSpeed(300.0);
  LJL.setMaxSpeed(300.0);
  LJR.setMaxSpeed(300.0);
  ROLL.setMaxSpeed(300.0);

  // Set acceleration for all motors
  for (int i = 0; i < 11; i++) {
    motors[i]->setAcceleration(1000000.0);  // Effectively no ramping
    motors[i]->setCurrentPosition(0);
  }
  
  //detension();
  Serial.println("Starting motor test sequence...");
  testMotors();
  Serial.println("Motor testing complete. Beginning normal operation...");
}

void manualControl() {
  verbose_output = true;
  
  Serial.println("\nSelect movement type:");
  Serial.println("1 - Elbow Pitch");
  Serial.println("2 - Elbow Yaw");
  Serial.println("Enter choice (1 or 2):");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  char choice = Serial.read();
  while (Serial.available()) {
    Serial.read(); // Clear any additional characters
  }
  
  // Prompt for target angle
  Serial.println("\nEnter movement angle. Positive values wind up and to the left, negative values wind down and to the right:");
  delay(1000);
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  String input = Serial.readStringUntil('\n');
  float target_theta = input.toFloat();
  
  if (target_theta >= -90 && target_theta <= 90) {
    if (choice == '1') {
      moveEP(target_theta);
    } else if (choice == '2') {
      moveEY(target_theta);
    } else {
      Serial.println("Invalid choice. Please enter 1 or 2.");
    }
  } else {
    Serial.println("Invalid angle. Please enter a value between 0 and 180 degrees.");
  }
}

void loop()
{
  manualControl();
  //detension();
  //elbowTest();
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
    
    Serial.println("\nProceed with movement? (y/n)");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    
    char input = Serial.read();
    while (Serial.available()) {
      Serial.read(); // Clear any additional characters
    }
    
    if (input != 'y' && input != 'Y') {
      Serial.println("Movement cancelled.");
      return;
    }
  }
  
  EPU.move(steps);
  EPD.move(-steps);

  // Run motors until movement is complete
  while((EPU.isRunning() || EPD.isRunning())){
    EPU.run();
    EPD.run();
  }
}

void moveEY(float theta_deg){
  float K = 1.4; //open loop gain
  float theta_rad = DEG_TO_RAD(theta_deg);  // Calculate change in angle
  float delta_s = theta_rad*3.1;
  int steps = MM_TO_STEPS(delta_s)*K;
  if (verbose_output){
    Serial.println("\nMovement values for verification:");
    Serial.print("Delta theta: ");
    Serial.println(theta_deg);
    Serial.print("Steps: ");
    Serial.println(steps);
    
    Serial.println("\nProceed with movement? (y/n)");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    
    char input = Serial.read();
    while (Serial.available()) {
      Serial.read(); // Clear any additional characters
    }
    
    if (input != 'y' && input != 'Y') {
      Serial.println("Movement cancelled.");
      return;
    }
  }
  
  EYR.move(steps);
  EYL.move(-steps);

  // Run motors until movement is complete
  while((EYR.isRunning() || EYL.isRunning())){
    EYR.run();
    EYL.run();
  }
}

void elbowTest() {
  unsigned long startTime;
  unsigned long endTime;
  verbose_output = false;

  Serial.println("Start test?");
  while (!Serial.available()) {
    delay(100);
  }
  Serial.read(); // Clear the input character
  
  Serial.println("Enter elbow test mode");
  Serial.println("Begin Aurora Recording in 3 seconds...");
  Serial.print("3...");
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Recording...");
  startTime = millis();

  // Initial position is 90, log it.
  // moveEPToTest(90); // Call if you need to ensure it's actively set, even if assumed.
  Serial.print("90, ");
  Serial.println(millis() - startTime);
  // If moveEPToTest(90) was called above, you might add a small delay here
  // if the function returns before movement is complete.
  // delay(100); // Example small delay

  // Upward step sequence: 90-100-90, 90-110-90, ..., 90-140-90
  int upwardSteps[] = {100, 110, 120, 130, 140};
  for (int i = 0; i < 5; i++) { // Loop 5 times for the 5 target angles
    // Move to target angle
    moveEPToTest(upwardSteps[i]);
    Serial.print(upwardSteps[i]);
    Serial.print(", ");
    Serial.println(millis() - startTime);
    delay(1000); // Optional delay after reaching target

    // Move back to 90
    moveEPToTest(90);
    Serial.print("90, ");
    Serial.println(millis() - startTime);
    delay(1000); // Optional delay after returning to base
  }

  // Downward step sequence: 90-80-90, 90-70-90, ..., 90-40-90
  // (Assuming a similar pattern and range downwards from 90)
  int downwardSteps[] = {80, 70, 60, 50, 40}; // Adjust if your downward range is different
  for (int i = 0; i < 5; i++) { // Loop 5 times for the 5 target angles
    // Move to target angle
    moveEPToTest(downwardSteps[i]);
    Serial.print(downwardSteps[i]);
    Serial.print(", ");
    Serial.println(millis() - startTime);
    delay(1000); // Optional delay after reaching target

    // Move back to 90
    moveEPToTest(90);
    Serial.print("90, ");
    Serial.println(millis() - startTime);
    delay(1000); // Optional delay after returning to base
  }

  endTime = millis();
  // Serial.println("stop"); // Optional: can be sent to your recording software
  Serial.flush();
  Serial.print("Time taken: ");
  Serial.println(endTime - startTime);
}

/*void elbowTest(){
  Serial.println("Start test?");
  while (!Serial.available()) {
    delay(100);
  }
  Serial.read(); // Clear the input character
  
  Serial.println("Enter elbow test mode");
  Serial.println("Begin Aurora Recording in 3 seconds...");
  Serial.print("3...");
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Recording...");
  
  // Wait for any character input

  // Move through test sequence
  startTime = millis();
  Serial.print("90, ");
  Serial.println(millis()-startTime);
  for(int j=0; j<10; j++){
    for (int i = 0; i < 25; i++){
      moveEPToTest(90+i);
      Serial.print(90+i);
      Serial.print(", ");
      Serial.println(millis()-startTime);
    }
    for (int i = 0; i<25 ; i++){
      moveEPToTest(115-i);
      Serial.print(115-i);
      Serial.print(", ");
      Serial.println(millis()-startTime);
    }
  }
  endTime= millis();
  Serial.println("stop");
  Serial.flush();
  Serial.print("Time taken: ");
  Serial.println(endTime-startTime);
}*/

void moveEYTo(float abs_theta){

  float curr_jaw_right_PL, curr_jaw_left_PL;
  float dest_jaw_right_PL, dest_jaw_left_PL;

  float delta_theta = ey_pos-abs_theta;  // Calculate change in angle
  float ey_steps = getEYSteps(delta_theta);
  
  if(ey_pos<90){
    curr_jaw_right_PL = getLongerPathEY(ey_pos);
    curr_jaw_left_PL = getShorterPathEY(ey_pos);
  }
  else{
    curr_jaw_right_PL = getShorterPathEY(ey_pos);
    curr_jaw_left_PL =  getLongerPathEY(ey_pos); 
  }

  if(abs_theta<90){
    dest_jaw_right_PL = getLongerPathEY(abs_theta);
    dest_jaw_left_PL = getShorterPathEY(abs_theta);
  }
  else{
    dest_jaw_right_PL = getShorterPathEY(abs_theta);
    dest_jaw_left_PL =  getLongerPathEY(abs_theta); 
  }

  float delta_jaw_right = curr_jaw_right_PL-dest_jaw_right_PL;
  int jaw_right_steps = MM_TO_STEPS(delta_jaw_right);
  float delta_jaw_left = curr_jaw_left_PL-dest_jaw_left_PL;
  int jaw_left_steps = MM_TO_STEPS(delta_jaw_left);

  // Print essential movement values
  Serial.println("\nMovement values for verification:");
  Serial.print("Initial theta: ");
  Serial.println(ey_pos);
  Serial.print("Final theta: ");
  Serial.println(abs_theta);
  Serial.print("Delta theta: ");
  Serial.println(delta_theta);
  Serial.print("Delta jaw right (mm): ");
  Serial.println(delta_jaw_right, 3);
  Serial.print("Delta jaw left (mm): ");
  Serial.println(delta_jaw_left, 3);
  Serial.print("Elbow yaw steps: ");
  Serial.println(ey_steps);
  
  Serial.println("\nProceed with movement? (y/n)");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  char input = Serial.read();
  while (Serial.available()) {
    Serial.read(); // Clear any additional characters
  }
  
  if (input != 'y' && input != 'Y') {
    Serial.println("Movement cancelled.");
    return;
  }

  RJR.move(jaw_right_steps);
  LJR.move(jaw_right_steps);
  RJL.move(jaw_left_steps);
  LJL.move(jaw_left_steps);
  EYR.move(ey_steps);
  EYL.move(ey_steps);

  // All cables will now move at the same physical speed
  while((RJR.isRunning() || LJR.isRunning() || RJL.isRunning() || LJL.isRunning() || EYR.isRunning() || EYL.isRunning())){
    RJR.run();
    LJR.run();
    RJL.run();
    LJL.run();
    EYR.run();
    EYL.run();
  }

  ey_pos = abs_theta;  // Update the stored position after movement is complete
}

void moveEYToTest(float abs_theta){


  float delta_theta = ey_pos-abs_theta;  // Calculate change in angle

  float shorter_initial, longer_initial;
  getEYLength(ey_pos, shorter_initial, longer_initial);
  
  float yaw_right_initial, yaw_left_initial;
  if (ey_pos<90){
    yaw_right_initial = shorter_initial;
    yaw_left_initial = longer_initial;
  }
  else{
    yaw_right_initial = longer_initial;
    yaw_left_initial = shorter_initial;
  }

  float shorter_final, longer_final;
  getEYLength(abs_theta, shorter_final, longer_final);
  
  float yaw_right_final, yaw_left_final;
  if (abs_theta<90){
    yaw_right_final = shorter_final;
    yaw_left_final = longer_final;
  }
  else{
    yaw_right_final = longer_final;
    yaw_left_final = shorter_final;
  }

  float delta_yaw_right_pl = yaw_right_initial - yaw_right_final;
  float delta_yaw_left_pl = yaw_left_initial - yaw_left_final;

  float yaw_right_steps = MM_TO_STEPS(delta_yaw_right_pl);
  float yaw_left_steps = MM_TO_STEPS(delta_yaw_left_pl);

  if(verbose_output){
    // Print essential movement values
    Serial.println("\nMovement values for verification:");
    Serial.print("Initial theta: ");
    Serial.println(ey_pos);
    Serial.print("Final theta: ");
    Serial.println(abs_theta);
    Serial.print("Delta theta: ");
    Serial.println(delta_theta);
    Serial.print("Elbow yaw right steps: ");
    Serial.println(yaw_right_steps);
    Serial.print("Elbow yaw left steps: ");
    Serial.println(yaw_left_steps);
    
    Serial.println("\nProceed with movement? (y/n)");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    
    char input = Serial.read();
    while (Serial.available()) {
      Serial.read(); // Clear any additional characters
    }
    
    if (input != 'y' && input != 'Y') {
      Serial.println("Movement cancelled.");
      return;
    }
  }

  // Calculate the ratio of steps to determine speed scaling
  float max_steps = max(abs(yaw_right_steps), abs(yaw_left_steps));
  float speed_ratio_right = abs(yaw_right_steps) / max_steps;
  float speed_ratio_left = abs(yaw_left_steps) / max_steps;

  // Set speeds proportionally to the step counts
  EYR.setMaxSpeed(100.0 * speed_ratio_right);
  EYL.setMaxSpeed(100.0 * speed_ratio_left);

  //set the EYR and EYL motors
  EYR.move(yaw_right_steps);
  EYL.move(yaw_left_steps);

  // Run motors until movement is complete
  while((EYR.isRunning() || EYL.isRunning())){
    EYR.run();
    EYL.run();
  }

  // Reset speeds to default
  EYR.setMaxSpeed(100.0);
  EYL.setMaxSpeed(100.0);

  ey_pos = abs_theta;  // Update the stored position after movement is complete
}

float getShorterPathEY(float theta_deg){
  Serial.println("\nDebug getShorterPathEY:");
  Serial.print("Input theta (degrees): ");
  Serial.println(theta_deg);
  
  // Convert to radians at the start
  float theta = DEG_TO_RAD(theta_deg);
  if(theta_deg>90){
    theta = DEG_TO_RAD(180-theta_deg);
  }
  
  // Add 0.222 to compensate for slightly different reference frame in geometry calculations
  float result = 0.96*sin(theta-1.16)+1.9 + 0.222;  // 66.45 degrees = 1.16 radians
  Serial.print("Final result: ");
  Serial.println(result);
  
  return result;
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

void moveWPTo(float abs_theta){
  float curr_cable1_PL, curr_cable2_PL;
  float dest_cable1_PL, dest_cable2_PL;

  float delta_theta = abs_theta - wp_pos;  // Calculate change in angle
  float wp_steps = getWPSteps(delta_theta);  // Convert degrees to mm
  
  // Calculate current cable lengths based on current angle
  if(wp_pos < 90){
    curr_cable1_PL = getLongerPathWP(wp_pos);
    curr_cable2_PL = getShorterPathWP(wp_pos);
  }
  else{
    curr_cable1_PL = getShorterPathWP(wp_pos);
    curr_cable2_PL = getLongerPathWP(wp_pos); 
  }

  // Calculate desired cable lengths based on target angle
  if(abs_theta < 90){
    dest_cable1_PL = getLongerPathWP(abs_theta);
    dest_cable2_PL = getShorterPathWP(abs_theta);
  }
  else{
    dest_cable1_PL = getShorterPathWP(abs_theta);
    dest_cable2_PL = getLongerPathWP(abs_theta); 
  }

  // Calculate required cable length changes
  float delta_cable1 = dest_cable1_PL - curr_cable1_PL;
  int cable1_steps = MM_TO_STEPS(delta_cable1);
  float delta_cable2 = dest_cable2_PL - curr_cable2_PL;
  int cable2_steps = MM_TO_STEPS(delta_cable2);

  // Move all motors
  WPD.move(wp_steps);
  WPU.move(wp_steps);
  // Add any dependent cable motors here when we know which ones they are

  // Run all motors until movement is complete
  while(WPD.isRunning() || WPU.isRunning()){
    WPD.run();
    WPU.run();
    // Add dependent cable motors here
  }

  wp_pos = abs_theta;  // Update the stored position after movement is complete
  Serial.print("Wrist Pitch moved to: ");
  Serial.println(abs_theta);
}

// Placeholder function to convert degrees to mm for wrist pitch
float getWPSteps(float delta_theta){
  // TODO: Implement actual mathematical function to convert degrees to mm
  //    THEN MAP MM TO STEPS!
  return 0.0;
}

// Placeholder functions for wrist pitch cable length calculations
float getLongerPathWP(float theta){
  // TODO: Implement actual mathematical function
  return 0.0;
}

float getShorterPathWP(float theta){
  // TODO: Implement actual mathematical function
  return 0.0;
}

void detension() {
  Serial.println("Detensioning cables...");
  
  // Set speeds for all motors (negative for counterclockwise)
  RJR.setSpeed(-100.0);
  RJL.setSpeed(-100.0);
  LJR.setSpeed(-100.0);
  LJL.setSpeed(-100.0);
  WPD.setSpeed(-100.0);
  WPU.setSpeed(-100.0);
  
  // Run motors for 2 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    RJR.runSpeed();
    RJL.runSpeed();
    LJR.runSpeed();
    LJL.runSpeed();
    WPD.runSpeed();
    WPU.runSpeed();
  }
  
  // Stop all motors
  RJR.setSpeed(0);
  RJL.setSpeed(0);
  LJR.setSpeed(0);
  LJL.setSpeed(0);
  WPD.setSpeed(0);
  WPU.setSpeed(0);
  
  Serial.println("Detensioning complete");
}

void tensionMotor(AccelStepper* motor) {
  Serial.println("Select tension mode:");
  Serial.println("f - Fine tension (50 speed)");
  Serial.println("c - Coarse tension (600 speed)");
  
  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  
  char mode = Serial.read();
  while (Serial.available()) {
    Serial.read(); // Clear any additional characters
  }
  
  float tensionSpeed;
  if (mode == 'c' || mode == 'C') {
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
  motor->setSpeed(0);  // Stop the motor
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
  motor->setSpeed(0);  // Stop the motor
  Serial.println("Detensioning stopped");
}

void testMotors() {
  bool restartTest = true;
  
  while (restartTest) {
    Serial.println("Run motor test sequence? (y/n)");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    
    char input = Serial.read();
    while (Serial.available()) {
      Serial.read(); // Clear any additional characters
    }
    
    if (input != 'y' && input != 'Y') {
      Serial.println("Skipping motor test sequence.");
      return;
    }
    
    const char* motorNames[] = {"EPD", "EPU", "EYR", "EYL", "WPU", "WPD", "LJR", "LJL", "RJR", "RJL", "ROLL"};
    
    for (int i = 0; i < 11; i++) {
      bool motorTested = false;
      
      while (!motorTested) {
        Serial.print("Testing motor ");
        Serial.println(motorNames[i]);
        Serial.println("Moving 5 steps clockwise...");
        
        // Move motor 5 steps clockwise
        motors[i]->move(5);
        while (motors[i]->distanceToGo() != 0) {
          motors[i]->run();
        }
        
        Serial.println("Options:");
        Serial.println("y - Motor moved successfully, proceed to next");
        Serial.println("n - Retest this motor");
        Serial.println("t - Tension this motor (fine/coarse)");
        Serial.println("d - Detension this motor");
        Serial.println("e - Escape test sequence");
        
        // Wait for user input
        while (!Serial.available()) {
          delay(100);
        }
        
        char motorInput = Serial.read();
        while (Serial.available()) {
          Serial.read(); // Clear any additional characters
        }
        
        if (motorInput == 'y' || motorInput == 'Y') {
          Serial.println("Motor test successful!");
          motorTested = true;
        } else if (motorInput == 'n' || motorInput == 'N') {
          Serial.println("Retesting motor in 1 second...");
          delay(1000);
        } else if (motorInput == 't' || motorInput == 'T') {
          tensionMotor(motors[i]);
          Serial.println("Proceed to next motor? (y/n)");
          while (!Serial.available()) {
            delay(100);
          }
          char proceedInput = Serial.read();
          while (Serial.available()) {
            Serial.read(); // Clear any additional characters
          }
          if (proceedInput == 'y' || proceedInput == 'Y') {
            Serial.println("Proceeding to next motor...");
            motorTested = true;
          } else {
            Serial.println("Retesting current motor...");
            delay(1000);
          }
        } else if (motorInput == 'd' || motorInput == 'D') {
          detensionMotor(motors[i]);
          Serial.println("Proceed to next motor? (y/n)");
          while (!Serial.available()) {
            delay(100);
          }
          char proceedInput = Serial.read();
          while (Serial.available()) {
            Serial.read(); // Clear any additional characters
          }
          if (proceedInput == 'y' || proceedInput == 'Y') {
            Serial.println("Proceeding to next motor...");
            motorTested = true;
          } else {
            Serial.println("Retesting current motor...");
            delay(1000);
          }
        } else if (motorInput == 'e' || motorInput == 'E') {
          Serial.println("Test sequence aborted.");
          return;
        } else {
          Serial.println("Invalid input. Retesting motor in 1 second...");
          delay(1000);
        }
      }
      
      // Small delay between motors
      delay(1000);
    }
    
    Serial.println("All motors tested successfully!");
    Serial.println("Options:");
    Serial.println("y - Run test sequence again");
    Serial.println("n - End test sequence");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    
    char restartInput = Serial.read();
    while (Serial.available()) {
      Serial.read(); // Clear any additional characters
    }
    
    if (restartInput != 'y' && restartInput != 'Y') {
      restartTest = false;
    }
  }
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

void moveEPTo_EZ(float abs_theta){
  float K = 1.4; //open loop gain
  float delta_theta_rad = DEG_TO_RAD(ep_pos-abs_theta);  // Calculate change in angle
  float delta_s = delta_theta_rad*3.24;
  int steps = MM_TO_STEPS(delta_s)*K;
  if (verbose_output){
    Serial.println("\nMovement values for verification:");
    Serial.print("Initial theta: ");
    Serial.println(ep_pos);
    Serial.print("Final theta: ");
    Serial.println(abs_theta);
    Serial.print("Steps: ");
    Serial.println(steps);
    
    Serial.println("\nProceed with movement? (y/n)");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    
    char input = Serial.read();
    while (Serial.available()) {
      Serial.read(); // Clear any additional characters
    }
    
    if (input != 'y' && input != 'Y') {
      Serial.println("Movement cancelled.");
      return;
    }
  }
  
  EPU.move(steps);
  EPD.move(-steps);

  // Run motors until movement is complete
  while((EPU.isRunning() || EPD.isRunning())){
    EPU.run();
    EPD.run();
  }

  ep_pos = abs_theta;
}

void moveEPToTest(float abs_theta){

  float K = 2.8; //open loop gain

  float delta_theta = ep_pos-abs_theta;  // Calculate change in angle

  float shorter_initial, longer_initial;
  getEPLength(ep_pos, shorter_initial, longer_initial);
  
  float pitch_up_initial, pitch_down_initial;
  if (ep_pos<90){
    pitch_up_initial = shorter_initial;
    pitch_down_initial = longer_initial;
  }
  else{
    pitch_up_initial = longer_initial;
    pitch_down_initial = shorter_initial;
  }

  float shorter_final, longer_final;
  getEPLength(abs_theta, shorter_final, longer_final);
  
  float pitch_up_final, pitch_down_final;
  if (abs_theta<90){
    pitch_up_final = shorter_final;
    pitch_down_final = longer_final;
  }
  else{
    pitch_up_final = longer_final;
    pitch_down_final = shorter_final;
  }

  float delta_pitch_up_pl = pitch_up_initial - pitch_up_final;
  float delta_pitch_down_pl = pitch_down_initial - pitch_down_final;

  float pitch_up_steps = MM_TO_STEPS(delta_pitch_up_pl)*K;
  float pitch_down_steps = MM_TO_STEPS(delta_pitch_down_pl)*K;

  if (verbose_output){
    Serial.println("\nMovement values for verification:");
    Serial.print("Initial theta: ");
    Serial.println(ep_pos);
    Serial.print("Final theta: ");
    Serial.println(abs_theta);
    Serial.print("Delta theta: ");
    Serial.println(delta_theta);
    Serial.print("Elbow pitch up steps: ");
    Serial.println(pitch_up_steps);
    Serial.print("Elbow pitch down steps: ");
    Serial.println(pitch_down_steps);
    
    Serial.println("\nProceed with movement? (y/n)");
    
    // Wait for user input
    while (!Serial.available()) {
      delay(100);
    }
    
    char input = Serial.read();
    while (Serial.available()) {
      Serial.read(); // Clear any additional characters
    }
    
    if (input != 'y' && input != 'Y') {
      Serial.println("Movement cancelled.");
      return;
    }
  }

  // Calculate the ratio of steps to determine speed scaling
  float max_steps = max(abs(pitch_up_steps), abs(pitch_down_steps));
  float speed_ratio_up = abs(pitch_up_steps) / max_steps;
  float speed_ratio_down = abs(pitch_down_steps) / max_steps;

  // Set speeds proportionally to the step counts
  EPU.setMaxSpeed(100 * speed_ratio_up);
  EPD.setMaxSpeed(100 * speed_ratio_down);

  //set the EPU and EPD motors
  EPU.move(pitch_up_steps);
  EPD.move(pitch_down_steps);

  // Run motors until movement is complete
  while((EPU.isRunning() || EPD.isRunning())){
    EPU.run();
    EPD.run();
  }

  // Reset speeds to default
  EPU.setMaxSpeed(100.0);
  EPD.setMaxSpeed(100.0);

  ep_pos = abs_theta;  // Update the stored position after movement is complete
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

