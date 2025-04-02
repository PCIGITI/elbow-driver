#include <AccelStepper.h>
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define MM_TO_STEPS(x) ((int)((x/0.5)*200))  // For non-EP/EY motors: 0.5mm per rev, 200 steps per rev

// Emergency stop variables
volatile bool emergencyStop = false;
const int EMERGENCY_PIN = 9;

AccelStepper RJR(AccelStepper::DRIVER, 41, 40);
AccelStepper RJL(AccelStepper::DRIVER, 39, 38);
AccelStepper WPD(AccelStepper::DRIVER, 45, 44);
AccelStepper EY(AccelStepper::DRIVER, 47, 46); // ELBOW YAW
AccelStepper EP(AccelStepper::DRIVER, 49, 48);
AccelStepper WPU(AccelStepper::DRIVER, 51, 50);
AccelStepper LJR(AccelStepper::DRIVER, 43, 42);
AccelStepper LJL(AccelStepper::DRIVER, 53, 52);
AccelStepper ROLL(AccelStepper::DRIVER, 37, 36);

float ey_pos = 90;  // Initial position in degrees
float ep_pos = 90;  // Initial position in degrees
float wp_pos = 90;  // Initial wrist pitch position in degrees
AccelStepper* motors[] = {&RJR, &RJL, &WPD, &EY, &EP, &WPU, &LJR, &LJL, &ROLL};

// Function declarations
void moveLink(AccelStepper* m1, AccelStepper* m2, float steps);
void moveEYTo(float abs_theta);
float getLongerPathEY(float theta_deg);
float getShorterPathEY(float theta_deg);
int getEYSteps(float delta_theta);
void moveWPTo(float abs_theta);
float getWPSteps(float delta_theta);
float getLongerPathWP(float theta);
float getShorterPathWP(float theta);
void detension();
void testMotors();
void emergencyStopISR();
void tensionMotor(AccelStepper* motor);


//MOVING MOTORS ON LEAD SCREW CLOCKWISE (+VE STEPS) RESULTS IN CABLE LENGTH SHORTENING
//MOVING MOTORS ON LEAD SCREW COUNTERCLOCKWISE (-VE STEPS) RESULTS IN CABLE LENGTH LENGTHENING
void setup()
{  
  Serial.begin(9600);
  
  // Setup emergency stop
  pinMode(EMERGENCY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_PIN), emergencyStopISR, FALLING);
  // Set speeds for jaw motors (faster)
  RJR.setMaxSpeed(100.0);
  RJL.setMaxSpeed(100.0);
  LJR.setMaxSpeed(100.0);
  LJL.setMaxSpeed(100.0);
  
  // Set speed for Elbow motors (slower, about 1/5.85 of jaw speed)
  EY.setMaxSpeed(17.1);  // 100/5.85 ≈ 17.1
  EP.setMaxSpeed(17.1);  // Same as EY since it has same step ratio

  // Set speeds for other motors
  WPD.setMaxSpeed(100.0);
  WPU.setMaxSpeed(100.0);
  ROLL.setMaxSpeed(100.0);

  // Set acceleration for all motors
  for (int i = 0; i < 9; i++) {
    motors[i]->setAcceleration(1000000.0);  // Effectively no ramping
    motors[i]->setCurrentPosition(0);
  }

  
  //detension();
  Serial.println("Starting motor test sequence...");
  testMotors();
  Serial.println("Motor testing complete. Beginning normal operation...");

}

void loop()
{
  detension();
  ///moveEYTo(70);
  //moveEYTo(60);
  //moveEYTo(50);
}

void moveEYTo(float abs_theta){
  if (emergencyStop) {
    Serial.println("Movement cancelled due to emergency stop");
    emergencyStop = false;  // Reset the flag
    return;
  }

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
  Serial.print("Jaw right steps: ");
  Serial.println(jaw_right_steps);
  Serial.print("Jaw left steps: ");
  Serial.println(jaw_left_steps);
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
  EY.move(ey_steps);

  // All cables will now move at the same physical speed
  while((RJR.isRunning() || LJR.isRunning() || RJL.isRunning() || LJL.isRunning() || EY.isRunning()) && !emergencyStop){
    RJR.run();
    LJR.run();
    RJL.run();
    LJL.run();
    EY.run();
  }
  
  if (emergencyStop) {
    Serial.println("Movement stopped due to emergency stop");
    emergencyStop = false;  // Reset the flag
    return;
  }

  ey_pos = abs_theta;  // Update the stored position after movement is complete
}

float getLongerPathEY(float theta_deg){
  Serial.println("\nDebug getLongerPathEY:");
  Serial.print("Input theta (degrees): ");
  Serial.println(theta_deg);
  
  // Convert to radians at the start
  float theta = DEG_TO_RAD(theta_deg);
  if(theta_deg>90){
    theta = DEG_TO_RAD(180-theta_deg);
  }
  
  float L_a = 1.9;
  float L_b = 1.1;
  float r_s = 0.67/2;
  float r_c = 0.93/2;
  float L_1 = sqrt(pow(1.9,2) + pow(1.1,2));
  
  float x1 = -sin(theta)*r_c+L_b;
  float y1 = cos(theta)*r_c+L_a;
  
  float h = sqrt(pow(x1,2) + pow(y1,2));
  float L_3 = sqrt(pow(h,2) + pow(r_s,2));
  
  float beta = atan(L_a/L_b)-atan(y1/x1);
  float lambda = asin((sin(beta)*L_1)/r_c);
  float alpha = asin(r_s/L_3)-lambda;
  float arc_length = alpha*r_s;  // alpha is already in radians
  
  float result = L_3+arc_length;
  Serial.print("Final result: ");
  Serial.println(result);
  
  return result; 
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
  Serial.println("Tensioning motor... Press any key to stop");
  
  // Save current settings
  float currentMaxSpeed = motor->maxSpeed();
  float currentAccel = motor->acceleration();
  
  // Set up for tensioning
  motor->setMaxSpeed(50.0);
  motor->setAcceleration(1000000.0);  // Effectively no ramping
  motor->setSpeed(50.0);
  
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
    
    const char* motorNames[] = {"RJR", "RJL", "WPD", "EY", "EP", "WPU", "LJR", "LJL", "ROLL"};
    AccelStepper* testMotors[] = {&RJR, &RJL, &WPD, &EY, &EP, &WPU, &LJR, &LJL, &ROLL};
    
    for (int i = 0; i < 9; i++) {
      bool motorTested = false;
      
      while (!motorTested) {
        Serial.print("Testing motor ");
        Serial.println(motorNames[i]);
        Serial.println("Moving 5 steps clockwise...");
        
        // Move motor 5 steps clockwise
        testMotors[i]->move(5);
        while (testMotors[i]->distanceToGo() != 0) {
          testMotors[i]->run();
        }
        
        Serial.println("Options:");
        Serial.println("y - Motor moved successfully, proceed to next");
        Serial.println("n - Retest this motor");
        Serial.println("t - Tension this motor");
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
          tensionMotor(testMotors[i]);
          Serial.println("Proceeding to next motor...");
          motorTested = true;  // Skip retest after tensioning
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

void emergencyStopISR() {
  emergencyStop = true;
  Serial.println("EMERGENCY STOP TRIGGERED!");
}

