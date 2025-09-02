
// This sketch provides an integrated control system for 8 DYNAMIXEL motors.
// It listens for a specific hardware serial command, parses relative movement
// deltas, reads the motors' current positions, calculates new absolute goals,
// and executes the movement simultaneously using SyncRead and SyncWrite.
// It now uses Extended Position Control Mode for unlimited rotational travel.
//
// - Uses hardware interrupts to immediately stop all motors if a limit is reached.
// - Performs a pre-movement check to ensure a motor isn't already at its limit.
//
// - Send "TOGGLE_VERBOSE" to switch between detailed logging and a fast, quiet mode.
// - In quiet mode, only critical errors are printed.
//

#include <Dynamixel2Arduino.h>


// =============================================================================
// HARDWARE & COMMUNICATION CONFIGURATION
// =============================================================================
#if defined(ARDUINO_OpenRB)  // When using OpenRB-150
  #define DXL_SERIAL Serial1 // Hardware port for DYNAMIXELs
  #define DEBUG_SERIAL Serial   // USB-C port for monitoring and commands
  const int DXL_DIR_PIN = -1; // A value of -1 indicates no direction pin
  #else // Other boards when using a DYNAMIXEL Shield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2;
#endif

const float DXL_PROTOCOL_VERSION = 2.0;
const int DXL_BAUDRATE = 57600;
const int DEBUG_BAUDRATE = 115200;
const uint8_t BROADCAST_ID = 254;
const uint8_t DXL_ID_CNT = 8;

// --- Motor Position Limits ---
// These variables will be updated by messages from the Linux script.
// Initialize them to a safe, neutral position like 2048 (middle of 0-4095 range).
int theta1_min = 2048;
int theta1_max = 2048;
int theta2_min = 2048;
int theta2_max = 2048;

int lowerLimitSwitchPin = 10;

int m1_tuned_p = 3000;
int m2_tuned_p = 2000;

const uint32_t FAST_PROFILE_VELOCITY = 300; // Adjust as needed for the desired "fast" speed.
const uint32_t SLOW_PROFILE_VELOCITY = 5;   // Approx. 300 / 56. Can be fine-tuned.


// --- Verbose Mode Control ---
bool verbose_mode = true; // Controls detailed serial output. Toggled by command.

// =============================================================================
// REFACTORED MOTOR CONFIGURATION (SINGLE SOURCE OF TRUTH)
// =============================================================================
// A struct to hold all configuration for a single motor.
struct MotorConfig {
  const uint8_t id;
  const char* name;
  const int limitSwitchPin;
  const int commandIndex; // The 0-based index for this motor in the "MOVE_ALL_MOTORS" command
};

// This is now the ONLY place you need to edit motor configurations.
// The order of motors in THIS array defines the internal processing order.
// - id: The motor's DYNAMIXEL ID.
// - name: The string name for debugging.
// - limitSwitchPin: The Arduino pin for the limit switch (-1 if none).
// - commandIndex: The position (0-7) of this motor's delta in the incoming command string.
const MotorConfig motor_configs[DXL_ID_CNT] = {
  // {id, name,   limitSwitchPin, commandIndex}
  {1, "Q1",   -1, 0}, // Internal Motor 0 -> Receives delta from command position 0
  {2, "Q2",   -1, 1}, // Internal Motor 1 -> Receives delta from command position 1
  {3, "Q4L+",  6, 6}, // Internal Motor 2 -> Receives delta from command position 6
  {4, "Q4L-",  5, 7}, // Internal Motor 3 -> Receives delta from command position 7
  {7, "Q4R+",  4, 4}, // Internal Motor 4 -> Receives delta from command position 4
  {8, "Q3-",   7, 3}, // Internal Motor 5 -> Receives delta from command position 3
  {5, "Q4R-",  0, 5}, // Internal Motor 6 -> Receives delta from command position 5
  {6, "Q3+",   1, 2}  // Internal Motor 7 -> Receives delta from command position 2
};


// Volatile flag set by the ISR for an emergency stop.
volatile bool limitSwitchHit = false;

// ISR function - keeps it short and fast!
void onSwitchChange() {
  limitSwitchHit = true;
}

// =============================================================================
// DYNAMIXEL LIBRARY SETUP
// =============================================================================

// -- Control Table Addresses --
const uint16_t ADDR_PRESENT_POSITION = 132;
const uint16_t LEN_PRESENT_POSITION = 4;
const uint16_t ADDR_GOAL_POSITION = 116;
const uint16_t LEN_GOAL_POSITION = 4;

// Define structures to make handling the data easier.
typedef struct sr_data { int32_t present_position; } __attribute__((packed)) sr_data_t;
typedef struct sw_data { int32_t goal_position; } __attribute__((packed)) sw_data_t;

// Create arrays to hold data for each motor. These are unchanged from the original.
sr_data_t sr_data[DXL_ID_CNT]; // Holds data from SyncRead
sw_data_t sw_data[DXL_ID_CNT]; // Holds data for SyncWrite

// SyncRead and SyncWrite structures
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t  info_xels_sr[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t  info_xels_sw[DXL_ID_CNT];

// Instantiate the main Dynamixel2Arduino object.
#if DXL_DIR_PIN == -1
  Dynamixel2Arduino dxl(DXL_SERIAL);
#else
  Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
#endif

//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

// =============================================================================
// COMMAND PROCESSING VARIABLES
// =============================================================================
const char* CMD_HEADER = "MOVE_ALL_MOTORS:";
String incomingCommand = "";
const int DELTA_COUNT = 8;

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(DEBUG_BAUDRATE);
  while(!DEBUG_SERIAL);

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("Setting up motors and limit switches...");
  for(uint8_t i = 0; i < DXL_ID_CNT; i++){
    const auto& motor = motor_configs[i]; // Use a reference for cleaner code

    // --- Motor Setup ---
    if (dxl.ping(motor.id)) {
      if (verbose_mode) {
        DEBUG_SERIAL.print("   > Found motor: ");
        DEBUG_SERIAL.print(motor.name);
        DEBUG_SERIAL.print(" (ID ");
        DEBUG_SERIAL.print(motor.id);
        DEBUG_SERIAL.println(")");
      }
      dxl.torqueOff(motor.id);
      dxl.setOperatingMode(motor.id, OP_EXTENDED_POSITION);

           // Set the Profile Velocity to control motor speed.
      if (motor.id == 1 || motor.id == 2) {
        dxl.writeControlTableItem(PROFILE_VELOCITY, motor.id, SLOW_PROFILE_VELOCITY);
        if (verbose_mode) {
           DEBUG_SERIAL.print("       -> Set SLOW Profile Velocity: ");
           DEBUG_SERIAL.println(SLOW_PROFILE_VELOCITY);
        }
      } else {
        dxl.writeControlTableItem(PROFILE_VELOCITY, motor.id, FAST_PROFILE_VELOCITY);
        if (verbose_mode) {
           DEBUG_SERIAL.print("       -> Set FAST Profile Velocity: ");
           DEBUG_SERIAL.println(FAST_PROFILE_VELOCITY);
        }
      }

      if (motor.id == 1) {
        dxl.writeControlTableItem(POSITION_P_GAIN, motor.id, m1_tuned_p); 
        if (verbose_mode) DEBUG_SERIAL.print("         -> Set custom P-Gain: 1250");
      } else if (motor.id == 2) {
        dxl.writeControlTableItem(POSITION_P_GAIN, motor.id, m2_tuned_p); 
        if (verbose_mode) DEBUG_SERIAL.print("         -> Set custom P-Gain: 1200");
      }

    } else {
      DEBUG_SERIAL.print("   > FAILED to find motor ID: ");
      DEBUG_SERIAL.println(motor.id);
    }

    // --- Limit Switch Setup ---
    if (motor.limitSwitchPin != -1) {
      pinMode(motor.limitSwitchPin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(motor.limitSwitchPin), onSwitchChange, CHANGE);
      if (verbose_mode) {
        DEBUG_SERIAL.print("     - Interrupt attached to pin D");
        DEBUG_SERIAL.print(motor.limitSwitchPin);
        DEBUG_SERIAL.print(" for motor ");
        DEBUG_SERIAL.println(motor.name);
      }
    }
  }

  pinMode(10, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lowerLimitSwitchPin), onSwitchChange, CHANGE);

  dxl.torqueOn(BROADCAST_ID);
  DEBUG_SERIAL.println("Setup complete. Ready for commands.");

  // --- Prepare SyncRead Structure ---
  sr_infos.packet.p_buf = nullptr;
  sr_infos.addr = ADDR_PRESENT_POSITION;
  sr_infos.addr_length = LEN_PRESENT_POSITION;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = DXL_ID_CNT;
  for(uint8_t i = 0; i < DXL_ID_CNT; i++){
    info_xels_sr[i].id = motor_configs[i].id; // Use ID from config
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
  }
  sr_infos.is_info_changed = true;

  // --- Prepare SyncWrite Structure ---
  sw_infos.packet.p_buf = nullptr;
  sw_infos.addr = ADDR_GOAL_POSITION;
  sw_infos.addr_length = LEN_GOAL_POSITION;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = DXL_ID_CNT;
  for(uint8_t i = 0; i < DXL_ID_CNT; i++){
    info_xels_sw[i].id = motor_configs[i].id; // Use ID from config
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
  }
  sw_infos.is_info_changed = true;
}

void loop() {
  if (limitSwitchHit) {
    emergencyStop();
  }

  handleSerialCommands();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(10);
}

// =============================================================================
// EMERGENCY STOP
// =============================================================================
void emergencyStop() {
  if (verbose_mode) {
    DEBUG_SERIAL.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    DEBUG_SERIAL.println("!!! EMERGENCY STOP: Limit switch interrupt triggered!");
    DEBUG_SERIAL.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  } else {
    DEBUG_SERIAL.println("\n!!! EMERGENCY STOP !!!");
  }
  
  bool found_switch = false;
  dxl.torqueOff(BROADCAST_ID);
  delay(5); 

  // Check which switch was pressed
  for (int i = 0; i < DXL_ID_CNT; i++) {
    const auto& motor = motor_configs[i];
    if (motor.limitSwitchPin != -1) {
      if (digitalRead(motor.limitSwitchPin) == HIGH) {
        DEBUG_SERIAL.print("!!! ERROR: Limit switch reached for Motor ");
        DEBUG_SERIAL.print(motor.name);
        DEBUG_SERIAL.print(" (ID ");
        DEBUG_SERIAL.print(motor.id);
        DEBUG_SERIAL.print(") on pin D");
        DEBUG_SERIAL.println(motor.limitSwitchPin);
        found_switch = true; 
      }
    }
  }

  if(!found_switch){
    DEBUG_SERIAL.print("Check lower limit switches!");
  }

  limitSwitchHit = false;
  dxl.torqueOn(BROADCAST_ID);
  if (verbose_mode) {
    DEBUG_SERIAL.println("!!! System reset. Torque re-enabled. Ready for new command.\n");
  }
}

void handleSerialCommands() {
  if (DEBUG_SERIAL.available() > 0) {
    char incomingChar = DEBUG_SERIAL.read();
    if (incomingChar == '\n') {
      incomingCommand.trim();
      if (incomingCommand.startsWith(CMD_HEADER)) {
        parseAndExecuteMoveCommand(incomingCommand);
      }
      else if (incomingCommand.startsWith("FIND_LIMITS")){
        findLimits();
      }
      else if(incomingCommand.startsWith("UPDATE_LIMITS [")) {
        int endIndex = incomingCommand.indexOf(']');
        if (endIndex == -1) {
          Serial.println("Error: Malformed command, no closing ']' found.");
          return;
        }
        updateLimits(incomingCommand, endIndex);
      }
      else if (incomingCommand.startsWith("TOGGLE_VERBOSE")) {
        verbose_mode = !verbose_mode;
        DEBUG_SERIAL.print("Verbose mode is now ");
        DEBUG_SERIAL.println(verbose_mode ? "ON" : "OFF");
      }
      incomingCommand = "";
    } else {
      incomingCommand += incomingChar;
    }
  }
}

// =============================================================================
// PRE-MOVE SAFETY CHECK
// =============================================================================
bool isMoveBlockedBySwitch(long deltas[]) {
  for (int i = 0; i < DXL_ID_CNT; i++) {
    const auto& motor = motor_configs[i];
    
    // Check only motors that are supposed to move AND have a switch
    if (deltas[i] != 0 && motor.limitSwitchPin != -1) {
      if (digitalRead(motor.limitSwitchPin) == HIGH) {
        if (verbose_mode) {
          DEBUG_SERIAL.print("\n   > PRE-MOVE CHECK FAILED: Cannot move motor ");
          DEBUG_SERIAL.print(motor.name);
          DEBUG_SERIAL.print(" because its limit switch on pin D");
          DEBUG_SERIAL.print(motor.limitSwitchPin);
          DEBUG_SERIAL.println(" is already pressed.");
        }
        return true; // Move is blocked
      }
    }
  }
  return false; // Move is not blocked
}

bool isMoveBlockedByLimit(long deltas[]) {
  // Loop through all configured motors to find Q1 (ID 1) and Q2 (ID 2).
  for (int i = 0; i < DXL_ID_CNT; i++) {
    const auto& motor = motor_configs[i];
    
    // We only need to check motors that are actually being commanded to move.
    if (deltas[i] == 0) {
      continue; // Skip to the next motor if this one isn't moving.
    }

    // Calculate the potential new position for the current motor.
    long currentPosition = sr_data[i].present_position;
    long newGoalPosition = currentPosition + deltas[i];

    // --- Check limits for Motor ID 1 (Q1) ---
    if (motor.id == 1) {
      // Block if moving towards the minimum AND the goal is less than the min,
      // OR if moving towards the maximum AND the goal is greater than the max.
      if ((deltas[i] < 0 && newGoalPosition < theta1_min) || (deltas[i] > 0 && newGoalPosition > theta1_max)) {
        if (verbose_mode) {
          DEBUG_SERIAL.print("\n   > PRE-MOVE CHECK FAILED: Cannot move motor ");
          DEBUG_SERIAL.print(motor.name);
          DEBUG_SERIAL.print(". Goal (");
          DEBUG_SERIAL.print(newGoalPosition);
          DEBUG_SERIAL.print(") exceeds limits [");
          DEBUG_SERIAL.print(theta1_min);
          DEBUG_SERIAL.print(", ");
          DEBUG_SERIAL.print(theta1_max);
          DEBUG_SERIAL.println("].");
        }
        return true; // Move is blocked
      }
    }
    
    // --- Check limits for Motor ID 2 (Q2) ---
    else if (motor.id == 2) {
      // Apply the same logic as above for motor Q2 and its specific limits.
      if ((deltas[i] < 0 && newGoalPosition < theta2_min) || (deltas[i] > 0 && newGoalPosition > theta2_max)) {
        if (verbose_mode) {
          DEBUG_SERIAL.print("\n   > PRE-MOVE CHECK FAILED: Cannot move motor ");
          DEBUG_SERIAL.print(motor.name);
          DEBUG_SERIAL.print(". Goal (");
          DEBUG_SERIAL.print(newGoalPosition);
          DEBUG_SERIAL.print(") exceeds limits [");
          DEBUG_SERIAL.print(theta2_min);
          DEBUG_SERIAL.print(", ");
          DEBUG_SERIAL.print(theta2_max);
          DEBUG_SERIAL.println("].");
        }
        return true; // Move is blocked
      }
    }
  }

  // If the loop completes without finding any violations, the move is safe.
  return false; // Move is NOT blocked
}


// =============================================================================
// COMMAND PARSING AND EXECUTION
// =============================================================================
void parseAndExecuteMoveCommand(String cmd) {
  if (verbose_mode) {
    DEBUG_SERIAL.println("\n=======================================================");
    DEBUG_SERIAL.print("Received Command: ");
    DEBUG_SERIAL.println(cmd);
  }

  if(cmd.startsWith("FIND LIMITS")){
    findLimits();
  }
  else {
    // 1. Get current positions
    if (verbose_mode) DEBUG_SERIAL.println("Step 1: Reading current motor positions...");
    if(dxl.syncRead(&sr_infos) != DXL_ID_CNT) {
      DEBUG_SERIAL.println("   > ERROR: Failed to read all motors. Aborting.");
      return;
    }
    if (verbose_mode) DEBUG_SERIAL.println("   > Success. All motors responded.");

    // 2. Parse delta values
    if (verbose_mode) DEBUG_SERIAL.println("Step 2: Parsing movement deltas...");
    long parsed_deltas[DELTA_COUNT] = {0};
    String valueString = cmd.substring(strlen(CMD_HEADER));
    
    // Create a mutable copy of the string for strtok to safely modify.
    char string_to_parse[valueString.length() + 1];
    strcpy(string_to_parse, valueString.c_str());

    char* token;
    int index = 0;
    token = strtok(string_to_parse, ",");
    while (token != NULL && index < DELTA_COUNT) {
      parsed_deltas[index++] = atol(token);
      token = strtok(NULL, ",");
    }
    if (index != DELTA_COUNT) {
        DEBUG_SERIAL.print("   > ERROR: Command requires ");
        DEBUG_SERIAL.print(DELTA_COUNT);
        DEBUG_SERIAL.println(" values. Aborting.");
        return;
    }

    // 3. Map parsed deltas to final deltas using the config array
    if (verbose_mode) DEBUG_SERIAL.println("Step 3: Mapping deltas to motors...");
    long final_deltas[DXL_ID_CNT] = {0};
    for (int i = 0; i < DXL_ID_CNT; i++) {
      // The i-th motor in our internal processing order gets its delta
      // from the commandIndex specified in its config.
      final_deltas[i] = parsed_deltas[motor_configs[i].commandIndex];
    }

    // 4. Pre-move safety check
    if (verbose_mode) DEBUG_SERIAL.println("Step 4: Performing pre-move safety check...");
    if (isMoveBlockedBySwitch(final_deltas)) {
      if (verbose_mode) {
        DEBUG_SERIAL.println("   > Aborting move command due to active limit switch.");
        DEBUG_SERIAL.println("=======================================================\n");
      }
      return;
    }
    if (isMoveBlockedByLimit(final_deltas)){
      if (verbose_mode) {
        DEBUG_SERIAL.println("   > Aborting move command due to position limits.");
        DEBUG_SERIAL.println("=======================================================\n");
      }
      return;
    }
    if (verbose_mode) DEBUG_SERIAL.println("   > Success. Path is clear.");

    // 5. Calculate final goals
    if (verbose_mode) DEBUG_SERIAL.println("Step 5: Finalizing goal positions...");
    for(int i=0; i<DXL_ID_CNT; i++){
      sw_data[i].goal_position = sr_data[i].present_position + final_deltas[i];
    }

    // 6. Execute move
    if (verbose_mode) DEBUG_SERIAL.println("Step 6: Executing move with SyncWrite...");
    sw_infos.is_info_changed = true;
    if(dxl.syncWrite(&sw_infos)) {
      if (verbose_mode) DEBUG_SERIAL.println("   > [SyncWrite] Success. Command sent.");
    } else {
      DEBUG_SERIAL.print("   > [SyncWrite] Fail, Lib error code: ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
    }
    if (verbose_mode) DEBUG_SERIAL.println("=======================================================\n");
  }
}


//@brief Updates the position limits for a specific motor in EEPROM and RAM.
//@param motor_ID The ID of the motor to update (e.g., 1 or 2).
//@param newMin The new minimum position limit.
//@param newMax The new maximum position limit.

void updateLimits(String message, int endIndex) {
    String numbersPart = message.substring(15, endIndex);

    // Create a temporary array to hold the parsed values
    int parsedValues[4];
    int valueIndex = 0;
    int lastCommaIndex = -1;

    // Loop to find each comma and extract the number between them
    for (int i = 0; i < 4; i++) {
      int nextCommaIndex = numbersPart.indexOf(',', lastCommaIndex + 1);
      String valStr;

      if (i < 3 && nextCommaIndex == -1) {
        Serial.println("Error: Malformed command, expected 4 values.");
        return; // Exit if we expect more commas but don't find one
      }
      
      if (nextCommaIndex == -1) {
        // This is the last value
        valStr = numbersPart.substring(lastCommaIndex + 1);
      } else {
        valStr = numbersPart.substring(lastCommaIndex + 1, nextCommaIndex);
      }
      
      parsedValues[valueIndex] = valStr.toInt();
      valueIndex++;
      lastCommaIndex = nextCommaIndex;
    }

    // If we successfully parsed 4 values, update the global variables
    if (valueIndex == 4) {
      theta1_min = parsedValues[0];
      theta1_max = parsedValues[1];
      theta2_min = parsedValues[2];
      theta2_max = parsedValues[3];

      Serial.println("Successfully updated motor limits:");
      Serial.print("  Theta1 Min: "); Serial.println(theta1_min);
      Serial.print("  Theta1 Max: "); Serial.println(theta1_max);
      Serial.print("  Theta2 Min: "); Serial.println(theta2_min);
      Serial.print("  Theta2 Max: "); Serial.println(theta2_max);
    }

}

void findLimits() {
  Serial.println("\n--- Starting Limit Finding Routine ---");
  char command;

  // --- 1. Find Theta 1 Minimum ---
  Serial.println("[Step Q1 until it is at its minimum. Send 's' to step by -100, 'l' to lock limit]");
  while (true) {
    if (Serial.available() > 0) {
      command = tolower(Serial.read());
      if (command == 's') {
        int32_t currentPos = dxl.getPresentPosition(1);
        dxl.setGoalPosition(1, currentPos - 100);
        Serial.println("Stepping...");
      } else if (command == 'l') {
        theta1_min = dxl.getPresentPosition(1);
        Serial.print("--> Theta 1 minimum set to: ");
        Serial.println(theta1_min);
        break;
      }
    }
  }

  // --- 2. Find Theta 1 Maximum ---
  Serial.println("\n[Step Q1 until it is at its maximum. Send 's' to step by +100, 'l' to lock limit]");
  while (true) {
    if (Serial.available() > 0) {
      command = tolower(Serial.read());
      if (command == 's') {
        int32_t currentPos = dxl.getPresentPosition(1);
        dxl.setGoalPosition(1, currentPos + 100);
        Serial.println("Stepping...");
      } else if (command == 'l') {
        theta1_max = dxl.getPresentPosition(1);
        Serial.print("--> Theta 1 maximum set to: ");
        Serial.println(theta1_max);
        break;
      }
    }
  }

  // --- 3. Find Theta 2 Minimum ---
  Serial.println("\n[Step Q2 until it is at its minimum. Send 's' to step by -100, 'l' to lock limit]");
  while (true) {
    if (Serial.available() > 0) {
      command = tolower(Serial.read());
      if (command == 's') {
        int32_t currentPos = dxl.getPresentPosition(2);
        dxl.setGoalPosition(2, currentPos - 100);
        Serial.println("Stepping...");
      } else if (command == 'l') {
        theta2_min = dxl.getPresentPosition(2);
        Serial.print("--> Theta 2 minimum set to: ");
        Serial.println(theta2_min);
        break;
      }
    }
  }

  // --- 4. Find Theta 2 Maximum ---
  Serial.println("\n[Step Q2 until it is at its maximum. Send 's' to step by +50, 'l' to lock limit]");
  while (true) {
    if (Serial.available() > 0) {
      command = tolower(Serial.read());
      if (command == 's') {
        int32_t currentPos = dxl.getPresentPosition(2);
        dxl.setGoalPosition(2, currentPos + 50);
        Serial.println("Stepping...");
      } else if (command == 'l') {
        theta2_max = dxl.getPresentPosition(2);
        Serial.print("--> Theta 2 maximum set to: ");
        Serial.println(theta2_max);
        break;
      }
    }
  }
  
  Serial.println("\n--- Limit Finding Complete! ---");
  Serial.println("Final limits:");
  Serial.print("  Theta1: "); Serial.print(theta1_min); Serial.print(" -> "); Serial.println(theta1_max);
  Serial.print("  Theta2: "); Serial.print(theta2_min); Serial.print(" -> "); Serial.println(theta2_max);
}