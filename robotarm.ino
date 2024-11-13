#include <VarSpeedServo.h>
#include <math.h>

// Define the number of servos
#define NUM_SERVOS 5

// Define the speed for servo movement (1-255)
#define SERVO_SPEED 10

// Create an array of VarSpeedServo objects
VarSpeedServo servos[NUM_SERVOS];

// Define the pins for each servo
const int servoPins[NUM_SERVOS] = {A0, A1, A2, 8, 9};

// Array to store the target angles for each servo
int servoAngles[NUM_SERVOS] = {90, 90, 90, 90, 90};

// Servo indices for readability
#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define WRIST_SERVO 3
#define CLAW_SERVO 4

// Arm segment lengths (adjust these values as per your robot arm)
const float L1 = 90.0; // Shoulder to elbow length
const float L2 = 80.0; // Elbow to wrist length

// Target coordinates
float targetX = 0;
float targetY = 0;
float targetZ = 0;

void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(9600);

  // Attach each servo to its corresponding pin
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(servoAngles[i], SERVO_SPEED, true); // Initialize servos to 90 degrees
  }

  Serial.println("5-DOF Robot Arm Control");
  Serial.println("Type 'help' for command instructions.");
}

void serialHandler(){
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra spaces or newlines

    if (input.equalsIgnoreCase("help")) {
      displayHelp();
    } else if (input.equalsIgnoreCase("stop")) {
      Serial.println("Shutting down robot arm.");
      delay(100);
      resetFunc();
    } else if (input.startsWith("targetx ") || input.startsWith("targety ") || input.startsWith("targetz ")) {
      parseAndSetTarget(input);
    } else if (input.equalsIgnoreCase("clawopen")) {
      moveServo(CLAW_SERVO,90);
    } else if(input.equalsIgnoreCase("clawclose")){
      moveServo(CLAW_SERVO, 20);
    }
    else{
      Serial.println("Error: Invalid command. Type 'help' for commands.");
    }
  }
}

void loop() {
  // Check for serial input
  serialHandler();
}

void displayHelp() {
  Serial.println("Commands: ");
  // Serial.println("  <joint><angle> - Move specific joint (e.g., base180, shoulder90)");
  Serial.println("  targetx<value> - Set X coordinate");
  Serial.println("  targety<value> - Set Y coordinate");
  Serial.println("  targetz<value> - Set Z coordinate");
  Serial.println("  stop - Reset the robot arm");
  Serial.println("  clawopen - Open claw");
  Serial.println("  clawclose - Close claw");
}

// Function to parse and set target coordinates
void parseAndSetTarget(String input) {
  // Split the input at the space
  int spaceIndex = input.indexOf(' ');

  String command = input.substring(0, spaceIndex);
  float value = input.substring(spaceIndex + 1).toFloat();

  if (command.equalsIgnoreCase("targetx")) {
    targetX = value;
  } else if (command.equalsIgnoreCase("targety")) {
    targetY = value;
  } else if (command.equalsIgnoreCase("targetz")) {
    targetZ = value;
  } 

  // Calculate joint angles using inverse kinematics
  if (calculateIK(targetX, targetY)) {
    Serial.print("Moving to coordinates (");
    Serial.print(targetX); Serial.print(", ");
    Serial.print(targetY); Serial.print(", ");
    Serial.print(targetZ); Serial.println(")");
  } else {
    Serial.println("Target coordinates out of reach.");
  }
}

// Function to calculate inverse kinematics for a 3-DOF arm
bool calculateIK(float x, float y) {
  float r = sqrt(x * x + y * y);

  // Check if the target is reachable
  if (r > (L1 + L2)) {
    return false;
  }

  // Calculate the elbow angle theta2
  float cosTheta2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float theta2 = acos(cosTheta2); // Elbow angle in radians

  // Calculate the shoulder angle theta1
  float alpha = atan2(y, x); // Angle to the target point
  float beta = atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)); // Offset angle
  float theta1 = alpha - beta; // Shoulder angle in radians

  // Convert radians to degrees
  int shoulderAngle = (int)(theta1 * 180.0 / PI);
  int elbowAngle = (int)(theta2 * 180.0 / PI);

  int wristAngle = 180 - (shoulderAngle + elbowAngle);

  // Move the servos to calculated angles
  moveServo(SHOULDER_SERVO, shoulderAngle);
  moveServo(ELBOW_SERVO, elbowAngle);
  moveServo(WRIST_SERVO, wristAngle);
  return true;
}





void moveServo(int servoIndex, int angle) {
  angle = constrain(angle, 0, 180);
  servos[servoIndex].write(angle, SERVO_SPEED, false);
  servoAngles[servoIndex] = angle;

  // Print confirmation
  // Serial.print("Moving servo ");
  // Serial.print(servoIndex);
  // Serial.print(" to ");
  // Serial.print(angle);
  // Serial.println(" degrees.");
}

void parseAndMoveServo(String input) {
  // Extract the joint name (alphabetic part) and angle (numeric part)
  String joint = "";
  String angleStr = "";

  for (int i = 0; i < input.length(); i++) {
    if (isAlpha(input[i])) {
      joint += input[i];
    } else if (isDigit(input[i])) {
      angleStr += input[i];
    }
  }

  if (angleStr.length() == 0) {
    Serial.println("Invalid command. Type 'help' for usage.");
    return;
  }

  int angle = angleStr.toInt();
  angle = constrain(angle, 0, 180);

  int servoIndex = -1;
  if (joint.equalsIgnoreCase("base")) {
    servoIndex = BASE_SERVO;
  } else if (joint.equalsIgnoreCase("shoulder")) {
    servoIndex = SHOULDER_SERVO;
  } else if (joint.equalsIgnoreCase("elbow")) {
    servoIndex = ELBOW_SERVO;
  } else if (joint.equalsIgnoreCase("wrist")) {
    servoIndex = WRIST_SERVO;
  } else if (joint.equalsIgnoreCase("claw")) {
    servoIndex = CLAW_SERVO;
  } else {
    Serial.println("Unknown joint. Type 'help' for usage.");
    return;
  }

  servos[servoIndex].write(angle, SERVO_SPEED, false);
  servoAngles[servoIndex] = angle;

  Serial.print("Moving ");
  Serial.print(joint);
  Serial.print(" to ");
  Serial.print(angle);
  Serial.println(" degrees.");
}
