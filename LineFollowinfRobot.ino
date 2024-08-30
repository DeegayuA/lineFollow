// IR Sensor Pin Definitions
#define IR_SENSOR_RIGHT 11  // Pin for the right IR sensor
#define IR_SENSOR_LEFT 12   // Pin for the left IR sensor

// Motor Speed Definitions
#define MOTOR_SPEED 150  // Default motor speed (PWM value) (min 80 - max 255 and 150 = 58.82% speed)

// Right Motor Pin Definitions
int enableRightMotor = 10; // PWM pin to control speed of right motor
int rightMotorPin1 = 9;    // Pin to control direction of right motor
int rightMotorPin2 = 8;    // Pin to control direction of right motor

// Left Motor Pin Definitions
int enableLeftMotor = 5;   // PWM pin to control speed of left motor
int leftMotorPin1 = 7;     // Pin to control direction of left motor
int leftMotorPin2 = 6;     // Pin to control direction of left motor

// Setup function: This runs once when the microcontroller starts
void setup() {
  // Adjust PWM frequency on pins 5 and 6 to 7812.5 Hz
  // This allows for finer control of motor speed at higher PWM values
  TCCR0B = (TCCR0B & 0b11111000) | 0b00000010;  // Set Timer 0 prescaler to 8

  // Initialize motor control pins as outputs
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Initialize IR sensor pins as inputs
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);

  // Stop both motors at startup
  rotateMotor(0, 0);   
}

// Loop function: This runs continuously after setup()
void loop() {
  // Read IR sensor values
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  // Control motor movement based on sensor readings
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    // Both sensors see white (no line): move straight
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  } else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    // Right sensor sees black (line detected): turn right
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
  } else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    // Left sensor sees black (line detected): turn left
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  } else {
    // Both sensors see black (on the line): stop
    rotateMotor(0, 0);
  }
}

// Function to control motor rotation and speed
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  
  // Control right motor direction
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);    
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);      
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);      
  }

  // Control left motor direction
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);    
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);      
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);      
  }

  // Set motor speed using PWM signals
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
