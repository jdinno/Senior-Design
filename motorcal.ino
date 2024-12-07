#include <Servo.h>

// Define pins
const int servoPin = 9;       // PWM pin connected to the servo motor
const int shuntPin = A0;      // Analog pin connected to the shunt resistor

// Define voltage thresholds
const float voltageLowThreshold = 0.5; // Adjust based on your requirement (in volts)
const float voltageHighThreshold = 2.5; // Adjust based on your requirement (in volts)

// Servo control
Servo myServo;
int servoAngle = 0;           // Current angle of the servo

// Limits to store
int lowVoltageAngle = -1;     // Angle corresponding to low voltage threshold
int highVoltageAngle = -1;    // Angle corresponding to high voltage threshold

// ADC properties
const float referenceVoltage = 5.0; // Reference voltage for ADC (5V for typical Arduino)
const int adcResolution = 1024;     // ADC resolution for 10-bit ADC

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);

  // Calibration process
  Serial.println("Starting calibration...");
  for (servoAngle = 0; servoAngle <= 180; servoAngle++) {
    myServo.write(servoAngle);
    delay(20); // Allow servo to reach position

    // Read the voltage across the shunt resistor
    float shuntVoltage = readShuntVoltage();

    // Print for debugging
    Serial.print("Angle: ");
    Serial.print(servoAngle);
    Serial.print(" - Voltage: ");
    Serial.println(shuntVoltage);

    // Check thresholds and store angles
    if (shuntVoltage >= voltageLowThreshold && lowVoltageAngle == -1) {
      lowVoltageAngle = servoAngle;
      Serial.print("Low voltage limit angle: ");
      Serial.println(lowVoltageAngle);
    }
    if (shuntVoltage >= voltageHighThreshold && highVoltageAngle == -1) {
      highVoltageAngle = servoAngle;
      Serial.print("High voltage limit angle: ");
      Serial.println(highVoltageAngle);
    }

    // Exit loop if both limits are found
    if (lowVoltageAngle != -1 && highVoltageAngle != -1) {
      break;
    }
  }

  // Print final limits
  Serial.println("Calibration complete:");
  Serial.print("Low Voltage Limit: ");
  Serial.println(lowVoltageAngle);
  Serial.print("High Voltage Limit: ");
  Serial.println(highVoltageAngle);

  // Stop further operations
  myServo.detach();
}

void loop() {
  // No operation in loop
}

// Function to read the voltage across the shunt resistor
float readShuntVoltage() {
  int rawADC = analogRead(shuntPin);
  return (rawADC * referenceVoltage) / adcResolution;
}
