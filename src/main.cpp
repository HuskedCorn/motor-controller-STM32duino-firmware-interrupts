#include <Arduino.h>

#define PIN_MOSFET_A_LO D11
#define PIN_MOSFET_A_HI D10
#define PIN_MOSFET_B_LO D5
#define PIN_MOSFET_B_HI D3
#define PIN_MOSFET_C_LO D9
#define PIN_MOSFET_C_HI D6
#define PIN_TEST_DRIVE D7
#define THROTTLE A4
#define PIN_HALL_A A3
#define PIN_HALL_B A2
#define PIN_HALL_C A1

volatile int hallAState = 0;
volatile int hallBState = 0;
volatile int hallCState = 0;
volatile bool hallSensorsActive = false;
volatile unsigned long lastHallUpdateTime = 0;  // To detect if Hall sensors have stopped
const unsigned long HALL_TIMEOUT = 1000;        // Timeout in milliseconds to detect stopped motor


void updateMotorState() {
    hallAState = digitalRead(PIN_HALL_A);
    hallBState = digitalRead(PIN_HALL_B);
    hallCState = digitalRead(PIN_HALL_C);
    hallSensorsActive = true;  // Motor is spinning, Hall sensors are active
    lastHallUpdateTime = millis();  // Record the last time Hall sensors changed
}

void initMotorController() {
    // Code to initialize PWM or other motor control signals
}

void hallSensorISR1() {
    updateMotorState();
}

void hallSensorISR2() {
    updateMotorState();
}

void hallSensorISR3() {
    updateMotorState();
}

void kickStartMotor(int dutyCycle) {
    // Apply a small kick to get the motor moving
    analogWrite(PIN_MOSFET_A_LO, dutyCycle);  // Activate one phase
    analogWrite(PIN_MOSFET_C_HI, dutyCycle);  // Complementary phase

    delay(50);  // Short delay to kickstart the motor
    analogWrite(PIN_MOSFET_A_LO, 0);
    analogWrite(PIN_MOSFET_C_HI, 0);
}

void stopMotor() {
    // Turn off all MOSFETs to stop the motor
    analogWrite(PIN_MOSFET_A_LO, 0);
    analogWrite(PIN_MOSFET_A_HI, 0);
    analogWrite(PIN_MOSFET_B_LO, 0);
    analogWrite(PIN_MOSFET_B_HI, 0);
    analogWrite(PIN_MOSFET_C_LO, 0);
    analogWrite(PIN_MOSFET_C_HI, 0);

    hallSensorsActive = false;  // Mark the motor as stopped
}

void setMotorControlSignal(int dutyCycle) {
    // Determine which MOSFETs to activate based on Hall sensor states
    if (!hallSensorsActive) {
        // If no Hall sensor activity, kickstart the motor
        kickStartMotor(dutyCycle);
    } else{
        // Regular motor control using Hall sensor feedback
        if (!hallAState && !hallBState && hallCState) { // 001
            analogWrite(PIN_MOSFET_A_LO, dutyCycle); 
            analogWrite(PIN_MOSFET_C_HI, dutyCycle);
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
        } else if (!hallAState && hallBState && !hallCState) { // 010
            analogWrite(PIN_MOSFET_B_HI, dutyCycle);
            analogWrite(PIN_MOSFET_C_LO, dutyCycle);
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
        } else if (!hallAState && hallBState && hallCState) { // 011
            analogWrite(PIN_MOSFET_A_LO, dutyCycle);
            analogWrite(PIN_MOSFET_B_HI, dutyCycle);
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
        } else if (hallAState && !hallBState && !hallCState) { // 100
            analogWrite(PIN_MOSFET_A_HI, dutyCycle);
            analogWrite(PIN_MOSFET_B_LO, dutyCycle);
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
        } else if (hallAState && !hallBState && hallCState) { // 101
            analogWrite(PIN_MOSFET_B_LO, dutyCycle);
            analogWrite(PIN_MOSFET_C_HI, dutyCycle);
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
        } else if (hallAState && hallBState && !hallCState) { // 110
            analogWrite(PIN_MOSFET_A_HI, dutyCycle);
            analogWrite(PIN_MOSFET_C_LO, dutyCycle);
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
        } else if (hallAState && hallBState && hallCState) { // 111
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
        }
        else if (!hallAState && !hallBState && !hallCState) { // 111
            analogWrite(PIN_MOSFET_A_HI, 0);
            analogWrite(PIN_MOSFET_C_LO, 0);
            analogWrite(PIN_MOSFET_A_LO, 0);
            analogWrite(PIN_MOSFET_B_LO, 0);
            analogWrite(PIN_MOSFET_B_HI, 0);
            analogWrite(PIN_MOSFET_C_HI, 0);
        }
    }
}

void controlMotor() {
    int throttleValue = analogRead(THROTTLE);  // Read throttle input

    if (throttleValue > 50) {  // Threshold to start the motor
        int dutyCycle = map(throttleValue, 0, 1023, 0, 255);  // Scale throttle to 0-255

        // Check if Hall sensors have timed out (i.e., motor has stopped)
        if (millis() - lastHallUpdateTime > HALL_TIMEOUT) {
            hallSensorsActive = false;  // Mark the motor as stopped
        }

        // Apply motor control
        setMotorControlSignal(dutyCycle);
    } else {
        // If throttle is too low, stop the motor
        stopMotor();
    }
}

void setup() {
    pinMode(PIN_HALL_A, INPUT);
    pinMode(PIN_HALL_B, INPUT);
    pinMode(PIN_HALL_C, INPUT);
    pinMode(THROTTLE, INPUT);

    pinMode(PIN_MOSFET_A_LO, OUTPUT);
    pinMode(PIN_MOSFET_A_HI, OUTPUT);
    pinMode(PIN_MOSFET_B_LO, OUTPUT);
    pinMode(PIN_MOSFET_B_HI, OUTPUT);
    pinMode(PIN_MOSFET_C_LO, OUTPUT);
    pinMode(PIN_MOSFET_C_HI, OUTPUT);

    // Initialize all MOSFETs to off
    digitalWrite(PIN_MOSFET_A_LO, 0);
    digitalWrite(PIN_MOSFET_A_HI, 0);
    digitalWrite(PIN_MOSFET_B_LO, 0);
    digitalWrite(PIN_MOSFET_B_HI, 0);
    digitalWrite(PIN_MOSFET_C_LO, 0);
    digitalWrite(PIN_MOSFET_C_HI, 0);

    // Attach interrupts to Hall sensor pins
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_A), hallSensorISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_B), hallSensorISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_C), hallSensorISR3, CHANGE);

    // Initialize motor controller (PWM setup, etc.)
    initMotorController();
}

void loop() {
    controlMotor();
    // Perform other tasks (if any)
}
