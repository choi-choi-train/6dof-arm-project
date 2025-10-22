#include <Arduino.h>
#include <stepper.h>
#include <TeensyTimerTool.h>
#include <cmath>
#include <vector>

#define BUTTON_PIN 15
#define POT_PIN 14
#define LED_PIN 13
#define STEP_PIN 19
#define DIR_PIN 20
#define ENABLE_PIN 16

using namespace std;
using namespace TeensyTimerTool;
PeriodicTimer t_main(TCK_RTC);

bool buttonHeld = false;
int startTime = 0;
int releaseTime = 0;
float maxSpeed = 7500.0;

void potentiometer_button_control();
void angle_homing();

Stepper motor_1(STEP_PIN, DIR_PIN, maxSpeed);

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(POT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    motor_1.setPullInSpeed(1000.0); //speed to which the stepper can instantly jump without ramp-up

    while(!Serial) {;}
    TeensyTimerTool::attachErrFunc(TeensyTimerTool::ErrorHandler(Serial));
    // t_main.begin(potentiometer_button_control, 1000);
    // Serial.println("STARTING PROGRAM");
    t_main.begin(angle_homing, 1000);
}

void potentiometer_button_control() {
    int read = analogRead(POT_PIN); //1023 -> 0 when wiping left to right
    float targetSpeed = ((float)(511.5-read)/511.5) * maxSpeed;

    bool button_pressed = digitalRead(BUTTON_PIN) == LOW;
    if (button_pressed) {
        digitalWrite(LED_PIN, HIGH);
        if (!buttonHeld) {
            Serial.printf("STARTING SPEED: %f\n", motor_1.currentSpeed());
            Serial.printf("ACCELERATING TO SPEED %f\n", targetSpeed);
            buttonHeld = true;
            motor_1.setStartingSpeed(motor_1.currentSpeed());
            startTime = micros();
        }
        motor_1.setTargetSpeed(targetSpeed);
        Serial.printf("^^^ %f\n", motor_1.currentSpeed());
        // motor_1.bezcelerate(startTime);
    } else {
        digitalWrite(LED_PIN, LOW);
        if (buttonHeld) {
            buttonHeld = false;
            releaseTime = micros();
            motor_1.setStartingSpeed(motor_1.currentSpeed());
            Serial.println("DECELERATING....");
        }
        motor_1.setTargetSpeed(0);
        Serial.printf("vvv %f\n", motor_1.currentSpeed());
        // motor_1.bezcelerate(releaseTime);
    }
}

bool event_happening = false;
bool sequenceRunning = false;
int angle_sequence[7] = {45*60, -90*60, 45*60, 80*60, -160*60, 80*60};
int idx = 0;
void angle_homing() {
    bool buttonPressed = digitalRead(BUTTON_PIN) == LOW;
    if (event_happening) {
        digitalWrite(LED_PIN, HIGH);
        if (motor_1.doneMoving()) {
            motor_1.stop();
            idx++;
            if (idx < sizeof(angle_sequence)/sizeof(angle_sequence[0])) {
                int angle = angle_sequence[idx];
                motor_1.moveByAngle(angle);
            } else {
                // finished sequence
                event_happening = false;
                sequenceRunning = false;
                buttonHeld = false;
                idx = 0;
                motor_1.stop();
                digitalWrite(ENABLE_PIN, HIGH);
            }
        }
    } else {
        digitalWrite(LED_PIN, LOW);
    }

    if (!buttonHeld && buttonPressed) {
        // Serial.println(startTime);
        motor_1.setTargetSpeed(7500);
        digitalWrite(ENABLE_PIN, LOW);
        motor_1.moveByAngle(angle_sequence[idx]);
        buttonHeld = true;
        event_happening = true;
    }
}

void loop() {
    // sanitychecking
    // bool button_pressed = digitalRead(BUTTON_PIN) == LOW;
    // if (button_pressed) {
    //     digitalWrite(LED_PIN, HIGH);
    //     digitalWrite(STEP_PIN, HIGH);
    //     delayMicroseconds(1000); // 1–2 µs minimum for TMC2209
    //     digitalWrite(STEP_PIN, LOW);
    //     delayMicroseconds(1000);
    // } else {
    //     digitalWrite(LED_PIN, LOW);
    // }
    ;
}