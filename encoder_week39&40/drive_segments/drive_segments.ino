#include <LiquidCrystal.h>

#define Motor_forward         1
#define Motor_return          0
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
const int joyX = A8;
const int joyY = A9;
const int joyButton = 19;

volatile bool stop_motors = false;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

const int X_CENTER = 500;  
const int Y_CENTER = 487;
const int DEADZONE = 20;   

const int STOP_TIME = 3000;
const int PULSE_AVG = 13.2;

volatile int pulseCountR = 0;
volatile int pulseCountL = 0;
unsigned long startTime = 0;

#define ENCA_R 2  // Right motor encoder A connected to pin 2
#define ENCA_L 3  // Left motor encoder A connected to pin 3

void encoderISR() {
  pulseCountR++;
}

void encoderISRleft() {
  pulseCountL++;
}

void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4);

  stopCar();

  // Initialize motor pins
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);

  pinMode(ENCA_R, INPUT);
  pinMode(ENCA_L, INPUT);

  pinMode(joyButton, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_R), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), encoderISRleft, RISING);
  attachInterrupt(digitalPinToInterrupt(joyButton), buttonISR, FALLING);
}

void loop() {
  if (buttonPressed) {
    pulseCountR = 0;
    pulseCountL = 0;
    driveSegments();
    buttonPressed = false;
  }
  
  // Display the pulse and distance on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pulses R: ");
  lcd.print(pulseCountR);

  float cmDistance = pulseCountR / PULSE_PER_CM;
  lcd.setCursor(0, 1);
  lcd.print("Distance cm: ");
  lcd.print(cmDistance);
}

void buttonISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
}

void driveSegments() {
  // Segment 1: Drive at 50% speed for the first 10 cm
  setMotorSpeed(50);
  while (pulseCountR < 10 * PULSE_PER_CM);

  // Segment 2: Drive at 25% speed for the next 5 cm
  setMotorSpeed(25);
  while (pulseCountR < 15 * PULSE_PER_CM);

  // Segment 3: Drive at 75% speed for the next 15 cm
  setMotorSpeed(75);
  while (pulseCountR < 30 * PULSE_PER_CM);

  stopCar();
}
void setMotorSpeed(int speedPercentage) {
  int pwmValue = map(speedPercentage, 0, 100, 0, 255);
  digitalWrite(Motor_L_dir_pin, Motor_forward);
  digitalWrite(Motor_R_dir_pin, Motor_forward);
  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);
}

void stopCar() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
  Serial.println("Car stopped.");
}
