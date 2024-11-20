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

volatile int pulseCount = 0;
unsigned long startTime = 0;

#define ENCA_R 2  // Right motor encoder A connected to pin 2
#define ENCA_L 3  // Left motor encoder A connected to pin 3

void encoderISR() {
  pulseCount++;
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

  attachInterrupt(digitalPinToInterrupt(ENCA_R), encoderISR, RISING);

  startTime = millis();

  driveForward();
}

void loop() {
  
  if (millis() - startTime >= STOP_TIME) {
    
    stopCar();

    // Print the pulse count and calculated distance information
    Serial.print("Pulses: ");
    Serial.println(pulseCount);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pulses: ");
    lcd.print(pulseCount);

    float cmDistance = pulseCount / PULSE_AVG;
    lcd.setCursor(0, 1);
    lcd.print("Distance cm: ");
    lcd.print(cmDistance);

    while (true);
  }
}

void driveForward() {
  digitalWrite(Motor_L_dir_pin, Motor_forward);
  digitalWrite(Motor_R_dir_pin, Motor_forward);
  analogWrite(Motor_L_pwm_pin, 255); 
  analogWrite(Motor_R_pwm_pin, 255);
}

void stopCar() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
  Serial.println("Car stopped.");
}
