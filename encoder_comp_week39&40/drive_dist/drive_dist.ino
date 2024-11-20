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

const int X_CENTER = 500;   // Center value for X-axis
const int Y_CENTER = 487;   // Center value for Y-axis
const int DEADZONE = 20;    // Threshold to ignore small joystick movements

void setup() {
  Serial.begin(9600);
  lcd.begin(20, 4);

  // Initialize motor pins
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  
  pinMode(joyButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(joyButton), buttonISR, FALLING);
}

void loop() {
  if (stop_motors) {
    // If the button is pressed, stop the motors
    analogWrite(Motor_L_pwm_pin, 0);
    analogWrite(Motor_R_pwm_pin, 0);
    return;
  }

  // Read joystick values
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);

  // Map joystick Y values to motor speeds (-255 to 255)
  int motorSpeedL = map(yValue, 0, 1023, -255, 255);
  int motorSpeedR = motorSpeedL;  // Initially, both motors move forward/backward equally

  // Apply turning only if the X-axis is outside the deadzone
  if (abs(xValue - X_CENTER) > DEADZONE) {
    int turningFactor = map(xValue, 0, 1023, -255, 255);
    motorSpeedL += turningFactor;
    motorSpeedR -= turningFactor;
  }

  if (buttonPressed) {
    motorsRun();
    buttonPressed = false;
  }

  // Set motor directions based on speed
  digitalWrite(Motor_L_dir_pin, motorSpeedL >= 0 ? Motor_forward : Motor_return);
  digitalWrite(Motor_R_dir_pin, motorSpeedR >= 0 ? Motor_forward : Motor_return);

  // Set motor speeds (absolute value for PWM)
  analogWrite(Motor_L_pwm_pin, abs(motorSpeedL));
  analogWrite(Motor_R_pwm_pin, abs(motorSpeedR));

  // Display joystick values on LCD
  float xVoltage = (xValue / 1023.0) * 5.0;
  float yVoltage = (yValue / 1023.0) * 5.0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(xValue);
  lcd.print(" ");
  lcd.print(xVoltage, 2);
  lcd.print("V");

  lcd.setCursor(0, 1);
  lcd.print("Y:");
  lcd.print(yValue);
  lcd.print(" ");
  lcd.print(yVoltage, 2);
  lcd.print("V");

}

// Interrupt Service Routine for the joystick button
void buttonISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    stop_motors = !stop_motors;  // Toggle motor stop state
    buttonPressed = true;        // Set buttonPressed to true to trigger motorsRun()
    Serial.println("Button pressed!");
    lastDebounceTime = currentTime;
  }
}

void motorsRun() {
  leftWheelForward(50, 1000);
  rightWheelForward(50, 1000);
  bothWheelsBack(25, 2000);
}

void leftWheelForward(int speedPercentage, int duration) {
  int pwmValue = map(speedPercentage, 0, 100, 0, 255);
  digitalWrite(Motor_L_dir_pin, Motor_forward);
  analogWrite(Motor_L_pwm_pin, pwmValue);
  delay(duration);
  analogWrite(Motor_L_pwm_pin, 0);
}


void rightWheelForward(int speedPercentage, int duration) {
  int pwmValue = map(speedPercentage, 0, 100, 0, 255);
  digitalWrite(Motor_R_dir_pin, Motor_forward);
  analogWrite(Motor_R_pwm_pin, pwmValue);
  delay(duration);
  analogWrite(Motor_R_pwm_pin, 0);
}

void bothWheelsBack(int speedPercentage, int duration) {
  int pwmValue = map(speedPercentage, 0, 100, 0, 255);
  digitalWrite(Motor_L_dir_pin, Motor_return);
  digitalWrite(Motor_R_dir_pin, Motor_return);
  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);
  delay(duration);
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}
