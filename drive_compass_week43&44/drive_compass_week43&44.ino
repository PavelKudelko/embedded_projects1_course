#include <Wire.h>
#include <LiquidCrystal.h>

const int ADDRESS = 0x60;
LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
const int joyX = A8;
const int joyY = A9;
const int joyButton = 19;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

const int OFFSET = 2;

const int STOP_TIME = 3000;
const int PULSE_AVG = 13.2;

volatile int pulseCountR = 0;
volatile int pulseCountL = 0;
unsigned long startTime = 0;

#define ENCA_R 2 
#define ENCA_L 3 

void encoderISR() {
  pulseCountR++;
}

void encoderISRleft() {
  pulseCountL++;
}

volatile bool stop_motors = false;
#define Motor_forward         1
#define Motor_return          0
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

void setup() {
  Serial.begin(9600);
  Wire.begin(); 

  lcd.begin(20, 4);

  pinMode(ENCA_R, INPUT);
  pinMode(ENCA_L, INPUT);

  pinMode(joyButton, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_R), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), encoderISRleft, RISING);
  attachInterrupt(digitalPinToInterrupt(joyButton), buttonISR, FALLING);

  delay(2000);
}

void loop() {
  uint8_t compassBearingRaw = readCompassBearing();
  float compassBearingDegrees = (compassBearingRaw * 360.0) / 255.0;
  // Serial.print("Compass Bearing: ");
  // Serial.println(compassBearingRaw);
  String direction = getDirection(compassBearingDegrees);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("degrees: ");
  lcd.print(compassBearingDegrees);
  lcd.print((char)223);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print("Direction: ");
  lcd.print(direction);

  if (buttonPressed) {
    buttonPressed = false;
    executeRoute();
  }

  delay(500);
}

void executeRoute() {
  driveDistance(20, 50); 
  delay(1000);
  turnHeading(110);
  delay(1000);  

  driveDistance(13, 25); 
  delay(1000);
  turnHeading(110);

  driveDistance(20, 75); 

  // turnHeading(110);     
  stopMotors();          
}

void driveDistance(int cm, int speedPercent) {
  int targetPulses = cm * PULSE_AVG;
  pulseCountR = 0;      
  pulseCountL = 0;

  int pwmValue = map(speedPercent, 0, 100, 0, 255);

  digitalWrite(Motor_L_dir_pin, Motor_forward);
  digitalWrite(Motor_R_dir_pin, Motor_forward);
  
  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);

  while (pulseCountR < targetPulses) {
    // waiting reaching distance
  }
  
  Serial.println(pulseCountR);
  stopMotors();
}

void turnHeading(int angle) {
  uint8_t initialBearing = readCompassBearing();
  uint8_t targetBearing = (initialBearing + angle) % 360;

  find_heading(targetBearing);  // Adjust heading to target
}

void buttonISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
  }
}

void find_heading(int targetBearing) {
  uint8_t compassRaw;
  float currentHeading;

  while (true) {
    compassRaw = readCompassBearing();
    currentHeading = (compassRaw * 360.0) / 255;

    // Calculate the smallest angle difference (considering wrap-around at 360 degrees)
    float angleDifference = currentHeading - targetBearing;
    if (angleDifference > 180) angleDifference -= 360;
    else if (angleDifference < -180) angleDifference += 360;

    // Check if we are within the acceptable range
    if (abs(angleDifference) <= OFFSET) {
      stopMotors();
      Serial.println("Reached target heading.");
      break;
    }

    // Turn in the appropriate direction
    if (angleDifference > 0) {
      Serial.println("Turning left");
      turnLeft(25);
    } else {
      Serial.println("Turning right");
      turnRight(25);
    }

    delay(50);  // Small delay to avoid rapid polling and stabilize readings
  }
}

void turnLeft(int speedPercent) {
  int pwmValue = map(speedPercent, 0, 100, 0, 255);

  digitalWrite(Motor_L_dir_pin, Motor_forward);
  digitalWrite(Motor_R_dir_pin, Motor_return);

  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);
}

void turnRight(int speedPercent) {
  int pwmValue = map(speedPercent, 0, 100, 0, 255);

  digitalWrite(Motor_R_dir_pin, Motor_forward);
  digitalWrite(Motor_L_dir_pin, Motor_return);

  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);
}

void stopMotors() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
  Serial.println("Car stopped.");
}

uint8_t readCompassBearing() {
  Wire.beginTransmission(ADDRESS); 
  Wire.write(0x01);
  Wire.endTransmission(false);
  
  Wire.requestFrom(ADDRESS, 1, true);
  if (Wire.available()) {
    return Wire.read(); 
  }

  return 0; 
}

String getDirection(uint8_t bearing) {
  if (bearing >= 338 && bearing < 23) return "N";
  else if (bearing >= 23 && bearing < 68) return "NE";
  else if (bearing >= 68 && bearing < 113) return "E";
  else if (bearing >= 113 && bearing < 158) return "SE";
  else if (bearing >= 158 && bearing < 203) return "S";
  else if (bearing >= 203 && bearing < 248) return "SW";
  else if (bearing >= 248 && bearing < 293) return "W";
  else if (bearing >= 293 && bearing < 338) return "NW";
  else return "Unknown";  // Fallback in case of an unexpected value
}
