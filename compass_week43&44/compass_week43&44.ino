#include <Wire.h>
#include <LiquidCrystal.h>

const int ADDRESS = 0x60;
LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
const int joyX = A8;
const int joyY = A9;
const int joyButton = 19;

const int OFFSET = 2;

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

  find_heading(90);

  delay(50);
}

void find_heading(int given_degree) {
  uint8_t compassRaw = readCompassBearing();
  float currentHeading = (compassRaw * 360.0) / 255;

  if (abs(currentHeading - given_degree) > OFFSET) {
    if ((currentHeading > given_degree && currentHeading - given_degree <= 180) ||
    (currentHeading < given_degree && given_degree - currentHeading > 180)) {
      Serial.println(currentHeading);
      Serial.println(" tunrn left");
      turnLeft(20);
    }
    else {
      Serial.println("\n");
      Serial.println(currentHeading);
      Serial.println(" tunrn right");
      turnRight(20);
    }
  } 
  else {
    stopMotors();
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
}
