#include <Wire.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
const int ADDRESS = 0x60;
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

void setup(){
  delay(2000);
  
  Serial.begin(9600);
  Serial.println("Write something to the serial monitor.");
  pinMode(ENCA_R, INPUT);
  pinMode(ENCA_L, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), encoderISRleft, RISING);
  lcd.begin(20, 4);
  Wire.begin();
}

void loop(){

  uint8_t compassBearingRaw = readCompassBearing();
  float compassBearingDegrees = (compassBearingRaw * 360.0) / 255.0;
  // Serial.print("Compass Bearing: ");
  // Serial.println(compassBearingRaw);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("degrees: ");
  lcd.print(compassBearingDegrees);
  lcd.print((char)223);

  if (Serial.available() > 0){
    String message = Serial.readStringUntil('\n'); 
    Serial.print("Message received, content: ");  
    Serial.println(message);
    // lcd command
    int pos_lcd = message.indexOf("LCD");
    // drive command
    int pos_drive_dist = message.indexOf("dist");
    // turn command
    int pos_turn = message.indexOf("degree");

    if (pos_lcd > -1) {
      Serial.println("Command = LCD ");
      pos_lcd = message.indexOf(":");

      String stat = message.substring(pos_lcd + 1);
      // print to lcd
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(stat);
    }
    if (pos_drive_dist > -1) {
      Serial.println("Command = dist ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = dist ");
      pos_drive_dist = message.indexOf(":");

      if (pos_drive_dist > -1) {
        String stat = message.substring(pos_drive_dist + 1);
        int stat_int = stat.toInt();

        if (stat_int <= 20 && stat_int >= -20) {
          lcd.print(stat_int);
          driveDistance(stat_int, 100);
        }
        else {
          lcd.setCursor(0, 1);
          lcd.print("Error: invalid value");
        }
      }
    }
    if (pos_turn > -1) {
      Serial.print("Command = degree ");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Command = degree ");
      pos_turn = message.indexOf(":");

      if (pos_turn > -1) {
        String stat = message.substring(pos_turn + 1);
        int stat_int = stat.toInt();

        if (stat_int >= 0 && stat_int <= 360) {
          lcd.print(stat_int);
          find_heading(stat_int);
        }
        else {
          lcd.setCursor(0, 1);
          lcd.print("Error: invalid value");
        }
      }
    }
    else{
	    Serial.println("error");
      lcd.print("error");
    }
  }

  delay(100);
}

// void turnHeading(int angle) {
//   uint8_t initialBearing = readCompassBearing();
//   uint8_t targetBearing = (initialBearing + angle) % 360;

//   find_heading(targetBearing);  // Adjust heading to target
// }

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
      turnLeft(25);
    } else {
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

void driveDistance(int cm, int speedPercent) {
  int targetPulses = abs(cm) * PULSE_AVG;
  pulseCountR = 0;      
  pulseCountL = 0;
  int pwmValue = map(speedPercent, 0, 100, 0, 255);

  if (cm > 0) {
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);
  } else {
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_return);
  }
  
  analogWrite(Motor_L_pwm_pin, pwmValue);
  analogWrite(Motor_R_pwm_pin, pwmValue);

  // Wait until the desired pulse count is reached
  while (pulseCountR < targetPulses) {
    // waiting to reach the target distance
  }

  stopMotors();
}


void stopMotors() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}