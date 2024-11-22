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
#define Motor_return         0
#define Motor_L_dir_pin      7
#define Motor_R_dir_pin      8
#define Motor_L_pwm_pin      9
#define Motor_R_pwm_pin      10

void setup() {
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

void loop() {
  uint8_t compassBearingRaw = readCompassBearing();
  float compassBearingDegrees = (compassBearingRaw * 360.0) / 255.0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("degrees: ");
  lcd.print(compassBearingDegrees);
  lcd.print((char)223);

  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n'); 
    Serial.print("Message received, content: ");  
    Serial.println(message);
    
    int pos_lcd = message.indexOf("LCD");
    int pos_drive_dist = message.indexOf("Move");
    int pos_turn = message.indexOf("Turn");
    int pos_find_north = message.indexOf("find");

    if (pos_lcd > -1) {
      Serial.println("Command = LCD ");
      pos_lcd = message.indexOf(":");
      String stat = message.substring(pos_lcd + 1);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(stat);
    }
    else if (pos_drive_dist > -1) {
      Serial.println("Command = Move ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = Move ");
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
    else if (pos_turn > -1) {
      Serial.print("Command = Turn ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = Turn ");
      pos_turn = message.indexOf(":");

      if (pos_turn > -1) {
        String stat = message.substring(pos_turn + 1);
        int stat_int = stat.toInt();
        lcd.print(stat_int);
        turnExact(stat_int);
      }
    }
    else if (pos_find_north > -1) {
      Serial.print("Command = find ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = find ");
      pos_find_north = message.indexOf(":");

      if (pos_find_north > -1) {
        String stat = message.substring(pos_turn + 1);
        lcd.print(stat);
        find_heading(0);  // 0 degrees is North
      }
    }
    else {
      Serial.println("error");
      lcd.print("error");
    }
  }
  delay(100);
}

void turnExact(int angle) {
  if (angle == 0) return;  // No turn needed
  
  uint8_t initialBearing = readCompassBearing(); 
  float startHeading = (initialBearing * 360.0) / 255;
  float lastHeading = startHeading;
  float currentHeading;
  float accumulatedAngle = 0;  // Track total angle turned
  
  Serial.print("Start heading: "); Serial.println(startHeading);
  Serial.print("Target angle: "); Serial.println(angle);

  // Determine direction and target angle
  bool isRightTurn = (angle > 0);
  float targetAngle = abs(angle);

  while (abs(accumulatedAngle) < targetAngle) {
    currentHeading = (readCompassBearing() * 360.0) / 255;
    
    // Calculate the difference since last reading
    float deltaAngle = currentHeading - lastHeading;
    
    // Handle wrap-around cases
    if (isRightTurn) {
      if (deltaAngle < -180) {  // Wrapped from 359 to 0
        deltaAngle += 360;
      } else if (deltaAngle > 180) {  // Wrong direction
        deltaAngle -= 360;
      }
    } else {  // Turning left
      if (deltaAngle > 180) {  // Wrapped from 0 to 359
        deltaAngle -= 360;
      } else if (deltaAngle < -180) {  // Wrong direction
        deltaAngle += 360;
      }
    }
    
    // Accumulate the angle turned
    accumulatedAngle += deltaAngle;
    lastHeading = currentHeading;

    // Turn in the specified direction
    if (isRightTurn) {
      turnRight(25);
    } else {
      turnLeft(25);
    }
    
    // Debug output
    Serial.print("Current: "); Serial.print(currentHeading);
    Serial.print(" Accumulated: "); Serial.println(abs(accumulatedAngle));
    
    delay(50);  // Small delay for stability
  }
  
  stopMotors();
  Serial.println("Turn completed!");
}

void find_heading(int targetBearing) {
  float currentHeading;

  while (true) {
    currentHeading = (readCompassBearing() * 360.0) / 255;

    // Calculate shortest angle difference
    float angleDifference = currentHeading - targetBearing;
    if (angleDifference > 180) angleDifference -= 360;
    else if (angleDifference < -180) angleDifference += 360;

    if (abs(angleDifference) <= OFFSET) {
      stopMotors();
      Serial.println("Reached target heading.");
      break;
    }

    // Turn in the direction of shortest path
    if (angleDifference > 0) {
      turnLeft(25);
    } else {
      turnRight(25);
    }
    delay(50);
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

  while (pulseCountR < targetPulses) {
    // waiting to reach the target distance
  }

  stopMotors();
}

void stopMotors() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}