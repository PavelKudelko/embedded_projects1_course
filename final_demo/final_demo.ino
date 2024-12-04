#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
// compass address
const int ADDRESS = 0x60;
// offset for compass
const int OFFSET = 2;
// joystick pins
const int joyX = A8;
const int joyY = A9;
const int joyButton = 19;
// pulse count per cm (found by experiments)
const int PULSE_AVG = 13.2;

#define JOYSTICK_MODE 0
#define SERIAL_MODE 1
// encoder pins for pulse counting
#define ENCA_R 2 
#define ENCA_L 3

volatile int controlMode = SERIAL_MODE;
volatile int pulseCountR = 0;
volatile int pulseCountL = 0;
unsigned long startTime = 0;
volatile bool stop_motors = false;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE = 100;
// motors
#define Motor_forward         1
#define Motor_return         0
#define Motor_L_dir_pin      7
#define Motor_R_dir_pin      8
#define Motor_L_pwm_pin      9
#define Motor_R_pwm_pin      10

const int X_CENTER = 500;   // Center value for X-axis
const int Y_CENTER = 487;   // Center value for Y-axis
const int DEADZONE = 20;    // Threshold to ignore small joystick movements

void encoderISR() {
  pulseCountR++;
}

void encoderISRleft() {
  pulseCountL++;
}

void setup() {
  // delay for stability (mostly for compass)
  delay(2000);
  Serial.begin(9600);
  pinMode(ENCA_R, INPUT);
  pinMode(ENCA_L, INPUT);
  // motors setup
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  // ISR functions (pulse count for L and R and joy button)
  attachInterrupt(digitalPinToInterrupt(ENCA_R), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), encoderISRleft, RISING);
  attachInterrupt(digitalPinToInterrupt(joyButton), buttonISR, FALLING);
  pinMode(joyButton, INPUT_PULLUP);

  lcd.begin(20, 4);
  Wire.begin();
}

void buttonISR() {
  // reset pulse count
  pulseCountR = 0;
  pulseCountL = 0;
  // handle button bouncing
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > DEBOUNCE) {
    controlMode = (controlMode == JOYSTICK_MODE) ? SERIAL_MODE : JOYSTICK_MODE;
    lastDebounceTime = currentTime;
  }
}

void loop() {
  if (controlMode == JOYSTICK_MODE) {
    handleJoystickControl();
  } else if (controlMode == SERIAL_MODE) {
    handleSerialControl();
  }
  delay(100);  // Add a short delay for stability
}
// Commands handling part
// that are recieved from esp in serial monitor
void handleSerialControl() {
  // lcd controll
  lcd.clear();
  lcd.setCursor(0, 0);
  float compassBearingDegrees = (readCompassBearing() * 360.0) / 255.0;
  displayCompassInfo(compassBearingDegrees);
  displayPulseInfo();
  lcd.setCursor(0, 3);
  lcd.print("Serial");
  // Check if there is incoming data from the serial monitor
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    Serial.print("Message received, content: ");
    Serial.println(message);
    // Find the positions of the command keywords in the message
    int pos_lcd = message.indexOf("LCD");
    int pos_drive_dist = message.indexOf("Move");
    int pos_turn = message.indexOf("Turn");
    int pos_find_north = message.indexOf("find");
    // Handle LCD message commands
    // In not used in the esp
    if (pos_lcd > -1) {
      Serial.println("Command = LCD ");
      pos_lcd = message.indexOf(":");
      String stat = message.substring(pos_lcd + 1); // Extract message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(stat);
    }
    // Handle Move command
    else if (pos_drive_dist > -1) {
      Serial.println("Command = Move ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = Move ");
      pos_drive_dist = message.indexOf(":");

      if (pos_drive_dist > -1) {
        // Extract the distance
        String stat = message.substring(pos_drive_dist + 1);
        int stat_int = stat.toInt();

        if (stat_int <= 20 && stat_int >= -20) {
          lcd.print(stat_int);
          driveDistance(stat_int, 100);
        } else {
          lcd.setCursor(0, 1);
          lcd.print("Error: invalid value");
        }
      }
      else {
        lcd.setCursor(0, 1);
        lcd.print("Error: missing value");
      }
    } 
    // Handle Turn command
    else if (pos_turn > -1) {
      Serial.print("Command = Turn ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = Turn ");
      pos_turn = message.indexOf(":");

      if (pos_turn > -1) {
        String stat = message.substring(pos_turn + 1); // Extract the angle
        // angle str to int convert
        int stat_int = stat.toInt();
        // Since it's a slide bar on the web page (0 to 360)
        // validating angle and error handling is not needed here
        lcd.print(stat_int);
        turnExact(stat_int);
      } else {
        lcd.setCursor(0, 1);
        lcd.print("Error: missing angle");
      }
    // Handle find North command
    } else if (pos_find_north > -1) {
      Serial.print("Command = find ");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Command = find ");
      pos_find_north = message.indexOf(":");

      if (pos_find_north > -1) {
        String stat = message.substring(pos_turn + 1);
        lcd.print(stat);
        find_heading(0);  // Find the heading to 0 degrees (North)
      }
    }
    // Handle invalid or unrecognized commands
    else {
      lcd.setCursor(0, 0);
      lcd.print("Error: unknown cmd");
    }
  }
}
void handleJoystickControl() {
  // Joystick control logic (your existing joystick implementation)
  int xValue = analogRead(joyX);
  int yValue = analogRead(joyY);

  // Map joystick Y values to motor speeds (-255 to 255)
  int motorSpeedL = map(yValue, 0, 1023, -255, 255);
  int motorSpeedR = motorSpeedL;

  // Apply turning only if the X-axis is outside the deadzone
  if (abs(xValue - X_CENTER) > DEADZONE) {
    int turningFactor = map(xValue, 0, 1023, -255, 255);
    motorSpeedL += turningFactor;
    motorSpeedR -= turningFactor;
  }

  // Set motor directions based on speed
  digitalWrite(Motor_L_dir_pin, motorSpeedL >= 0 ? Motor_forward : Motor_return);
  digitalWrite(Motor_R_dir_pin, motorSpeedR >= 0 ? Motor_forward : Motor_return);

  // Set motor speeds (absolute value for PWM)
  analogWrite(Motor_L_pwm_pin, abs(motorSpeedL));
  analogWrite(Motor_R_pwm_pin, abs(motorSpeedR));
  // lcd prints
  float compassBearingDegrees = (readCompassBearing() * 360.0) / 255.0;
  displayCompassInfo(compassBearingDegrees);
  displayPulseInfo();
  lcd.setCursor(0, 3);
  lcd.print("JoyStick");
}

void displayCompassInfo(float compassBearingDegrees) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Degrees: ");
  lcd.print(compassBearingDegrees);
  lcd.print((char)223); // Degree symbol
  
  lcd.setCursor(0, 1);
  lcd.print("Direction: ");
  lcd.print(getDirection(compassBearingDegrees));
}

String getDirection(float degrees) {
  if (degrees >= 337.5 || degrees < 22.5) return "N";
  if (degrees >= 22.5 && degrees < 67.5) return "NE";
  if (degrees >= 67.5 && degrees < 112.5) return "E";
  if (degrees >= 112.5 && degrees < 157.5) return "SE";
  if (degrees >= 157.5 && degrees < 202.5) return "S";
  if (degrees >= 202.5 && degrees < 247.5) return "SW";
  if (degrees >= 247.5 && degrees < 292.5) return "W";
  if (degrees >= 292.5 && degrees < 337.5) return "NW";
  return "Unknown";
}

void displayPulseInfo() {
  lcd.setCursor(0, 2);
  lcd.print("Pulses: R:"); 
  lcd.print(pulseCountR); 
  lcd.print(" L:"); 
  lcd.print(pulseCountL);
}

void turnExact(int angle) {
  if (angle == 0) return;  // No turn needed
  
  uint8_t initialBearing = readCompassBearing(); 
  float startHeading = (initialBearing * 360.0) / 255;
  float lastHeading = startHeading;
  float currentHeading;
  float accumulatedAngle = 0;  // Track total angle turned

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
