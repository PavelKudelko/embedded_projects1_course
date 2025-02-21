#include <Wire.h>
#include <LIDARLite.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

LIDARLite myLidarLite;
LiquidCrystal lcd(37, 36, 35, 34, 33, 32);
// compass address
const int ADDRESS = 0x60;
// joystick pins
const int joyX = A8;
const int joyY = A9;
const int joyButton = 19;
// pulse count per cm (found by experiments)
const int PULSE_AVG = 13.2;
// encoder pins for pulse counting
#define ENCA_R 2
#define ENCA_L 3
volatile int pulseCountR = 0;
volatile int pulseCountL = 0;
unsigned long startTime = 0;
volatile bool stop_motors = false;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const int DEBOUNCE = 200;
// motors
#define Motor_forward         1
#define Motor_return          0
#define Motor_L_dir_pin      7
#define Motor_R_dir_pin      8
#define Motor_L_pwm_pin      9
#define Motor_R_pwm_pin      10

const int potPin = A1;

// offset for getting real values
volatile int COMPASS_READING_OFFSET = 0;

const int LIDAR_SAMPLES = 30;
int lidar_vals[LIDAR_SAMPLES] = {0};
int curIndx = 0;

// length of the car
const int ROBOT_LENGTH = 20;

//lidar correction
const int LIDAR_CORR_VAL = 10;

// start reading from the first byte (address 0) of the EEPROM
int address = 0;
byte value;

void encoderISR() {
  pulseCountR++;
}

void encoderISRleft() {
  pulseCountL++;
}
// compass reading handling
uint8_t readCompassBearing() {
  Wire.beginTransmission(ADDRESS);
  Wire.write(0x01);
  Wire.endTransmission(false);

  Wire.requestFrom(ADDRESS, 1, true);
  if (Wire.available()) {
    return Wire.read();
  }
  // If connection fails, return 0
  return 0;
}

// Calibrate compass
void calibrateCompass() {
  uint8_t initVal = readCompassBearing();
  if (initVal <= 127) {
    COMPASS_READING_OFFSET = -initVal; // Offset for values closer to 0
  } else {
    COMPASS_READING_OFFSET = 255 - initVal; // Offset for values closer to 255
  }
  Serial.print("Compass Calibration Offset: ");
  Serial.println(COMPASS_READING_OFFSET);
}

// Get corrected compass bearing in degrees
int getCorrectedCompassBearing() {
  int rawBearing = readCompassBearing();
  int correctedRaw = (rawBearing + COMPASS_READING_OFFSET) % 255; // Apply offset in 0–255 range
  if (correctedRaw < 0) {
    correctedRaw += 255; // Ensure value is non-negative
  }
  // Convert to degrees
  float correctedDegrees = (correctedRaw * 360.0) / 255.0;
  return static_cast<int>(correctedDegrees); // Return as integer
}
int avg_lidar_val(int vals[], int size) {
  int sum = 0;
  for (int i = 0; i < size; i++) {
    sum += vals[i];
  }
  return sum / size;
}

void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial1.begin(9600);
  // define second serial monitor
  // Serial2.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLidarLite.configure(0); // Change this number to try out alternate configurations

  pinMode(ENCA_R, INPUT);
  pinMode(ENCA_L, INPUT);
  // motors setup
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  // potentialmeter setup
  pinMode(potPin, INPUT);
  // ISR functions (pulse count for L and R and joy button)
  attachInterrupt(digitalPinToInterrupt(ENCA_R), encoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), encoderISRleft, RISING);
  attachInterrupt(digitalPinToInterrupt(joyButton), buttonISR, FALLING);
  pinMode(joyButton, INPUT_PULLUP);
  lcd.begin(20, 4);
  Wire.begin();
  // Compass calibration
  calibrateCompass();
}

void buttonISR() {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > DEBOUNCE) {
    buttonPressed = true;
    lastDebounceTime = currentTime;
    lcd.clear();
  }
}

void loop() {
  handleSerialControl();

  if (buttonPressed) {
    stopMotors();
    buttonPressed = false;
  }
  Serial1.println("LIDAR:" + String(get_dist()));
  Serial1.println("COMPASS:" + String(getCorrectedCompassBearing()));
  delay(100); // Delay to control update frequency
}

void displayLidarValues() {
    int distance = get_dist();
    lcd.setCursor(0, 3);
    lcd.print("LIDAR: ");
    lcd.print(distance);
    lcd.print(" cm  "); 
}


void turnExact(int angle, String direction = "") {
  if (angle == 0) return;  // No turn needed
  int startHeading = getCorrectedCompassBearing();
  int currentHeading = startHeading;
  int accumulatedAngle = 0;
  // decide turn dir. If no str provided decide dir by the closest
  bool isRightTurn = (direction == "") ? (angle > 0) : (direction == "right");  int targetAngle = abs(angle);
  // Loop until the accumulated angle reaches the target angle
  while (accumulatedAngle < targetAngle) {
    int newHeading = getCorrectedCompassBearing();
    int deltaAngle = newHeading - currentHeading;

    if (deltaAngle > 180) {
      deltaAngle -= 360;
    } else if (deltaAngle < -180) {
      deltaAngle += 360;
    }

    if (isRightTurn && deltaAngle > 0) {
      accumulatedAngle += deltaAngle;
    } else if (!isRightTurn && deltaAngle < 0) {
      accumulatedAngle -= deltaAngle;
    }

    currentHeading = newHeading;
    if (isRightTurn) {
      turnRight(25); 
    } else {
      turnLeft(25);
    }
    delay(50);
  }

  stopMotors(); // Stop the motors after the turn
}

void handleSerialControl() {
  // Check if there is incoming data from the serial monitor
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    Serial.print("Message received, content: ");
    Serial.println(message);

    int pos_drive_dist = message.indexOf("MOVE");
    int pos_turn = message.indexOf("TURN");

    if (pos_drive_dist > -1) {
      Serial.println("Command = Move");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = Move");
      pos_drive_dist = message.indexOf(":");

      if (pos_drive_dist > -1) {
        // Extract the distance
        String stat = message.substring(pos_drive_dist + 1);
        int stat_int = stat.toInt();
        // some error handling
        if (stat_int <= 51 && stat_int >= -51) {
          lcd.print(stat_int);
          move(stat_int, 100);
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
    // Handle Measure command (no parameters)
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
    }
    
    else if (message.indexOf("calibrate") > - 1) {
      Serial.println("Command = calibrate");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = calibrate");

      calibrate();
    }
    else if (message.indexOf("getEEPROM") > - 1) {
      Serial.println("Command = getEEPROM");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("cmd = getEEPROM");

      getEEPROM();
    }
    // Handle invalid or unrecognized commands
    else {
     lcd.setCursor(0, 0);
     lcd.print("Error: unknown cmd");
    }
  }
}

void move(int cm, int speedPercent) {
  // existed pulses + desired travell distance = targer pulses
  // we do not zero pulse counts here becuase we need to keep track on them
  // and zero them only when ESP/JoyStick mode changed
  int targetPulses = (abs(cm) * PULSE_AVG) + pulseCountR;
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















void getEEPROM(){
  uint8_t retrievedValue = EEPROM.read(0);
  double distPerPulse = retrievedValue / 100.0; // Convert back to double

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EEPROM val: ");
  lcd.print(distPerPulse);
}

void calibrate() {
  int initDist = get_dist();
  pulseCountR = 0;
  pulseCountL = 0;
  int finalDist = initDist;  

  // Drive until we have traveled 20cm
  while (abs(initDist - finalDist) < 20) { 
    drive(30, true);
    finalDist = get_dist();
  }
  stopMotors();

  Serial.println("### calibration results ###");
  Serial.print("dist travelled: ");
  Serial.println(abs(initDist - finalDist));
  Serial.print("pulse counts: ");
  Serial.print(pulseCountR);
  Serial.print("|");
  Serial.println(pulseCountL);

  double distPerPulse = 0.0;
  if (pulseCountR > 0) { // Prevent division by zero
    distPerPulse = (double)(abs(initDist - finalDist)) / pulseCountR;
  }

  Serial.print("dist per pulse: ");
  Serial.println(distPerPulse);
  Serial.println("### end of results ###");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("dist: ");
  lcd.print(abs(initDist - finalDist));
  lcd.setCursor(0, 1);
  lcd.print("pls cnt: ");
  lcd.print(pulseCountR);
  lcd.print("|");
  lcd.print(pulseCountL);
  lcd.setCursor(0, 2);
  lcd.print("distPerPulse: ");
  lcd.print(distPerPulse);

  // Store to EEPROM
  uint8_t scaledValue = round(distPerPulse * 100);
  EEPROM.update(0, scaledValue);
  lcd.setCursor(0, 3);
  lcd.print("EEPROM val updated");
}

int get_dist() {
  for (int i = 0; i < LIDAR_SAMPLES; i++) {
    lidar_vals[i] = myLidarLite.distance() - LIDAR_CORR_VAL;
  }

  return avg_lidar_val(lidar_vals, LIDAR_SAMPLES); // Calculate average
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

void drive(int speed, bool direction) {
  int pwmValue = map(speed, 0, 100, 0, 255);
  if (direction) {
    digitalWrite(Motor_L_dir_pin, Motor_forward);
    digitalWrite(Motor_R_dir_pin, Motor_forward);

    analogWrite(Motor_L_pwm_pin, pwmValue);
    analogWrite(Motor_R_pwm_pin, pwmValue);
  }
  else {
    digitalWrite(Motor_L_dir_pin, Motor_return);
    digitalWrite(Motor_R_dir_pin, Motor_return);

    analogWrite(Motor_L_pwm_pin, pwmValue);
    analogWrite(Motor_R_pwm_pin, pwmValue);
  }
}

void stopMotors() {
  analogWrite(Motor_L_pwm_pin, 0);
  analogWrite(Motor_R_pwm_pin, 0);
}