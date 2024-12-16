#include <Arduino.h>
#include <BluetoothSerial.h>
#include <SPIFFS.h>

#define QUAD_ENCODER_A 27
#define QUAD_ENCODER_B 26

#define BL_QUAD_ENCODER_A 34
#define BL_QUAD_ENCODER_B 35

#define BR_MOTOR_IN1 32
#define BR_MOTOR_IN2 16

#define BL_MOTOR_IN3 17
#define BL_MOTOR_IN4 18

#define ADC_PIN 13
#define ADC_PIN2 14

#define LIN_ACTUATE 22
#define LIN_DEACTUATE 23

#define TEMP_PIN 33
#define PH_PIN 4
#define MOISTURE_PIN 25
#define ADC_RESOLUTION 4095.0
#define VOLTAGE_REF 3.3
#define ATLAS_COEFF -5.6548
#define ATLAS_CONST 15.509
#define MOISTURE_COEFF 1102033.43
#define MOISTURE_Y 1089774.62

volatile int position = 0; // Encoder position
volatile int bl_position = 0; 
volatile int direction = 0; // 1 for clockwise, -1 for counterclockwise

const int pwmChannel = 0;
const int pwmFrequency = 5000;
const int pwmResolution = 12;
const int pwmChannel_2 = 1;
int targetDistance = 1000;    // Target distance in encoder counts
// int maxSpeed = 3000;
int maxSpeed = 100;
// int accelerationStep = 100;    // Incremental speed step for acceleration and deceleration
int accelerationStep = 10;
int delayPerStep = 10;        // Delay per step during acceleration and deceleration (ms)

int stops_done = 0;

int waypoints = 4;

float alen = 0.75;

uint8_t speed;
uint8_t bl_speed;

float increment;

volatile float coord1;
volatile float coord2;

BluetoothSerial SerialBT;

void IRAM_ATTR updateEncoder() {
  int a = digitalRead(QUAD_ENCODER_A);
  int b = digitalRead(QUAD_ENCODER_B);

  int c = digitalRead(BL_QUAD_ENCODER_A);
  int d = digitalRead(BL_QUAD_ENCODER_B);

  if(a == HIGH && b == LOW){
    position += 1;
  }

  if(c == HIGH && d == LOW){
    bl_position += 1;
  }
}


void setMotor(uint8_t speed, uint8_t bl_speed, uint8_t dir){
  Serial.println(speed);
  Serial.println(bl_speed);
  if(speed == 0){
    ledcWrite(pwmChannel, 0);
    digitalWrite(BR_MOTOR_IN1, LOW);
    digitalWrite(BR_MOTOR_IN2, LOW);
  }
  else{
    ledcWrite(pwmChannel, abs(speed));
    digitalWrite(BR_MOTOR_IN2, HIGH);
    // digitalWrite(BR_MOTOR_IN2, LOW);
  }

  if(bl_speed == 0){
    ledcWrite(pwmChannel_2, 0);
    digitalWrite(BL_MOTOR_IN3, LOW);
    digitalWrite(BL_MOTOR_IN4, LOW);
  }
  else{
    ledcWrite(pwmChannel_2, abs(bl_speed));
    digitalWrite(BL_MOTOR_IN4, HIGH);
    // digitalWrite(BL_MOTOR_IN4, LOW);
  }
}


void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32-WROOM-Receiver", true);
  SerialBT.println("Waiting for Bluetooth data...");

  // Serial.begin(9600);
  pinMode(QUAD_ENCODER_A, INPUT_PULLUP);
  pinMode(QUAD_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_B), updateEncoder, CHANGE);

  pinMode(BL_QUAD_ENCODER_A, INPUT_PULLUP);
  pinMode(BL_QUAD_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BL_QUAD_ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_QUAD_ENCODER_B), updateEncoder, CHANGE);


  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(BR_MOTOR_IN1, pwmChannel);
  pinMode(BR_MOTOR_IN2, OUTPUT);

  ledcSetup(pwmChannel_2, pwmFrequency, pwmResolution);
  ledcAttachPin(BL_MOTOR_IN3, pwmChannel_2);
  pinMode(BL_MOTOR_IN4, OUTPUT);

  setMotor(0,0,0);

  int analogValue = analogRead(ADC_PIN);
  int analogValue2 = analogRead(ADC_PIN2);

  int receivedValue = map(analogValue, 0, 4095, 0, 255);
  int receivedValue2 = map(analogValue2, 0, 4095, 0, 255);

  coord1 = receivedValue/255.0 * 5.0;
  coord2 = receivedValue2/255.0 * 5.0;

  float clen = (coord1+coord2)/2;
  float blen = sqrt(clen*clen-alen*alen);
  increment = blen/waypoints;
  // Serial.println(mid_clen);

  pinMode(LIN_ACTUATE, OUTPUT);
  pinMode(LIN_DEACTUATE, OUTPUT);

  digitalWrite(LIN_ACTUATE, LOW);
  digitalWrite(LIN_DEACTUATE, LOW);

  if(!SPIFFS.begin(true)){
    Serial.println("Failed to initialize SPIFFS");
    return;
  }
  Serial.println("SPIFFS initialized successfully");

  if(SPIFFS.exists("/temperatures.txt")){
    Serial.println("File exists. Preparing data for BLE transmission:");
    File file = SPIFFS.open("/temperatures.txt", FILE_READ);

    if(file){
      SPIFFS.remove("/temperatures.txt");
      Serial.println("Previous temperatures.txt deleted");
    }
    else{
      Serial.println("Failed to open temperatures.txt for reading.");\
    }
  }

  if(SPIFFS.exists("/pH.txt")){
    Serial.println("File exists. Preparing data for BLE transmission:");
    File file = SPIFFS.open("/pH.txt", FILE_READ);

    if(file){
      SPIFFS.remove("/pH.txt");
      Serial.println("Previous pH.txt deleted");
    }
    else{
      Serial.println("Failed to open pH.txt for reading.");
    }
  }

  if(SPIFFS.exists("/moistures.txt")){
    Serial.println("File exists. Preparing data for BLE transmission:");
    File file = SPIFFS.open("/moistures.txt", FILE_READ);

    if(file){
      SPIFFS.remove("/moistures.txt");
      Serial.println("Previous moistures.txt deleted");
    }
    else{
      Serial.println("Failed to open moistures.txt for reading.");
    }
  }
}

void loop() {
  speed = 0;
  bl_speed = 0;
  speed = 0;
  bl_speed = speed;
  setMotor(speed, bl_speed, 1);
  delay(1000);
  int remaining = waypoints - stops_done;
  int height_left = increment*remaining;
  if (coord1+coord2 < 1.8) {

    Serial.println("Printing sensor file contents:");

    File tempFile = SPIFFS.open("/temperatures.txt", FILE_READ);
    if(tempFile){
      while(tempFile.available())
      {
        String tempLine = tempFile.readStringUntil('\n');
        tempLine = tempLine + "\n";
        Serial.println(tempLine);
      }
      tempFile.close();
      Serial.println("End of temperature file contents.");
    }

    File pHFile = SPIFFS.open("/pH.txt", FILE_READ);
    if(pHFile){
      while(pHFile.available())
      {
        String pHLine = pHFile.readStringUntil('\n');
        pHLine = pHLine + "\n";
        Serial.println(pHLine);
      }
      pHFile.close();
      Serial.println("End of pH file contents.");
    }

    File moisturesFile = SPIFFS.open("/moistures.txt", FILE_READ);
    if(moisturesFile){
      while(moisturesFile.available())
      {
        String moisturesLine = moisturesFile.readStringUntil('\n');
        moisturesLine = moisturesLine + "\n";
        Serial.println(moisturesLine);
      }
      moisturesFile.close();
      Serial.println("End of moisture file contents.");
    }

    delay(10000);
  }
  else if (stops_done < waypoints && (coord1+coord2)/2 < sqrt(height_left*height_left+alen*alen) + 0.2) {
    delay(1000);
    digitalWrite(LIN_ACTUATE, HIGH);
    digitalWrite(LIN_DEACTUATE, LOW);
    Serial.println("Linear Actuator Extended");

    int temperature = analogRead(TEMP_PIN);
    int pH = analogRead(PH_PIN);
    float pH_voltage = (pH / ADC_RESOLUTION) * VOLTAGE_REF;
    float pH_actual = (ATLAS_COEFF * pH_voltage) + ATLAS_CONST;
    int moisture = analogRead(MOISTURE_PIN);
    // float moisture_actual = MOISTURE_COEFF * moisture + MOISTURE_Y;

    File tempFile = SPIFFS.open("/temperatures.txt", FILE_APPEND);
    if (tempFile) {
        tempFile.println(String(temperature));
        tempFile.close();
        Serial.println("Temperature saved to SPIFFS");
    } else {
        Serial.println("Failed to open temperatures.txt for writing");
    }

    File pHFile = SPIFFS.open("/pH.txt", FILE_APPEND);
    if (pHFile) {
        pHFile.println(String(pH_actual));
        pHFile.close();
        Serial.println("PH saved to SPIFFS");
    } else {
        Serial.println("Failed to open pH.txt for writing");
    }

    //10ko 0.739
    //100 k0 0.998
    //330 k0 1.69
    //1 m0 2.55
    //2 m0 2.9
    //3 m0 2.9

    File moisturesFile = SPIFFS.open("/moistures.txt", FILE_APPEND);
    if (moisturesFile) {
        moisturesFile.println(String(moisture));
        moisturesFile.close();
        Serial.println("Moisture saved to SPIFFS");
    } else {
        Serial.println("Failed to open moistures.txt for writing");
    }
    
    delay(10000);
    digitalWrite(LIN_ACTUATE, LOW);
    digitalWrite(LIN_DEACTUATE, HIGH);
    Serial.println("Linear Actuator Retracted");
    stops_done++;
  }
  else if (coord2 > coord1 + 0.15) {
    speed = 0;
    bl_speed = 80;
    setMotor(speed, bl_speed, 1);
    Serial.println("Turning left");
    delay(200);
    speed = 80;
    bl_speed = speed;
    setMotor(speed, bl_speed, 1);
    Serial.println("Going straight");
    delay(700);
  }
  else if (coord1 > coord2 + 0.15) {
    bl_speed = 0;
    speed = 80;
    setMotor(speed, bl_speed, 1);
    Serial.println("Turning right");
    delay(200);
    speed = 80;
    bl_speed = speed;
    setMotor(speed, bl_speed, 1);
    Serial.println("Going straight");
    delay(700);
  }
  else {
    speed = 80;
    bl_speed = speed;
    setMotor(speed, bl_speed, 1);
    Serial.println("Going straight");
    delay(700);
  }

  int analogValue = analogRead(ADC_PIN);
  int analogValue2 = analogRead(ADC_PIN2);

  // Convert the analog value to match the DAC range (0-255)
  int receivedValue = map(analogValue, 0, 4095, 0, 255);
  int receivedValue2 = map(analogValue2, 0, 4095, 0, 255);

  coord1 = receivedValue/255.0 * 5.0;
  coord2 = receivedValue2/255.0 * 5.0;

  // Print the received value for debugging
  Serial.println("Received value 1: " + String(coord1));
  Serial.println("Received value 2: " + String(coord2));
}


