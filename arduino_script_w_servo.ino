#include <Wire.h>
#include <math.h>
#include <Servo.h>

Servo myServo;

#define TRIG 3
#define ECHO 2
#define BUTTON 4
#define LED 13
#define SERVO 5

bool SendData = false;
bool buttonState = false;  // Current button state
bool lastButtonState = false;  // Previous button state
unsigned long lastDebounceTime = 0;  // The last time the button was toggled
unsigned long debounceDelay = 50;  // Debounce time in milliseconds

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
long lastTime = millis();
float AngleYaw = 0;

const unsigned long interval = 10;  // Interval between movements (1 second)
int angle = 0;  // Starting angle
int increment = 1;  // Amount to move the servo each interval (1 degree)

unsigned long previousMillis = 0;  // Stores the last time the servo was moved

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.06;
  
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;
  
  // Calculate yaw
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
  lastTime = currentTime;

  // Integrate the gyro rate to get the angle
  AngleYaw += RateYaw * deltaTime;

  // Optional: use complementary filter for better stability (if you have a magnetometer)
  // Here, we're just using gyroscope for simplicity.
}

void setup() {
  Serial.begin(57600);
  pinMode(LED, OUTPUT);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);  // Use internal pull-up resistor
  myServo.attach(SERVO);  // Attach the servo to the pin
  myServo.write(angle);
}



void loop() {
  gyro_signals();
  
  int reading = digitalRead(BUTTON);
  
  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // If the state has been stable for longer than the debounce delay, toggle SendData
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW) {  // Button pressed (assuming active-low configuration)
        SendData = !SendData;
      }
    }
  }

  lastButtonState = reading;

  if (SendData) {
    digitalWrite(13, HIGH);
    float distance = normalizeUSRead();
    
    // Start of the packet
    Serial.print("<");
    
    // Send the yaw angle
    Serial.print("A:");
    Serial.print(AngleYaw);
    Serial.print(",");
    
    // Send the distance
    Serial.print("D:");
    Serial.print(distance);
    
    // End of the packet
    Serial.println(">");
    
    delay(20);
    unsigned long currentMillis = millis();  // Get the current time

    // Check if it's time to move the servo
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;  // Save the last time the servo was moved

      angle += increment;  // Change the angle

      // Reverse the direction at the limits
      if (angle >= 180 || angle <= 0) {
        increment = -increment;
      }
      myServo.write(angle);  // Move the servo to the new angle
      //Serial.print("Servo angle: ");
      Serial.println(angle);
    }
  } else {
    digitalWrite(13, LOW);
  }
}

float distance;
long duration;
float ultraSoundRead() {
  digitalWrite(TRIG, LOW);
  delay(2);
  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 0.034 / 2;
  if (distance > 40) {
    return -1;
  }
  return distance;
}

float normalizeUSRead() {
  int count1 = 0;
  float sum = 0.0;
  int timeDelay = 5;
  int normTime = 100;

  for (int i = 0; i < 200 / normTime; i++) {
    float currVal = ultraSoundRead();
    if (currVal != -1) {
      count1++;
      sum += currVal;
    }
    delay(timeDelay);
  }

  if (count1 < 2) {
    return -1.00;
  }
  return (float)sum / (float)(count1);
}
