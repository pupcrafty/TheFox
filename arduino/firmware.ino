/*
 * Arduino Firmware for Raspberry Pi Touch Screen Project
 * 
 * This sketch handles peripheral inputs and communicates with the Raspberry Pi
 * via serial communication.
 */

// Configuration
const int BAUD_RATE = 9600;

// Pin definitions - adjust these based on your hardware
const int BUTTON_PIN = 2;
const int SENSOR_PIN = A0;
const int LED_PIN = 13;

// State variables
bool lastButtonState = LOW;
unsigned long lastSendTime = 0;
unsigned int SEND_INTERVAL = 100; // milliseconds - make modifiable

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
  
  // Initialize pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  
  // Blink LED to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  // Send ready message
  Serial.println("ARDUINO_READY");
  
  lastSendTime = millis();
}

void loop() {
  // Read button state
  bool buttonState = digitalRead(BUTTON_PIN);
  
  // Detect button press (falling edge with pull-up)
  if (buttonState == LOW && lastButtonState == HIGH) {
    Serial.println("BUTTON_PRESSED");
    digitalWrite(LED_PIN, HIGH);
  }
  
  if (buttonState == HIGH && lastButtonState == LOW) {
    digitalWrite(LED_PIN, LOW);
  }
  
  lastButtonState = buttonState;
  
  // Read sensor and send data periodically
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    int sensorValue = analogRead(SENSOR_PIN);
    
    // Send sensor data in format: SENSOR:value
    Serial.print("SENSOR:");
    Serial.println(sensorValue);
    
    lastSendTime = currentTime;
  }
  
  // Check for incoming commands from Raspberry Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Handle commands
    if (command == "LED_ON") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED_ON");
    } else if (command == "LED_OFF") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED_OFF");
    } else if (command.startsWith("SET_INTERVAL:")) {
      // Example: SET_INTERVAL:200
      int interval = command.substring(13).toInt();
      if (interval > 0 && interval < 10000) {
        SEND_INTERVAL = interval;
        Serial.print("INTERVAL_SET:");
        Serial.println(interval);
      }
    }
  }
  
  delay(10); // Small delay to prevent excessive CPU usage
}

