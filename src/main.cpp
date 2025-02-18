#include <QMC5883LCompass.h>
#include <Arduino.h>
#define BUTTON_PIN 4 // GPIO 4 (D2 on WEMOS D1 R32)

QMC5883LCompass compass;
volatile bool buttonPressed = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 milliseconds debounce delay

// Interrupt Service Routine (ISR)
void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    buttonPressed = !buttonPressed;
    lastDebounceTime = currentTime;
    Serial.println("Interrupt triggered!"); // Print a message when the interrupt is triggered
  }
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Set GPIO 4 as input with internal pull-up resistor
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);
  compass.init();
}

void loop() {
  Serial.println(digitalRead(BUTTON_PIN)); // Print the state of the button
  Serial.println(compass.getAzimuth());
  delay(1000);
}