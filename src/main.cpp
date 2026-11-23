#include <Arduino.h>

const int buttonPin = 2;

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    bool pressed = digitalRead(buttonPin) == LOW;
    digitalWrite(LED_BUILTIN, pressed ? HIGH : LOW);

    Serial.println(pressed ? 1 : 0);
    delay(20);
}