
#include <Arduino.h>

void setup() {
    // initialize the digital pin as an output.
    // Pin 13 has an LED connected on most Arduino boards:
    pinMode(13, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    digitalWrite(13, HIGH);   // set the LED on
    delay(500);               // wait for a second
    digitalWrite(13, LOW);    // set the LED off
    delay(2000);              // wait for a second

    int c;

    while ((c = Serial.read()) != -1)
        Serial.write(c);
}

