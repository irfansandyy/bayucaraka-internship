#include <Arduino.h>

const int pirPin = 2;
const int ledPin = 3;

const int dataPin = 4;
const int clockPin = 5;
const int latchPin = 6;

const byte digits[] = {
  0b11000000,
  0b11111001,
  0b10100100,
  0b10110000,
  0b10011001,
  0b10010010,
  0b10000010,
  0b11111000,
  0b10000000,
  0b10010000 
};

volatile int count = 0;
bool lastMotionState = false;

void updateDisplay(int number) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, digits[number]);
  digitalWrite(latchPin, HIGH);
}

void setup() {
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  
  updateDisplay(0);
}

void loop() {
  bool motionDetected = digitalRead(pirPin);
  
  if(motionDetected && !lastMotionState) {
    digitalWrite(ledPin, HIGH);
    count = (count + 1) % 10;
    updateDisplay(count);
    delay(100);
  } else if(!motionDetected) {
    digitalWrite(ledPin, LOW);
  }
  
  lastMotionState = motionDetected;
}