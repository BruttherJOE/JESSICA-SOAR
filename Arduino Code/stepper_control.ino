int ledPin = 13;
int relayOutputPin = 6;  // output signal to control relay
int relayInputPin = 11;  // input signal from Pi to forward to relay
int motorSensorPin = 8;  // input signal from Pi to run motor
int enable = 3;  // motor enable pin
int pulse = 4;  // motor rotation pin

int prev = 0;  

void setup() {
  pinMode(motorSensorPin, INPUT_PULLUP);
  pinMode(relayInputPin, INPUT_PULLUP);
  pinMode(relayOutputPin, OUTPUT);
  pinMode(pulse, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  digitalWrite(enable, HIGH);
}

void moveStepper(int steps){          // move stepper motor by the number of steps
  digitalWrite(enable, LOW);
  delay(10);

  for (int i=0; i<steps; i++) {
    digitalWrite(pulse, HIGH);
    delay(1);
    digitalWrite(pulse, LOW);
    delay(1);
  }

  digitalWrite(enable, HIGH);
}

void loop() {
  // sending HIGH makes the EM off, sending LOW makes the EM on
  digitalWrite(relayOutputPin, !digitalRead(relayInputPin));    // flips NIRYOs commands HIGH/LOW and send LOW/HIGH to relay
                                                                // to switch relay ON/OFF
  
  int piOutput = digitalRead(motorSensorPin);
  digitalWrite(ledPin, piOutput);

  if (piOutput && !prev) moveStepper(200);

  prev = piOutput;
}
