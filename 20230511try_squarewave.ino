#include <PID_v1.h>

// Initialize variables for output of PID control, they will apply to variables potDCn
double output;

// Change the frequency of PWM control
float potPWMfq = 70;

int prescaler = 256; // Set this to match whatever prescaler value you set in CS registers below

// Initialize values for the PWM duty cycle set by pots
float potDC1 = 0;
float potDC2 = 0;

// Set time intervals for working period and rest period
int working_period = 250; // Working period in milliseconds
int rest_period = 250; // Rest period in milliseconds
long time_1 = 0; // Initial time

void setup() {
  // Set up communication parameters
  Serial.begin(9600);
  
  // Input pins for valve switches
  pinMode(50, INPUT);
  pinMode(51, INPUT);

  // Output pins for valve PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  
  int eightOnes = 255;  // This is 11111111 in binary
  TCCR3A &= ~eightOnes;   // This operation (AND plus NOT), set the eight bits in TCCR registers to 0 
  TCCR3B &= ~eightOnes;
  TCCR4A &= ~eightOnes;
  TCCR4B &= ~eightOnes;

  // Set waveform generation to frequency and phase correct, non-inverting PWM output
  TCCR3A = _BV(COM3A1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS42);
}

void pPWM(float pwmfreq, float pwmDC1, float pwmDC2) {
  // Set PWM frequency by adjusting ICR (top of triangle waveform)
  ICR3 = F_CPU / (prescaler * pwmfreq * 2);
  ICR4 = F_CPU / (prescaler * pwmfreq * 2);
  
  // Set duty cycles
  OCR3A = ICR3 * (pwmDC1 * 0.01);
  OCR4A = ICR4 * (pwmDC2 * 0.01);
}

void loop() {
  long current_time = millis();

  if (current_time >= time_1 + working_period + rest_period) {
    time_1 = current_time;
  }

  if (current_time < time_1 + working_period) {
    potDC1 = 0; 
    potDC2 = 0;
    if (digitalRead(50) == HIGH) {
      potDC1 = analogRead(A1) * 100.0 / 1024.0;
    } else {
      potDC1 = 30;
    }
    pPWM(potPWMfq, potDC1, potDC2);
  } else {
    OCR3A = 0;
  }
  
  delay(2);
}
