#include <PID_v1.h>

#define F_CPU 16000000UL // Define your clock speed
#define PI 3.141592653589793

// Initialize variables for output of PID control
double output;

// Change the frequency of PWM control
float potPWMfq = 70;

int prescaler = 256; // Set this to match whatever prescaler value you set in CS registers below

// Initialize values for the PWM duty cycle set by pots
float potDC1 = 0;
float potDC2 = 0;

// Variable for controlling the period of increasing potDC1
unsigned long increase_period = 1000; // Increase period in milliseconds
unsigned long start_time = 0; // Start time for the increase cycle
bool max_reached = false; // Flag to check if max output is reached

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
  
  if (digitalRead(50) == HIGH) {
    // Use the analog input to set potDC1 manually
    potDC1 = analogRead(A1) * 100.0 / 1024.0;
  } else {
    if (!max_reached) {
      // Calculate the increasing potDC1 value
      unsigned long elapsed_time = current_time - start_time;
      if (elapsed_time > increase_period) {
        start_time = current_time; // Reset start time after one cycle
        elapsed_time = 0; // Ensure potDC1 starts increasing from 20 again
      }
      float progress = (float)elapsed_time / increase_period;
      potDC1 = 20 + progress * 57; // Gradually increase from 20 to 70

      // Check if the maximum output (XX) is reached
      //(reference: 37 3psi, 53 6psi, 60 7.5psi, 70 9psi, 77 12psi)
      if (potDC1 >= 77) {
        potDC1 = 77; // Set to the maximum value
        max_reached = true; // Set the flag so we stop increasing
      }
    }
  }
  
  // Update PWM
  pPWM(potPWMfq, potDC1, potDC2);

  // Small delay to stabilize the loop
  delay(2);
}
