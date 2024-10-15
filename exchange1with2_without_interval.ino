#include <PID_v1.h>
// float P_MAX = 20.0; // maximum pressure pump can generate
// float P_MIN = 0.0; // minimum pressure of pump

const int numReadings = 20;

int count = 0;
double sum = 0.0;

double readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0.0;                // the average

// initialize variables for set points, they will be read from potentometers
double setPoint;

// initialize varibales for PID input, they will use values read from pressure sensor
double input;

// Initialize variables for output of PID control, they will apply to variables potDCn
double output;

// Set PID parameters
double Kp=7, Ki=0, Kd=0;

// set time intervals for working period
int interval_working_period = 10000; //this is the working period
long time_1 = -5000; //interval_working_period + time_1 is the time for first stop
// time period for stop is determined by the delay in stop function

// Create PID instances
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

int prescaler = 256; // set this to match whatever prescaler value you set in CS registers below

// intialize values for the PWM duty cycle set by pots
float potDC1 = 0;
float potDC2 = 0;

void setup() {
  // set up communication parameters
  Serial.begin(9600);

  // turn on all PID controls
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,255); // Set output limit

  // Hardcode setpoint
  setPoint = 10.0; // set pressure = 10.0 psi
  
  //Set up PID control parameters
  myPID.SetTunings(Kp,Ki,Kd);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;}
  
  // input pins for valve switches
  pinMode(50, INPUT);
  pinMode(51, INPUT);

  // output pins for valve PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  
  int eightOnes = 255;  // this is 11111111 in binary
  TCCR3A &= ~eightOnes;   // this operation (AND plus NOT), set the eight bits in TCCR registers to 0 
  TCCR3B &= ~eightOnes;
  TCCR4A &= ~eightOnes;
  TCCR4B &= ~eightOnes;

  // set waveform generation to frequency and phase correct, non-inverting PWM output
  TCCR3A = _BV(COM3A1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS42);
}


void pPWM(float pwmfreq, float pwmDC1, float pwmDC2) {

  // set PWM frequency by adjusting ICR (top of triangle waveform)
  ICR3 = F_CPU / (prescaler * pwmfreq * 2);
  ICR4 = F_CPU / (prescaler * pwmfreq * 2);
  
  // set duty cycles
  OCR3A = (ICR4) * (pwmDC1 * 0.01);
  OCR4A = (ICR4) * (pwmDC2 * 0.01);
}

void loop() {

    // clear settings of previous loop
  potDC1 = 0; potDC2 = 0;
    // read output voltages from sensors and convert to pressure reading in PSI, from
    //in this equation, Pmin=0psi, Pmax=100psi, Vsupply=10V, A8=A8*5/1024 as a result of analogue signal
  float P1 = (analogRead(A8)/1024.0 - 0.1)*100.0/0.8;

    // subtract the last reading:
  total = total - readings[readIndex];
    // read from the sensor:
  readings[readIndex] = P1;
    // add the reading to the total:
  total = total + readings[readIndex];
    // advance to the next position in the array:
  readIndex = readIndex + 1;

    // if we're at the end of the array...
  if (readIndex >= numReadings) {
  // ...wrap around to the beginning:
  readIndex = 0;
  

  // calculate the average:
  average = total / numReadings;

  // Give input after it is smoothed
  input = (double)average; 
    
    
    // Do PID calculation
    myPID.Compute();
    // if statement for manual switch override
    // scale values from pot to 0 to 100, which gets used for duty cycle percentage
    if (digitalRead(51) == HIGH) { potDC2 = analogRead(A2)*100.0/1024.0; }
  
    if (digitalRead(50) == HIGH) 
      { potDC1 = analogRead(A1)*100.0/1024.0; } 
    else
    // Assume pressure depends on duty cycle settings linearly
    // Transform readings from pressure sensor to duty cylce number based on relation between pressure and duty cycles 
    //{ potDC2 = 100; }
    { potDC1 = 28;}
    //{ potDC1 = (float)output/255 * 100.0; }
      
    // Set up PWM prequency from A7 pin
    // float potPWMfq = analogRead(A7)*100.0/1024.0; // scale values from pot to 0 to 100, which gets used for frequency (Hz)
    // potPWMfq = round(potPWMfq/5)*5+1; //1 to 91 Hz in increments of 5 (rounding helps to deal with noisy pot)
    // change the frequency of PWM control
    float potPWMfq = 70;
    
    // update PWM output based on the values calculated from PID control
    pPWM(potPWMfq, potDC1, potDC2);


    if (count <= 50)
    {
      sum = sum + input;
      count ++;
    }

    else
    {
      Serial.print(sum/double(count-1));
      Serial.print("\n");
      count = 1;
      sum = 0.0;
    }
    // print pressure readings
    //Serial.print(output); 
    //Serial.print("\n");
    //Serial.print(input); 
    //Serial.print("\n");
    //Serial.print(OCR4A); 
    //Serial.print("\t");
    //Serial.print(potDC2); 
    //Serial.print("\t");
    //Serial.print(pressure); 
    //Serial.print(input);
    //Serial.print("\n");
    
    delay(2);
  }

}
