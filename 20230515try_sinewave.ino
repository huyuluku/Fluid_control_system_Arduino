#define PI 3.141592653589793

// change the frequency of PWM control
float potPWMfq = 70;

int prescaler = 256; 

float potDC1;
int t = 0;
unsigned long time_1 = 0;
double theta = 0.0;
unsigned long sine_period = 20000; // 20 seconds in milliseconds

void setup() {
  Serial.begin(9600);
  
  pinMode(50, INPUT);
  pinMode(5, OUTPUT);
  
  int eightOnes = 255;  
  TCCR3A &= ~eightOnes;   
  TCCR3B &= ~eightOnes;

  TCCR3A = _BV(COM3A1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
}

void pPWM(float pwmfreq, float pwmDC1) {
  ICR3 = F_CPU / (prescaler * pwmfreq * 2);
  OCR3A = (ICR3) * (pwmDC1 * 0.01);
}

void loop() {
  potDC1 = 0; 
  if(millis() - time_1 < sine_period)
  {
    t = millis() - time_1;
    theta = double(PI * t / sine_period);
    if (digitalRead(50) == HIGH) 
      { potDC1 = analogRead(A1)*100.0/1024.0; } 
    else
      { potDC1 = 30;}
    pPWM(potPWMfq, sin(theta) * potDC1);    
  }
  else
  {
    time_1 = millis();
    pPWM(potPWMfq, 0);
  }
}
