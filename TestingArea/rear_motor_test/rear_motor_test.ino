#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63
//Initial PWM value
float pwm = 90;
float tOld = 0;
float tNew = 0;
float T = 0;
float speed = 0; //Measured Angular velocity in m/s
float desW = 3; //Desired Angular Velocity in m/s
int count = 0;
int x = 30; // Used in ramp up
//Gain for the proportional controller- goes haywire over 5
//Experimentally determined
float gain = 5;
void setup() {
  Serial.begin(9600);
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);
  while (x < pwm) { //Ramps up speed- Workaround for rear motor safety features
    analogWrite(pwm_rear, x);
    delay(100);
    x = x + 1;
    Serial.print(x);
    Serial.print("\n");
  }
  float voltage = analogRead(63/14.2);
  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.print("\n");
  analogWrite(pwm_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(in_pin), getPeriod, RISING); //Interrupt
}
void loop() {
  
  pwm = (int)(gain * (desW - speed) + pwm); //Actual Controller
  if(pwm>180)
    pwm= 180;
  if(pwm<60)
    pwm=60;
  
  Serial.print(speed);
  Serial.print('\t');
  Serial.print(desW);
  Serial.print('\t');
  Serial.print(micros());
  Serial.print('\t');
  Serial.print(analogRead(63) / 14.2);
  Serial.print('\n');
  analogWrite(pwm_rear, pwm);
}
 void getPeriod() {
    float tOld = tNew;
    tNew = micros();
    double T = (tNew - tOld);
    if ((1.2446)*(1E6) / (28 * T)  < 100) {
     speed = (1.2446)*(1E6) / (28 * T);
    }
}

