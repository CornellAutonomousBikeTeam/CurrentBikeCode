#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
int count = 0; //used to determine if we are running at 5hz
int prev_millis = 0;
int curr_millis = 0;
int bits_so_far = 0;
ros::NodeHandle  nh;

//Array containing bike state variables
std_msgs::Float32MultiArray bike_state;

//Array containing gps state variables
std_msgs::Float32MultiArray gps_state;

//Publisher object for the bike state
ros::Publisher state_pub("bike_state", &bike_state);

//Publisher object for the gps state
ros::Publisher gps_pub("gps", &gps_state);

float nav_instr = 0;
void updateInstruction(const std_msgs::Float32& data) {
  nav_instr = data.data;
}
//Ros listener
ros::Subscriber<std_msgs::Float32> nav_sub("nav_instr", &updateInstruction);

float temp = 0;

#include "TinyGPS++.h"
TinyGPSPlus gps;

#include "IMU.h"
#include "PID.h"
#include "RearMotor.h"
#include <math.h>


/*Define definite variables*/
//Front Motor
#define PWM_front 9
#define DIR 46
int steer_dir = 0;

//Rear Motor
#define PWM_rear 8  //rear motor PWM pin
#define hall_pin 11 //hall sensor pulse 
#define reverse_pin 50 //to change direction of rear motor

//Rear Motor Variables
float rear_pwm = 0; //current pwm value
double speed = 0; //speed in rev/s
boolean forward = true; //if False, is running in reverse
//Variables for calculating rear motor speed
float tOld = 0; //first time reading
float tNew = 0; //second time reading
double T = 0;

//Rear motor controller variable
float gain_p = 5;
float desired_speed = 3; //(m/s)

//Timed Loop Variables
const long interval = 10000;
long l_start;
long l_diff;

//Balance Control constants
const int k1 = 71; //phi = lean
const int k2 = 10; //was previously 21 //phidot=lean rate
const int k3 = -20; //delta=steer

//Encoder
const int quad_A = 2;
const int quad_B = 13;
const int idx = 60;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);
const unsigned int mask_idx = digitalPinToBitMask(idx);
int REnot = 3;
int DE = 4;
signed int oldPosition  = 0;
signed int oldIndex = 0;
unsigned long previous_t = 0;
signed int x_offset = 0;
float desired_pos = 0;
float current_pos = 0;
float current_vel = 0;
float desired_vel = 0;
float vel_error = 0;
float pos_error = 0;
float PID_output = 0;
float sp_error = 0;
float sv_error = 0;
int pwm = 0;

//count the number of times the time step has been calculated to calculate a running average time step
int numTimeSteps = 0;
float averageTimeStep = 0;
int n = 0;

float desired_steer = 0;
float desired_pos_array[250];
float theo_position = 0;

//Watchdog
#define WDI 42
#define EN 41

//Landing Gear
#define relay1 48
#define relay2 47
float front_steer_value ;
float back_wheel_speed ;
float steer_contribution ;
float commanded_speed ;

//#define front_steer_value 51
//#define back_wheel_speed 28

#define relay3 50
#define relay4 49

//RC
#define RC_CH1 51     //Steer Angle 
#define RC_CH2 28     //
#define RC_CH3 25     //Velocity 
#define RC_CH4 33     //
#define RC_CH5 27     //Kill Switch 
#define RC_CH6 32     //Landing Gear 

//timers for each channel
int duration_CH1, duration_CH2, duration_CH3, duration_CH4, duration_CH5, duration_CH6;
int start_CH1, start_CH2, start_CH3, start_CH4, start_CH5, start_CH6;
int end_CH1, end_CH2, end_CH3, end_CH4, end_CH5, end_CH6;
//current cycle's logic
boolean CH1, CH2, CH3, CH4, CH5, CH6;

//RC variables
float desired_angle;  //CH1
int PWM_rear_output;  //CH3


//voltage constants and variables
const int VOLTAGE_PIN = 63; //A9
float VOLTAGE_CONST = 14.2;
float battery_voltage = 0;
float VELOCITY_VOLTAGE_K = 1.7936;
float VELOCITY_VOLTAGE_C = -1.2002;

//define maximum front wheel pwm
int maxfront_PWM = 110;

//Read the relative position of the encoder
signed int relativePos = REG_TC0_CV0;
//Read the index value (Z channel) of the encoder
signed int indexValue = REG_TC0_CV1;


//set up timer for rc communication for steer and back motor speed
volatile unsigned long timer_start;  //micros when the pin goes HIGH
volatile unsigned long timer_start2;  //micros when the pin goes HIGH
volatile unsigned long timer_start6;  //micros when the pin goes HIGH
volatile int last_interrupt_time; //calcSignal is the interrupt handler
volatile int last_interrupt_time2; //calcSignal is the interrupt handler
volatile int last_interrupt_time6; //calcSignal is the interrupt handler
volatile float steer_range ;
volatile float foreward_speed ;
volatile float pulse_time ;
volatile float pulse_time2 ;
volatile float pulse_time6 ;

//difference between timer_start and micros() is the length of time that the pin
//was HIGH - the PWM pulse length. volatile int pulse_time;
//this is the time that the last interrupt occurred.
//you can use this to determine if your receiver has a signal or not.

void calcSignal()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH1) == HIGH)
  {
    timer_start = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start != 0 && ((volatile int)micros() - timer_start > 1100) && ((volatile int)micros() - timer_start < 1900) )
    {
      //record the pulse time
      pulse_time = ((volatile int)micros() - timer_start); //pulse time is the output from the rc value that we need to transform into a pwm value
      //restart the timer
      timer_start = 0;
    }
  }
}
void calcSignal2()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time2 = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH2) == HIGH)
  {
    timer_start2 = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start2 != 0 && ((volatile int)micros() - timer_start2 > 1100) && ((volatile int)micros() - timer_start2 < 1900) )
    {
      //record the pulse time
      pulse_time2 = ((volatile int)micros() - timer_start2); //pulse time is the output from the rc value that we need to transform into a pwm value
      //restart the timer
      timer_start2 = 0;
    }
  }
}

void calcSignal6()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time6 = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH6) == HIGH)
  {
    timer_start6 = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start6 != 0 && ((volatile int)micros() - timer_start6 > 1000) && ((volatile int)micros() - timer_start6 < 2000) )
    {
      //record the pulse time
      pulse_time6 = ((volatile int)micros() - timer_start6); //pulse time is the output from the rc value that we need to transform into a pwm value
      //restart the timer
      timer_start6 = 0;
    }
  }
}

//  method for sending hex messages to the gps
void sendUBX(byte *UBXmsg, byte msgLength) {
  for (int i = 0; i < msgLength; i++) {
    Serial3.write(UBXmsg[i]);
  }
}
void getPeriod() {
  float tOld = tNew;
  tNew = micros();
  double T = (tNew - tOld);
  if ((1.2446) * (1E6) / (28 * T) < 100) {
    speed = (1.2446) * (1E6) / (28 * T) ;

  }
}
//this is all normal arduino stuff
void setup()
{

  nh.initNode();
  int data_size = 10;
  int gps_state_data_size = 12;
  nh.subscribe(nav_sub);
  bike_state.data_length = data_size;
  gps_state.data_length = gps_state_data_size;
  bike_state.layout.data_offset = 0;
  gps_state.layout.data_offset = 0;
  //Allocate memory for the bike_state layout
  bike_state.layout.dim = (std_msgs::MultiArrayDimension *)
                          malloc(sizeof(std_msgs::MultiArrayDimension) * data_size);
  //Allocate memory for the gps_state layout
  gps_state.layout.dim = (std_msgs::MultiArrayDimension *)
                         malloc(sizeof(std_msgs::MultiArrayDimension) * data_size);
  bike_state.layout.dim[0].label = "desired_vel";
  bike_state.layout.dim[1].label = "current_vel";
  bike_state.layout.dim[2].label = "imu_rate";
  bike_state.layout.dim[3].label = "imu_angle";
  bike_state.layout.dim[4].label = "encoder_position";
  bike_state.layout.dim[5].label = "desired_steer";
  //Allocate memory for the bike_state data
  bike_state.data = (float *)malloc(sizeof(float) * data_size);
  gps_state.data = (float *)malloc(sizeof(float) * gps_state_data_size);


  nh.advertise(state_pub);
  nh.advertise(gps_pub);


  timer_start = 0;
  timer_start2 = 0;
  timer_start6 = 0;
  attachInterrupt(RC_CH1, calcSignal, CHANGE);
  attachInterrupt(RC_CH2, calcSignal2, CHANGE);
  attachInterrupt(RC_CH6, calcSignal6, CHANGE);

  Serial.begin(57600); //Set to the same rate as ROS for correct Serial connections
  //GPS
  // GPS baudrate (gps hardware runs natively at 9600)
  Serial3.begin(19200); //This was originally 9600 - test with higher baud rates

  //hex messages that config gps data rate - found hex encodings via u-center
  byte gpsmsg10[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; //10hz
  byte gpsmsg5[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A}; //5hz
  byte gpsmsg4[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96}; //3.3hz
  byte gpsmsg33[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x2C, 0x01, 0x01, 0x00, 0x01, 0x00, 0x43, 0xC7}; //3.3hz
  byte gpsmsg1[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0E8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39}; //1hz

  byte gpsmsgpoll[] = {0xB5, 0x62, 0x06, 0x00};
  byte gpsmsgbaud[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xDF };
  byte gpsmsgbaud2[] = {0xB5, 0x62, 0x06, 0x41, 0x09, 0x00, 0x01, 0x01, 0x30, 0x81, 0x00, 0x00, 0x00, 0x00, 0xFB, 0xFE, 0x1F};
  byte gpsmsgsaveconf[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB};
  //Configures gps to 5hz update rate; change gpsmsg5 to a different byte encoding for different configurations
  sendUBX(gpsmsgpoll, sizeof(gpsmsgpoll));
  sendUBX(gpsmsgbaud, sizeof(gpsmsgbaud));
  sendUBX(gpsmsg5, sizeof(gpsmsg5));
  //sendUBX(gpsmsgsaveconf, sizeof(gpsmsgsaveconf));
  initIMU();
  //setup rc
  //  pinMode(front_steer_value, INPUT);
  //  pinMode(back_wheel_speed, INPUT);

  //setup Encoder
  pinMode(REnot, OUTPUT);
  pinMode(DE, OUTPUT);
  // activate peripheral functions for quad pins
  REG_PIOB_PDR = mask_quad_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_A;   // choose peripheral option B
  REG_PIOB_PDR = mask_quad_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_B;   // choose peripheral option B
  REG_PIOB_PDR = mask_idx;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_idx;   // choose peripheral option B


  // activate clock for TC0 and TC1
  REG_PMC_PCER0 = (1 << 27) | (1 << 28) | (1 << 29);

  // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5;


  // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1 << 9) | (1 << 8) | (1 << 12);


  // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)
  REG_TC0_QIER = 1;


  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;

  //setup Motor Outputs
  pinMode(DIR, OUTPUT);
  pinMode (PWM_front, OUTPUT);
  pinMode (PWM_rear, OUTPUT);

  //setup Watchdog
  pinMode(WDI, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  //setup Landing Gear
  //pinMode(relay1, OUTPUT);
  //pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);

  //setup RC
  //pinMode(RC_CH1, INPUT);
  //pinMode(RC_CH2, INPUT);
  pinMode(RC_CH3, INPUT);
  pinMode(RC_CH4, INPUT);
  pinMode(RC_CH5, INPUT);
  //pinMode(RC_CH6, INPUT);



  //setup Rear Motor
  pinMode(hall_pin, INPUT);
  //pinMode(PWM_rear, OUTPUT);
  pinMode(reverse_pin, OUTPUT);
  digitalWrite(reverse_pin, HIGH); //when high the motor goes forward
  float voltage = analogRead(63) / 14.2 * pwm / 180;
  analogWrite(PWM_rear, pwm);
  attachInterrupt(digitalPinToInterrupt(hall_pin), getPeriod, RISING); //Interrupt


  // initializes pins for the landing gear relay switch
  pinMode(48, OUTPUT);
  pinMode(47, OUTPUT);



  // tell user to press any key to calibrate wheel
  //  int  message_delivered = 0;
  //   while (!(Serial.available())) {
  //    if (message_delivered == 0) {
  //      Serial.println("Calibrate the front wheel") ;
  //      message_delivered = 1;
  //    }
  //   }
  //
  //  the follwing loop will not terminate until wheel passes front tick on encoder twice. The second time should be passed very slowly-
  //  this will allow for the most accurate location to be found for the center alignment of the front wheel with the bike.

  signed int y = REG_TC0_CV1;
  oldIndex = y;
  digitalWrite(DIR, HIGH);

  /*
    while(y==oldIndex){
    analogWrite(PWM_front,20);
    y = REG_TC0_CV1;
    //Serial.println("Ticking");
    }*/

  //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  x_offset = REG_TC0_CV0;
  analogWrite(PWM_front, 0);
  //rear motor initialization

  int pwm = 60;
  int olav = 0;
  //  while (olav < pwm){ //Ramps up speed- Workaround for rear motor safety features
  //    analogWrite(PWM_rear, olav);
  //    delay(100);
  //    olav=olav+10;
  //Serial.println("tteeeeeettt");
  //  }
  //ramp up rear motor to 60 pwm
  rampToPWM(170, 0);

}


//landing gear functions
void landingGearDown() {
  digitalWrite(48, LOW); //sets relay pin 1 to High (turns light on)
  digitalWrite(47, LOW); //sets relay pin 2 to High  (turns light on)
}
void landingGearUp() {
  digitalWrite(48, HIGH); //Sets relay pin 1 to low (turns light off)
  digitalWrite(47, HIGH); //Sets relay pin 2 to low (turns light off)
}

/* takes in desired angular velocity returns pwm */
int velocityToPWM (float desiredVelocity) {
  battery_voltage = analogRead(VOLTAGE_PIN);
  //Serial.println("pin 63 output " + String(battery_voltage));
  battery_voltage = battery_voltage / VOLTAGE_CONST;

  //Serial.println("voltage is " + String(battery_voltage));
  pwm = 256 * (desiredVelocity - VELOCITY_VOLTAGE_C) / (battery_voltage * VELOCITY_VOLTAGE_K);
  //Serial.println("pwm is  " + String(pwm));

  if (desiredVelocity > 18 ) { //***TO DO*** THIS LIMITATION MUST GO ON ALL OF THE PWM GOING TO THE FRONT MOTOR, NOT JUST THE FEED FORWARD LOOP
    //put in the warning
    return maxfront_PWM;
  } else {
    return pwm;
  }
}


/* intakes commanded velocity from balance controller
   converts commanded velocity into commanded position */
float eulerIntegrate(float desiredVelocity, float current_pos) {
  float desiredPosition = current_pos + desiredVelocity * ((float)interval / 1000000.0) ;
  return desiredPosition;
}


// updates global variables representing encoder position
float updateEncoderPosition() {
  //Read the relative position of the encoder
  relativePos = REG_TC0_CV0;
  //Read the index value (Z channel) of the encoder
  indexValue = REG_TC0_CV1;
  current_pos = (((relativePos - x_offset) * 0.02197 * M_PI) / 180); //Angle (rad)
  return current_pos;
}

/* takes in desired position and applies a PID controller to minimize error between current position and desired position */
float frontWheelControl(float desiredVelocity, float current_pos) { //steer contribution doese not need to be passed into
  //frontWheelControl because it is a global variable

  unsigned long current_t = micros();

  //  if (n == 0) {
  //    float desired_pos = 0;
  //    PID_Controller(desired_pos, relativePos, x_offset, current_t, previous_t, oldPosition);
  //    n++;
  //  }
  float desired_pos = eulerIntegrate(desiredVelocity, current_pos);
  //Serial.println(String(theo_position) + '\t' + String(desired_pos) + '\t' + String(current_pos)) ;

  /*
    if (Serial.available()){
    desired_pos = M_PI / 180 * Serial.parseFloat();
    }
  */



  //Serial.println(String(steer_contribution) + '\t' +  String(commanded_speed));

  float current_vel = PID_Controller(desired_pos, relativePos, x_offset, current_t, previous_t, oldPosition);

  previous_t = current_t;
  oldPosition = relativePos - x_offset;
}

/* FUNCTION THAT RETURNS DESIRED ANGULAR VELOCITY OF FRONT WHEEL */
float balanceController(float roll_angle, float roll_rate, float encoder_angle) {
  float desiredSteerRate = (k1 * roll_angle) + (k2 * roll_rate) + k3 * (encoder_angle - desired_steer);
  if (desiredSteerRate > 10) {
    desiredSteerRate = 10;
  }
  else if (desiredSteerRate < -10) {
    desiredSteerRate = -10;
  }
  return desiredSteerRate;
}

struct roll_t {
  float rate;
  float angle;
};

// Retrieve data from IMU about roll angle and rate and return it
struct roll_t updateIMUData() {
  roll_t roll_data;
  //get data from IMU
  float roll_angle = getIMU(0x01);   //get roll angle
  float roll_rate = getIMU(0x26);    //get roll rate
  roll_data.angle = roll_angle;
  roll_data.rate = roll_rate;
  return roll_data;
}
//Loop variables
void loop() {
  Serial.print("entered");
  numTimeSteps++;
  //Rear motor controller with RC to switch between controller and RC inputs
  if (pulse_time6 > 1700 && pulse_time6 < 2100) {
    foreward_speed = map(pulse_time2, 1100, 1900, 0, 200);
  }
  else {
    rear_pwm = (int)(gain_p * (desired_speed - speed) + rear_pwm); //Actual Controller
    if (rear_pwm > 180) {
      rear_pwm = 180;
    }
    if (rear_pwm < 60) {
      rear_pwm = 60;
    }
    foreward_speed = rear_pwm;
  }

  analogWrite(PWM_rear, foreward_speed);

  //RC controls front wheel
  steer_range = map(pulse_time, 1100, 1900, -70, 70);
  desired_steer = steer_range * .01 ;

  //Nav controls front wheel
  //desired_steer = nav_instr;

  l_start = micros();
  float encoder_position = updateEncoderPosition(); //output is current position wrt front zero

  roll_t imu_data = updateIMUData();
  float desiredVelocity = balanceController(((1) * (imu_data.angle)), (1) * imu_data.rate, encoder_position); //*****PUT IN OFFSET VALUE BECAUSE THE IMU IS READING AN ANGLE OFF BY +.16 RADIANS

  float current_vel = frontWheelControl((-1) * desiredVelocity, encoder_position); //DESIRED VELOCITY SET TO NEGATIVE TO MATCH SIGN CONVENTION BETWEEN BALANCE CONTROLLER AND


  //Do not change indexes since nav algorithim hard codes some of these
  bike_state.data[0] = current_vel; //front motor (rad/s)
  bike_state.data[1] = desiredVelocity; //front motor (rad/s)
  bike_state.data[2] = encoder_position; //front motor (rad) (delta)
  bike_state.data[3] = desired_steer; //front motor (rad)
  bike_state.data[4] = imu_data.rate; //imu (rad) (phi-dot)
  bike_state.data[5] = imu_data.angle; //imu (rad/s) (phi)
  bike_state.data[6] = speed; //rear motor (m/s) (based on hall sensor)
  bike_state.data[7] = foreward_speed; //rear motor commanded speed (pwm)
  bike_state.data[8] = battery_voltage;
    Serial.print("CHECKPOINT2");

  //gps data (Don't change these indexes either)
  while (Serial3.available()) {
    //Serial.print((char)Serial3.read());
    gps.encode(Serial3.read());
    bits_so_far = bits_so_far + 8;
    Serial3.flush(); //WITHOUT THIS THE BUFFER WILL NOT BE ABLE TO BE READ AS FAST AS IT IS WRITTEN TO AND THIS WILL LOOP FOREVER
    //Serial.println(gps.location.isUpdated());
  }
  if (gps.time.isUpdated()) {
    prev_millis = curr_millis;
    curr_millis = millis();
    if (curr_millis - prev_millis > 70)
    {
      Serial.print("AGE: "); Serial.println(curr_millis - prev_millis);
      Serial.print("Latitude                      "); Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude                     "); Serial.println(gps.location.lng(), 6);
      Serial.print("Age                     ");       Serial.println(gps.location.age(), 6);
      Serial.print("Hours                         "); Serial.println(gps.time.hour()); // Hour (0-23)
      Serial.print("Minutes                       "); Serial.println(gps.time.minute()); // Minute (0-59)
      Serial.print("Seconds                       "); Serial.println(gps.time.second()); // Second (0-59)
      Serial.print("Centiseconds                  "); Serial.println(gps.time.centisecond()); // 100ths of a second (0-99)
      Serial.print("Date                          "); Serial.println(gps.date.value()); // Raw date in DDMMYY format
      Serial.print("Number of Satelites in use    "); Serial.println(gps.satellites.value()); // Number of satellites in use
      Serial.print("Course in degrees             "); Serial.println(gps.course.deg());
      Serial.print("Bits so far             "); Serial.println(bits_so_far);

    }
    //}
    //Serial.print("Valid remaining data: "); Serial.println(gps.charsProcessed());
    //Serial.print("Sentences that failed checksum="); Serial.println(gps.failedChecksum());
    // Testing overflow in SoftwareSerial is sometimes useful too.
    //Serial.print("Soft Serial device overflowed? "); Serial.println(Serial3.overflow() ? "YES!" : "No");
    //Serial.print("Serial3 read in"); Serial.println(Serial.read());
  }
  gps_state.data[0] = gps.location.lat(); //latitude (deg)
  gps_state.data[1] = gps.location.lng(); //longitude (deg)
  gps_state.data[2] = gps.time.hour(); // Hour (0-23)
  gps_state.data[3] = gps.time.minute(); // Minute (0-59)
  gps_state.data[4] = gps.time.second(); // Second (0-59)
  gps_state.data[5] = gps.location.age(); //how old location data is in ms
  gps_state.data[6] = gps.satellites.value(); // Number of satellites in use
  gps_state.data[7] = gps.course.deg(); //yaw relative to lat,long (deg)
  gps_state.data[8] = gps.speed.mps(); //speed from gps (m/s)
  gps_state.data[9] = gps.hdop.value(); // Horizontal Dim. of Precision (100ths-i32)
  gps_state.data[10] = curr_millis - prev_millis; //Time since last update
  gps_state.data[11] = bits_so_far; //Time since last update
  bits_so_far = 0;
  //gps_state.data[5] = (gps.time.centisecond()); // 100ths of a second (0-99)
  Serial.print("CHECKPOINT3");
  //Publish address of bike,gps state objects for ROS
  gps_pub.publish( &gps_state );
  state_pub.publish( &bike_state );
  nh.spinOnce();
  //    delay(1);

  l_diff = micros() - l_start;
  //Standardize the loop time by checking if it is currently less than the constant interval, if it is, add the differnce so the total loop time is standard
  if (l_diff < interval) {
    delayMicroseconds(interval - l_diff);
  } else {
    //      Serial.println("LOOP LENGTH WAS VIOLATED. LOOP TIME WAS: " + String(l_diff));
    //      while(true){}
  }
  //}
  /*
    Method that sets value "speed" to current speed in m/s
  */


}



//}

