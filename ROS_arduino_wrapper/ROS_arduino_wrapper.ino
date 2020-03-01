// Include header files for the support libraries
#include "BikeROS.h"
#include "TinyGPS++.h"
#include "IMU.h"
#include "FrontWheel.h"
#include "RC.h"
#include "RearMotor.h"
#include "Encoder.h"
#include "LandingGear.h"
#include "LightSensor.h"
#include <math.h>

// The GPS object
TinyGPSPlus gps;

int count = 0; //used to determine if we are running at 5hz
int prev_millis = 0; // holds the timestamp (in MILLIseconds) when the loop function was last called
int curr_millis = 0; //Used to help calculate time since last update
int total_millis = 0; //Holds total time passed so we know when to update Hz
int bits_so_far = 0; //Unclear what this is used for atm

// Timed Loop Variables
// holds the timestamp (in microseconds) when the loop() function started
long l_start;

// Cheap GPS support
// Buffer for storing NMEA sentences received from the GPS until they are published to the ROS topic /nmea
char nmea_buffer[200];
// Used to keep track of how long the NMEA sentence that we're reading right now is (because we might receive part of the sentence in one loop() call)
int nmea_idx = 0;

//Pins for the LEDs on bike
#define LED_1 22 // red    (as of Feb 2020, turned off when we exit setup())
#define LED_2 35 // yellow (as of Feb 2020, unused)
#define LED_3 36 // blue   (as of Feb 2020, toggled whenever loop() finishes once)

//Pins for the ligh sensor
#define S0 16
#define S1 49
#define S2 50
#define S3 16
#define OE 47  //output enable
#define sensorOut 48

//Voltage constants and variables
// One of the 2015-2016 reports has a derivation of these constants; the constants can be used to calculate the battery voltage
const int VOLTAGE_PIN = 63; //A9 - this pin is connected to a voltage sensor on the Arduino, in case we want to use it (feb 2020)
float VOLTAGE_CONST = 14.2;
float battery_voltage = 0;
float VELOCITY_VOLTAGE_K = 1.7936;
float VELOCITY_VOLTAGE_C = -1.2002;

//Method for sending hex messages to the cheap GPS
void sendUBX(byte *UBXmsg, byte msgLength) {
  for (int i = 0; i < msgLength; i++) {
    Serial3.write(UBXmsg[i]);
  }
}

/*Method to set the steer and speed based on either rc or nav instructors depending on mode set by remote*/
void navOrRC() {
  if (nav_mode) {
    desired_steer = nav_instr; // Get desired steer from the nav instructions (read from ROS topic /nav_instr)
    desired_lean = (desired_speed * desired_speed / 10.0) * desired_steer; //phi_d = (v^2/(l * g)) * delta_d [for more explanation of this formula see the end of Dylan Meehan's 2017 masterpiece "introduction ..." available in google drive
    
    // run a simple P controller to see if we should increase or decrease the speed of the rear wheel
    rear_pwm = (int)(gain_p * (desired_speed - speed) + rear_pwm); //Actual Controller

    // TODO clarify what 180 and 60 are in terms of meters per second
    // make sure that the rear PWM value is within safety margins
    if (rear_pwm > 180) {
      rear_pwm = 180; //Max PWM value that bike can do, regardless of what nav algo wants to say
    }
    if (rear_pwm < 60) {
      rear_pwm = 60; //Min PWM value that bike can do, regardless of what nav algo wants to say
    }

    // set the forward velocity of the bike (that comes from the rear wheel) to the PWM value for the rear wheel that we just calculated.
    forward_speed = rear_pwm;
    SerialUSB.println("Nav mode");
  } else {
    forward_speed = map(pulse_time2, 1100, 1900, 0, 200);
    setLandingGear(forward_speed>70);
    steer_range = map(pulse_time, 1100, 1900, 60, -60);
    desired_steer = steer_range * .01 ;
    desired_lean = (desired_speed * desired_speed / 10.0) * desired_steer; //phi_d = (v^2/l/g) * delta_d
    // desired_lean = (speed*speed)*desired_steer; //phi_d = (v^2/l/g) * delta_d
    // desired_lean = (gps_state.data[8]*gps_state.data[8]/10.0)*desired_steer; //phi_d = (v^2/l/g) * delta_d
    SerialUSB.println("RC mode");
  }

}

void setup() {
  initROS();

  timer_start = 0;
  timer_start2 = 0;
  timer_start5 = 0;
  timer_start6 = 0;

  attachInterrupt(RC_CH1, calcSignal, CHANGE);
  attachInterrupt(RC_CH2, calcSignal2, CHANGE);
  attachInterrupt(RC_CH5, calcSignal5, CHANGE);
  attachInterrupt(RC_CH6, calcSignal6, CHANGE);

  SerialUSB.begin(115200);
  Serial.begin(115200); //Set to the same rate as ROS for correct Serial connections

  //GPS

  Serial3.begin(9600); //GPS baudrate (gps hardware runs natively at 9600)
  byte gpsmsg10[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; //10hz
  byte gpsmsgbaud57600[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0x51}; //UBX and NMEA
  sendUBX(gpsmsg10, sizeof(gpsmsg10));

  initIMU();

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

  REG_PMC_PCER0 = (1 << 27) | (1 << 28) | (1 << 29); // activate clock for TC0 and TC1
  REG_TC0_CMR0 = 5; // select XC0 as clock source and set capture mode
  REG_TC0_BMR = (1 << 9) | (1 << 8) | (1 << 12); // activate quadrature encoder and position measure mode, no filters
  REG_TC0_QIER = 1; // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)

  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;

  //setup Motor Outputs
  pinMode(DIR, OUTPUT); //Direction of front wheel's rotation
  pinMode (PWM_front, OUTPUT);
  pinMode (PWM_rear, OUTPUT);

  initLandingGear();

  //setup RC
  //pinMode(RC_CH1, INPUT);
  //pinMode(RC_CH2, INPUT);
  pinMode(RC_CH3, INPUT);
  pinMode(RC_CH4, INPUT);
  pinMode(RC_CH5, INPUT);
  //pinMode(RC_CH6, INPUT);

  // setup LED pins
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

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


  signed int y = REG_TC0_CV1;
  oldIndex = y;
  digitalWrite(DIR, HIGH);


  //Front wheel calibration loop
  while (y == oldIndex) { 
    analogWrite(PWM_front, 40);
    y = REG_TC0_CV1;
    //Serial.println("Ticking");
    //Serial.println((String(REG_TC0_CV0) + '\t' + y).c_str());
  }
  

  x_offset = REG_TC0_CV0;   //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  analogWrite(PWM_front, 0);

  //rear motor initialization
  //int pwm = 60;
  //rampToPWM(170, 0);

  digitalWrite(LED_1, HIGH); //LED to signal setup function done

  nav_mode = true; //Variable that tells whether bike is in nav mode or not

  //light sensor setup
  // Setting the outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);

  // Setting frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

    // Setting unfiltered photodiodes to be read
   digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  digitalWrite(OE, LOW);
}

//Loop variables
int blinkState = HIGH;
void loop() {

  navOrRC();

  analogWrite(PWM_rear, forward_speed);

  l_start = micros();

  float encoder_position = updateEncoderPosition(); //output is current position wrt front zero
  //Serial.println("Got encoder position");
  roll_t imu_data = updateIMUData();
  //Serial.println("Got IMU data");
  float desiredVelocity = balanceController(((1) * (imu_data.roll_angle)), (1) * imu_data.roll_rate, encoder_position); //*****PUT IN OFFSET VALUE BECAUSE THE IMU IS READING AN ANGLE OFF BY +.16 RADIANS
  //Serial.println("Got desired velocity");
  // frontWheelControl also calls a function that sends the PWM signal to the front motor
  // frontWheelControl will update the pid_controller_data.data array with new info
  float current_vel = frontWheelControl((-1) * desiredVelocity, encoder_position); //DESIRED VELOCITY SET TO NEGATIVE TO MATCH SIGN CONVENTION BETWEEN BALANCE CONTROLLER AND
  //Serial.println("Got current velocity");

  rosPublishBikeState(current_vel, desiredVelocity, encoder_position, desired_steer, imu_data.roll_rate, imu_data.roll_angle, speed, forward_speed, battery_voltage, imu_data.yaw);

  char incoming;
  while (Serial3.available()) {
    //Serial.print((char)Serial3.read());
    gps.encode(Serial3.read());
    Serial3.flush(); //WITHOUT THIS THE BUFFER WILL NOT BE ABLE TO BE READ AS FAST AS IT IS WRITTEN TO AND THIS WILL LOOP FOREVER

    // Publish to /nmea ROS topic
    if (incoming == '$') {
      nmea_buffer[nmea_idx] = '\0';
      rosPublishNmea(nmea_buffer);
      nmea_idx = 1;
      nmea_buffer[0] = '$';
    } else {
      nmea_buffer[nmea_idx++] = incoming;
    }
  }

  rosPublishGps(gps.location.lat(), gps.location.lng(), gps.time.hour(), gps.time.minute(), gps.time.second(), gps.location.age(), gps.satellites.value(), gps.course.deg(), gps.speed.mps(), gps.hdop.value(), curr_millis - prev_millis, bits_so_far);
  bits_so_far = 0;
  //Publish address of bike,gps state objects for ROS
  rosPublishPid(pid_controller_data_array);
  prev_millis = curr_millis;
  curr_millis = millis();
  digitalWrite(LED_3, blinkState);
  if (blinkState == HIGH) {
    blinkState = LOW;
  }
  else {
    blinkState = HIGH;
  }

  //Serial.println("roll angle: " + String(imu_data.roll_angle) + " roll rate: " + String(imu_data.roll_rate) + " encoder: " + String(encoder_position));
  //SerialUSB.println("Pulse 1: " + String(pulse_time) + " Pulse 2: " + String(pulse_time2) + " Pulse 5: " + String(pulse_time5) + " Pulse 6: " + String(pulse_time6));
  //SerialUSB.println(speed);

  rosSpinOnce();

  total_millis = total_millis + (curr_millis - prev_millis);
  count += 1;
  if (total_millis >= 1000) {
    //SerialUSB.println("RUNNING AT" + String(count) + " HZ");
    total_millis = 0;
    count = 0;
  }
  //SerialUSB.print("Nav_instr: ");Serial.println(nav_instr);
  
  //light sensor loop


  // Reading the output frequency
  lightSensorFrequency = pulseIn(sensorOut, LOW);
  rosPublishLightSensor(lightSensorFrequency);
}
