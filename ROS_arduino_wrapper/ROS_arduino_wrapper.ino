#include "BikeROS.h"
#include "TinyGPS++.h"
TinyGPSPlus gps;
#include "IMU.h"
#include "FrontWheel.h"
#include "RC.h"
#include "RearMotor.h"
#include "Encoder.h"
#include "LandingGear.h"
#include <math.h>

int count = 0; //used to determine if we are running at 5hz
int prev_millis = 0; //Used to help calculate time since last update
int curr_millis = 0; //Used to help calculate time since last update
int total_millis = 0; //Holds total time passed so we know when to update Hz
int bits_so_far = 0; //Unclear what this is used for atm

boolean CH1, CH2, CH3, CH4, CH5, CH6; //current cycle's logic

//Timed Loop Variables
long l_start;
long l_diff;

//count the number of times the time step has been calculated to calculate a running average time step
int numTimeSteps = 0;
float averageTimeStep = 0;
int n = 0;

//Watchdog
#define WDI 42
#define EN 41

// Buffer for publishing NMEA sentences from the GPS to /nmea
char nmea_buffer[200];
int nmea_idx = 0;

//#define front_steer_value 51
//#define back_wheel_speed 28

#define relay3 50
#define relay4 49

//LEDs on bike
#define LED_1 22 //red
#define LED_2 35 //yellow
#define LED_3 36 //blue

//Voltage constants and variables
const int VOLTAGE_PIN = 63; //A9
float VOLTAGE_CONST = 14.2;
float battery_voltage = 0;
float VELOCITY_VOLTAGE_K = 1.7936;
float VELOCITY_VOLTAGE_C = -1.2002;


//Method for sending hex messages to the gps
void sendUBX(byte *UBXmsg, byte msgLength) {
  for (int i = 0; i < msgLength; i++) {
    Serial3.write(UBXmsg[i]);
  }
}

/*Method to set the steer and speed based on either rc or nav instructors depending on mode set by remote*/
void navOrRC() {
  if (nav_mode) {
    desired_steer = nav_instr; //Get desired steer from the nav instructions
    desired_lean = (desired_speed * desired_speed / 10.0) * desired_steer; //phi_d = (v^2/l/g) * delta_d
    rear_pwm = (int)(gain_p * (desired_speed - speed) + rear_pwm); //Actual Controller

    if (rear_pwm > 180) {
      rear_pwm = 180; //Max PWM value that bike can do, regardless of what nav algo wants to say
    }
    if (rear_pwm < 60) {
      rear_pwm = 60; //Min PWM value that bike can do, regardless of what nav algo wants to say
    }

    forward_speed = rear_pwm;
    SerialUSB.println("Nav mode");
  }
  else {
    forward_speed = map(pulse_time2, 1100, 1900, 0, 200);
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
    analogWrite(PWM_front, 50);
    y = REG_TC0_CV1;
    //Serial.println("Ticking");
    //Serial.println((String(REG_TC0_CV0) + '\t' + y).c_str());
  }

  x_offset = REG_TC0_CV0;   //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  analogWrite(PWM_front, 0);

  //rear motor initialization
  int pwm = 60;
  rampToPWM(170, 0);

  digitalWrite(LED_1, HIGH); //LED to signal setup function done

  nav_mode = true; //Variable that tells whether bike is in nav mode or not
}

//Loop variables
int blinkState = HIGH;
void loop() {
  numTimeSteps++;

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
    total_millis = 0;  l_diff = micros() - l_start;
    count = 0;
  }
  //SerialUSB.print("Nav_instr: ");Serial.println(nav_instr);
}
