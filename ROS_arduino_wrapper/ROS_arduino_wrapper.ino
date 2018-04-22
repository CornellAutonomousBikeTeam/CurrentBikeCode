#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
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
int prev_millis = 0;
int curr_millis = 0;
int total_millis = 0;
int bits_so_far = 0;

//ros::NodeHandle  nh;
ros::NodeHandle_<ArduinoHardware, 1, 3, 500, 500> nh; //set this back to 500 

std_msgs::Float32MultiArray bike_state; //Array containing bike state variables
std_msgs::Float32MultiArray gps_state; //Array containing gps state variables
ros::Publisher state_pub("bike_state", &bike_state); //Publisher object for the bike state
ros::Publisher gps_pub("gps", &gps_state); //Publisher object for the gps state
ros::Publisher pid_pub("pid", &pid_controller_data); // Publisher object for the pid controller debug variables

float nav_instr = 0; //Variable for nav steer instructions

//Ros listener
void updateInstruction(const std_msgs::Float32& data) {nav_instr = data.data;} //Update nav_instr var with nav algo data
uint32_t queue_size = 3000; 
ros::Subscriber<std_msgs::Float32> nav_sub("nav_instr", &updateInstruction);


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
      desired_steer = nav_instr;
      desired_lean = (desired_speed*desired_speed/10.0)*desired_steer; //phi_d = (v^2/l/g) * delta_d
      rear_pwm = (int)(gain_p * (desired_speed - speed) + rear_pwm); //Actual Controller
      if (rear_pwm > 180) {
        rear_pwm = 180;
      }
      if (rear_pwm < 60) {
        rear_pwm = 60;
      }
      foreward_speed = rear_pwm;
      SerialUSB.println("Nav mode");
    }
    else { 
      foreward_speed = map(pulse_time2, 1100, 1900, 0, 200);
      steer_range = map(pulse_time, 1100, 1900, 60, -60);
      desired_steer = steer_range * .01 ;
      desired_lean = (desired_speed*desired_speed/10.0)*desired_steer; //phi_d = (v^2/l/g) * delta_d
     // desired_lean = (speed*speed)*desired_steer; //phi_d = (v^2/l/g) * delta_d
     // desired_lean = (gps_state.data[8]*gps_state.data[8]/10.0)*desired_steer; //phi_d = (v^2/l/g) * delta_d
      SerialUSB.println("RC mode");
    }

}


/* takes in desired angular velocity, returns pwm. Currently unused. */
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

void setup()
{
  nh.initNode();
  int data_size = 12;
  int gps_state_data_size = 12;
  int pid_data_size = 6;
  nh.subscribe(nav_sub);
  bike_state.data_length = data_size;
  gps_state.data_length = gps_state_data_size;
  pid_controller_data.data_length = pid_data_size;
  bike_state.layout.data_offset = 0;
  gps_state.layout.data_offset = 0;
  pid_controller_data.layout.data_offset = 0;

  
  bike_state.layout.dim = (std_msgs::MultiArrayDimension *)
                          malloc(sizeof(std_msgs::MultiArrayDimension) * data_size); //Allocate memory for the bike_state layout
  gps_state.layout.dim = (std_msgs::MultiArrayDimension *)
                         malloc(sizeof(std_msgs::MultiArrayDimension) * data_size); //Allocate memory for the gps_state layout
  pid_controller_data.layout.dim = (std_msgs::MultiArrayDimension *)
      malloc(sizeof(std_msgs::MultiArrayDimension) * pid_data_size); // Allocate memory for the pid_controller_data layout

  bike_state.layout.dim[0].label = "desired_vel";
  bike_state.layout.dim[1].label = "current_vel";
  bike_state.layout.dim[2].label = "imu_rate";
  bike_state.layout.dim[3].label = "imu_angle";
  bike_state.layout.dim[4].label = "encoder_position";
  bike_state.layout.dim[5].label = "desired_steer";

  pid_controller_data.layout.dim[0].label = "current_pos";
  pid_controller_data.layout.dim[1].label = "desired_pos";
  pid_controller_data.layout.dim[2].label = "current_vel";
  pid_controller_data.layout.dim[3].label = "sp_error";
  pid_controller_data.layout.dim[4].label = "sv_error";
  pid_controller_data.layout.dim[5].label = "total_error";

  //Allocate memory for the outgoing data
  bike_state.data = (float *)malloc(sizeof(float) * data_size);
  gps_state.data = (float *)malloc(sizeof(float) * gps_state_data_size);
  pid_controller_data.data = (float *)malloc(sizeof(float) * pid_data_size);

  nh.advertise(state_pub);
  nh.advertise(gps_pub);
  nh.advertise(pid_pub);

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
  byte gpsmsgbaud57600[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0xE1,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x16,0x51}; //UBX and NMEA
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
    while(y==oldIndex) {
    analogWrite(PWM_front,50);
    y = REG_TC0_CV1;
    //Serial.println("Ticking");
    }

  x_offset = REG_TC0_CV0;   //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  analogWrite(PWM_front, 0);
  
  //rear motor initialization
  int pwm = 60;
  rampToPWM(170, 0);

  digitalWrite(LED_1, HIGH); //LED to signal setup function done
  nav_mode = true;
}

//Loop variables
int blinkState = HIGH;
void loop() {
  numTimeSteps++;

  navOrRC();

  analogWrite(PWM_rear, foreward_speed);

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

  // Do not change bike_state indexes - some of them are hard-coded into
  // the nav algorithm
  bike_state.data[0] = current_vel; //front motor (rad/s)
  bike_state.data[1] = desiredVelocity; //front motor (rad/s)
  bike_state.data[2] = encoder_position; //front motor (rad) (delta)
  bike_state.data[3] = desired_steer; //front motor (rad)
  bike_state.data[4] = imu_data.roll_rate; //imu (rad) (phi-dot)
  bike_state.data[5] = imu_data.roll_angle; //imu (rad/s) (phi)
  bike_state.data[6] = speed; //rear motor (m/s) (based on hall sensor)
  bike_state.data[7] = foreward_speed; //rear motor commanded speed (pwm)
  bike_state.data[8] = battery_voltage;
  bike_state.data[9] = imu_data.yaw; //yaw (rad)
  //Serial.print("YAW: "); Serial.println(imu_data.yaw);
  
  //gps data (Don't change these indexes either)
  while (Serial3.available()) {
    //Serial.print((char)Serial3.read());
    gps.encode(Serial3.read());
    Serial3.flush(); //WITHOUT THIS THE BUFFER WILL NOT BE ABLE TO BE READ AS FAST AS IT IS WRITTEN TO AND THIS WILL LOOP FOREVER
    //Serial.println(gps.location.isUpdated());
  }
  //Serial.print("Lat: ");Serial.println(gps.location.lat());
  //Serial.print("Long: ");Serial.println(gps.location.lng());
  //Serial.print("Age: "); Serial.println(gps.time.second()); 
  
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
  //Publish address of bike,gps state objects for ROS
  gps_pub.publish( &gps_state );
  state_pub.publish( &bike_state );
  pid_pub.publish( &pid_controller_data );
  //Serial.println("published");
  nh.spinOnce();
  prev_millis = curr_millis;
  curr_millis = millis();
  //    delay(1);
  digitalWrite(LED_3, blinkState);
  if(blinkState == HIGH) {blinkState = LOW;} 
  else {blinkState = HIGH;}

  //Serial.println("roll angle: " + String(imu_data.roll_angle) + " roll rate: " + String(imu_data.roll_rate) + " encoder: " + String(encoder_position));
  SerialUSB.println("Pulse 1: " + String(pulse_time) + " Pulse 2: " + String(pulse_time2) + " Pulse 5: " + String(pulse_time5) + " Pulse 6: " +String(pulse_time6));
  SerialUSB.println(speed);
    
  total_millis = total_millis + (curr_millis - prev_millis);
  count += 1;
  if(total_millis >= 1000) {
    //SerialUSB.println("RUNNING AT" + String(count) + " HZ");
    total_millis = 0;  l_diff = micros() - l_start;
    count = 0;
  }
  //SerialUSB.print("Nav_instr: ");Serial.println(nav_instr);
}




