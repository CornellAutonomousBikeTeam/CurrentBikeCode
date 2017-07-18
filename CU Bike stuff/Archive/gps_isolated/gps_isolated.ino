#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "TinyGPS++.h"
TinyGPSPlus gps;
TinyGPSCustom heading(gps, "GPTHS", 1); 
#include "IMU.h"
#include "PID.h"
#include "RearMotor.h"
#include <math.h>
//difference between timer_start and micros() is the length of time that the pin
//was HIGH - the PWM pulse length. volatile int pulse_time;
//this is the time that the last interrupt occurred.
//you can use this to determine if your receiver has a signal or not.=


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

//  method for sending hex messages to the gps
int curr_millis = 0;
int prev_millis = 0;
void sendUBX(byte *UBXmsg, byte msgLength) {
   for(int i = 0; i < msgLength; i++){
      Serial3.write(UBXmsg[i]);
   }
}
//this is all normal arduino stuff
void setup()
{
  //ROS STUFF
  nh.initNode();
  int data_size = 10;
  int gps_state_data_size = 11;
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

  Serial.begin(57600);
  //GPS
  // GPS baudrate (gps hardware runs natively at 9600)
  Serial3.begin(9600);
  /*
  CURRENT DEFAULT CONFIGURATION: 57600 Baud; 10 hz update rate
  */
  //hex messages that config gps data rate to 5Hz
  //byte setUBX[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x89};
  byte gpsmsg10[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; //10hz
  byte gpsmsg8[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x7D, 0x00, 0x01, 0x00, 0x01, 0x00, 0x93, 0xA8}; //8hz
  byte gpsmsg5[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A}; //5hz
  byte gpsmsg4[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96}; //3.3hz
  byte gpsmsg33[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x2C, 0x01, 0x01, 0x00, 0x01, 0x00, 0x43, 0xC7}; //3.3hz
  byte gpsmsg1[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0E8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39}; //1hz

  byte gpsmsgpoll[] = {0xB5, 0x62, 0x06, 0x00};
  byte gpsmsgbaud[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xBF };
  byte gpsmsgbaud2[] = {0xB5, 0x62, 0x06, 0x41, 0x09, 0x00, 0x01, 0x01, 0x30, 0x81, 0x00, 0x00, 0x00, 0x00, 0xFB, 0xFE, 0x1F};
  byte gpsConvertToDefault[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1B, 0x9A};
  byte gpsmsgSaveConf[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB};
  byte gpsmsgbaud57600[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xE1,0x00,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xDA,0xA9}; //UBX and NMEA
  byte gpsmsgbaud19200[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x4B,0x00,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x48,0x57};
  //Configures gps to 5hz update rate; change gpsmsg5 to a different byte encoding for different configurations
  //sendUBX(gpsmsgpoll, sizeof(gpsmsgpoll));
  //sendUBX(gpsConvertToDefault, sizeof(gpsConvertToDefault));
  //sendUBX(gpsmsgbaud57600, sizeof(gpsmsgbaud57600));
  sendUBX(gpsmsg8, sizeof(gpsmsg8));
  sendUBX(gpsmsgbaud57600, sizeof(gpsmsgbaud57600));
  sendUBX(gpsmsgSaveConf, sizeof(gpsmsgSaveConf));
  Serial3.end();
  Serial3.begin(57600); //This was originally 9600 - test with higher baud rates

}

//Loop variables
void loop() {
  //byte GPSon[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76};
  //byte GPSoff[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74};
  //gps data (Don't change these indexes either)
  //Serial.println("New read from serial");
  while (Serial3.available()) {
    //Serial.print((char)Serial3.read());
    gps.encode(Serial3.read());
    Serial3.flush();
    //Serial.println(gps.location.isUpdated());
  }
  if (gps.time.isUpdated()) {
     prev_millis = curr_millis;
    curr_millis = millis();
      {
        Serial.print("Sentences with a fix: "); Serial.println(gps.sentencesWithFix());
        Serial.print("Failed Checksum: "); Serial.println(gps.failedChecksum());
        //Serial.print("Heading"); Serial.println(heading.value());
        Serial.print("Time since last update: "); Serial.println(curr_millis - prev_millis);
        //prev_millis = curr_millis;
        //curr_millis = millis();
        //Serial.print("Curr_millis: "); Serial.println(curr_millis);
        //Serial.print("Prev_millis: "); Serial.println(prev_millis);
        
        Serial.print("Latitude                      "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude                     "); Serial.println(gps.location.lng(), 6);
        Serial.print("Age                     "); Serial.println(gps.location.age(), 6);
        Serial.print("Hours                         "); Serial.println(gps.time.hour()); // Hour (0-23)
        Serial.print("Minutes                       "); Serial.println(gps.time.minute()); // Minute (0-59)
        Serial.print("Seconds                       "); Serial.println(gps.time.second()); // Second (0-59)
        Serial.print("Centiseconds                  "); Serial.println(gps.time.centisecond()); // 100ths of a second (0-99)
        Serial.print("Date                          "); Serial.println(gps.date.value()); // Raw date in DDMMYY format
        Serial.print("Number of Satelites in use    "); Serial.println(gps.satellites.value()); // Number of satellites in use
        Serial.print("Heading                       "); Serial.println(gps.course.deg());
      //}
      //Serial.print("Valid remaining data: "); Serial.println(gps.charsProcessed());
      //Serial.print("Sentences that failed checksum="); Serial.println(gps.failedChecksum());
      // Testing overflow in SoftwareSerial is sometimes useful too.
      //Serial.print("Soft Serial device overflowed? "); Serial.println(Serial3.overflow() ? "YES!" : "No");
      //Serial.print("Serial3 read in"); Serial.println(Serial.read());
    }
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
   gps_pub.publish( &gps_state );
  state_pub.publish( &bike_state );
  nh.spinOnce();
}



//}


