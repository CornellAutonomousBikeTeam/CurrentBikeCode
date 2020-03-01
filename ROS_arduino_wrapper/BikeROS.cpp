#include "BikeROS.h"

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

RosNodeHandle nh;

// Publishers

// /bike_state
std_msgs::Float32MultiArray bike_state_data;
ros::Publisher bike_state_pub("bike_state", &bike_state_data);

// /gps
std_msgs::Float32MultiArray gps_state_data;
ros::Publisher gps_state_pub("gps", &gps_state_data);

// /pid
std_msgs::Float32MultiArray pid_controller_data;
ros::Publisher pid_pub("pid", &pid_controller_data);

// /nmea
std_msgs::String nmea_state;
ros::Publisher nmea_pub("nmea", &nmea_state);

//lightsensor
std_msgs::Int32 light_sensor_data;
ros::Publisher light_sensor_pub("light_sensor", &light_sensor_data);

// Subscribers

// /nav_instr
float nav_instr;
void rosNavInstrListener(const std_msgs::Float32& data) {
    nav_instr = data.data;
}
ros::Subscriber<std_msgs::Float32> nav_sub("nav_instr", &rosNavInstrListener);

void initROS() {
    nh.initNode();
    
    // Publishers

    // /bike_state
    int bike_state_data_size = 12;
    bike_state_data.data_length = bike_state_data_size;
    bike_state_data.layout.data_offset = 0;
    bike_state_data.layout.dim = (std_msgs::MultiArrayDimension *)
        malloc(sizeof(std_msgs::MultiArrayDimension) * bike_state_data_size);
    bike_state_data.layout.dim[0].label = "desired_vel";
    bike_state_data.layout.dim[1].label = "current_vel";
    bike_state_data.layout.dim[2].label = "imu_rate";
    bike_state_data.layout.dim[3].label = "imu_angle";
    bike_state_data.layout.dim[4].label = "encoder_position";
    bike_state_data.layout.dim[5].label = "desired_steer";
    bike_state_data.data = (float *)malloc(sizeof(float)*bike_state_data_size);
    nh.advertise(bike_state_pub);

    // /gps
    int gps_state_data_size = 12;
    gps_state_data.data_length = gps_state_data_size;
    gps_state_data.layout.data_offset = 0;
    gps_state_data.layout.dim = (std_msgs::MultiArrayDimension *)
        malloc(sizeof(std_msgs::MultiArrayDimension) * gps_state_data_size);
    gps_state_data.data = (float *)malloc(sizeof(float) * gps_state_data_size);
    nh.advertise(gps_state_pub);

    // /pid
    int pid_data_size = 6;
    pid_controller_data.data_length = pid_data_size;
    pid_controller_data.layout.data_offset = 0;
    pid_controller_data.layout.dim = (std_msgs::MultiArrayDimension *)
        malloc(sizeof(std_msgs::MultiArrayDimension) * pid_data_size);
    pid_controller_data.layout.dim[0].label = "current_pos";
    pid_controller_data.layout.dim[1].label = "desired_pos";
    pid_controller_data.layout.dim[2].label = "current_vel";
    pid_controller_data.layout.dim[3].label = "sp_error";
    pid_controller_data.layout.dim[4].label = "sv_error";
    pid_controller_data.layout.dim[5].label = "total_error";
    pid_controller_data.data = (float *)malloc(sizeof(float) * pid_data_size);
    nh.advertise(pid_pub);

    // /nmea
    nh.advertise(nmea_pub);

    //lightsensor
    nh.advertise(light_sensor_pub);

    // Subscribers

    // /nav_instr
    nh.subscribe(nav_sub);
}

void rosPublishBikeState(
        float current_vel, //front motor (rad/s)
        float desired_vel, //front motor (rad/s)
        float encoder_position, //front motor (rad/s) (delta)
        float desired_steer,  //front motor (rad)
        float imu_roll_rate,  //imu (rad) (phi-dot)
        float imu_roll_angle, // //imu (rad/s) (phi)
        float speed, // rear motor (m/s) (based on hall sensor)
        float forward_speed, // rear motor commanded speed (pwm)
        float battery_voltage, // battery voltage
        float imu_yaw) { // yaw (rad)
    // Do not change bike_state indexes - some of them are hard-coded into
    // the nav algorithm
    bike_state_data.data[0] = current_vel;
    bike_state_data.data[1] = desired_vel;
    bike_state_data.data[2] = encoder_position;
    bike_state_data.data[3] = desired_steer;
    bike_state_data.data[4] = imu_roll_rate;
    bike_state_data.data[5] = imu_roll_angle;
    bike_state_data.data[6] = speed;
    bike_state_data.data[7] = forward_speed;
    bike_state_data.data[8] = imu_yaw;
    bike_state_pub.publish(&bike_state_data);
}

void rosPublishGps(
        float latitude, //latitude (deg)
        float longitude, //longitude (deg)
        float hour, // Hour (0-23)
        float minute, // Minute (0-59)
        float second, // Second (0-59)
        float location_age, //how old location data is in ms
        float num_satellites, // Number of satellites in use
        float yaw, //yaw relative to lat,long (deg)
        float speed, //speed from gps (m/s)
        float hdop, // Horizontal Dim. of Precision (100ths-i32)
        float millis_since_prev, //Time since last update
        float bits_so_far) { // bits since last update
    gps_state_data.data[0] = latitude;
    gps_state_data.data[1] = longitude;
    gps_state_data.data[2] = hour;
    gps_state_data.data[3] = minute;
    gps_state_data.data[4] = second;
    gps_state_data.data[5] = location_age;
    gps_state_data.data[6] = num_satellites;
    gps_state_data.data[7] = yaw;
    gps_state_data.data[8] = speed;
    gps_state_data.data[9] = hdop;
    gps_state_data.data[10] = millis_since_prev;
    gps_state_data.data[11] = bits_so_far;
    gps_state_pub.publish(&gps_state_data);
}

void rosPublishPid(const float *data) {
    pid_controller_data.data[0] = data[0];
    pid_controller_data.data[1] = data[1];
    pid_controller_data.data[2] = data[2];
    pid_controller_data.data[3] = data[3];
    pid_controller_data.data[4] = data[4];
    pid_controller_data.data[5] = data[5];
    pid_pub.publish(&pid_controller_data);
}

void rosPublishNmea(const char *data) {
    nmea_state.data = data;
    nmea_pub.publish(&nmea_state);
}

void rosPublishLightSensor(const int32_t data) {
  light_sensor_data.data = data;
  light_sensor_pub.publish(&light_sensor_data);
}

void rosSpinOnce() {
  nh.spinOnce();
}
