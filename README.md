 # CurrentBikeCode - UNSTABLE
 Master branch should have stable code only. If unstable, write unstable at the top.

 ## Known Bugs
  - [ ] This repo needs a git ignore
 
 ## Last commited by
 >[Insert name here]
 
 ## Reason for commit
>[Insert what you did here]

---
# Documentation
The main code that runs on our Arduino Due is composed of the following modules:
* [Front Motor Controller/Encoder](#front)
* [Rear Motor Controller](#rear)
* [RC Controller](#rc)
* [GPS](#gps)
* [Landing Gear](#landing)
* [IMU (Intertial Measurement Unit)](#imu)
* [Watchdog](#watchdog)

Most of the pieces of hardware on the bike have been abstracted as objects in the code. Some, like IMU and Landing Gear are simply composed of convenience functions. Others like Rear and Front Motor Controllers can be initialized and possess methods and variables. Each module has a corresponding .cpp and.h file. All of these components are implemented in the Bike_State class, which can call methods from any of the modules.

The Bike_State files contain the defintiion for the overall Bike object that is intiialized and run in the main file. ROS communication is implemented in the main file. Aside from ROS, everything on the Arduino is implemented within this class.

The main .ino file initalizes a Bike_State object, and then calls one method. Everything else is handled internally within the Bike_State class. ROS communication is implemented in the main .ino file, in order to maintain consistency with the rest of the ROS modules.

---

### <a name="front"></a>Front Motor Controller/Encoder
This object currently represents both the front motor controller and the encoder. One can create a 
new instance of front motor controller using the following code:
```c++
myInstance = new Front_Motor_Controller(int K_p, int K_d, int K_i);
```
Where K_p is the proportional gain, K_d is derivative gain, and K_i is the integral gain. The balance controller for the bike is implemented in this module.

Class Definition:
  <details>
    <summary><small>Front_Motor.h</small></summary><p>
    
    
    
    #ifndef Front_Motor_h
    #define Front_Motor_h 
    #include <SPI.h>
    #include <math.h>
    
    class Front_Motor_Controller { //TODO Overall I can probably separate more stuff in here
        /*
           List of constants and variables used by Front_Motor_Controller
        */
      private:
        //Timed Loop Variables - Repetitive code from Watchdog files
        const long interval = 10000;
        long l_start;
        long l_diff;
    
        //voltage constants and variables- TODO these could totally be separated cause
        //Rear Motor uses them too
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
    
        int steer_dir = 0;
    
        //Balance Control constants TODO think about splitting these
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
    
        //Added by Kenneth for constructors
        const int K_p;
        const int K_d;
        const int K_i;
    
    
      public:
        /*
           Constructors for a Front_Motor_Controller. Parameters to input to this function
           Are Proportional Gain, Differential Gain, and lastly Integration Gain
        */
        Front_Motor_Controller(int, int, int);
    
        /*
           List of methods- TODO figure out which ones can be private.
        */
        void encoderAndFrontMotorSetup(); //Maybe change to just setup()
        void calibrate();
        int velocityToPWM(float);
        float eulerIntegrate(float, float);
        float updateEncoderPosition();
        float frontWheelControl(float, float);
        float balanceController(float, float, float); //And we'll all float on alright
        float PID_Controller(float, signed int, signed int, unsigned long, unsigned long, signed int);
    
        /*
           TODO These variables should be private
        */
        float desired_steer = 0;
        float desired_pos_array[250];
        float theo_position = 0;
    };
    #endif //Front_Motor_h
  </p></details>
  
Methods:

 Return Type  | Method Signature | Description 
:-------------: |:-------------:| :-----:
 void   | encoderAndFrontMotorSetup()| Initializes pins for encoder and front motor controls
 void   | calibrate() | Runs zeroing procedure for calibrating the front motor.
 float |  updateEncoderPosition() | Returns Encoder position in radians
 void |  balanceController(float roll_angle, float roll_rate, float encoder_angle) | Computes desired steer angle to balance bike
 void | frontWheelControl(float desiredVelocity, float current_pos) | PID controller for front motor
 void | PID_Controller(float, signed int, signed int, unsigned long, unsigned long, signed int) | deprecated: use frontwheelcontrol
 
 //TODO go over with Will

---
### <a name="rear"></a>Rear Motor Controller
This class controls the rear motor. It calculates the current speed of the rear motor in order to use
a proportional controller with a feedback loop. Currently under testing.

To initialize a rear motor object use the following code:
```c++
Rear_Motor_Controller myInstance = Rear_Motor_Controller(float K_p);
```
where K_p is the desired proportional gain.

Methods:

 Return Type  | Method Signature | Description 
:-------------: |:-------------:| :-----:
 void   | updateSpeed() | Attached to an interrupt in order to constantly update the bike's current speed- internal to class
 float  | getSpeed() | Returns bike's speed in meters per second (m/s)
 void | controlSpeed(float desiredVelocity)| Commands the proportional controller to hold desiredVelocity
 void |  switchDirection(boolean) | Switches rear motor's direction. True sets motor to forward, and False reverse
 void | setPins() | Initializes pins for rear motor controller.

---
### <a name="rc"></a>RC Controller
Handles signals from RC controller. Has __ channels total. Ask Will Murphy which channels are being used where.
Methods from this class are attached as interrupts in the main code.

Methods:

 Return Type  | Method Signature | Description 
:-------------: |:-------------:| :-----:
 void    | RCsetup()| Initializes pins for RC interrupts
void   |  calcSignal()| Used to adjust bike steering
 void | calcSignal2()| Used to adjust rear motor speed
 void |  calcSignal6() | Used to control landing gear

---
### <a name="gps"></a>GPS

To be collected from Aviv Blumfield

---
### <a name="landing"></a>Landing Gear

This module contains methods for landing gear control. Written by Dylan Meehan.

Methods:

 Return Type  | Method Signature | Description 
:-------------: |:-------------:| :-----:
 void    | landingGearSetup()| Initializes pins that control landing gear relays.
void   |  landingGearDown()| Lowers landing gear for stopping the bike without falling over.
 void | landingGearUp()| Raises landing gear
---
### <a name="imu"></a>IMU
This module handles communication with the IMU. Written by Arjun.

Methods:

  Return Type       | Method Signature          | Description 
:-------------: |:-------------:| :-----:
 void    | initIMU(void) | Initializes the IMU so that it is ready to deliver data
 float   |  float getIMU(byte)|  Method that returns IMU data. Input argument 0x01 returns lean angle, and 0x26 returns lean rate.
---
### <a name="watchdog"></a>Watchdog

Methods:

 Return Type       | Method Signature          | Description 
:-------------: |:-------------:| :-----:
 void    | recordStartTime()| Records Arduino time: called at beginning of main loop
 void   |  verifyEndTime()| Records Arduino time and checks against stored start time to avoid loop length violations
---

