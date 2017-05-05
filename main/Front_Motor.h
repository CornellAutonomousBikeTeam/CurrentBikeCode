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
