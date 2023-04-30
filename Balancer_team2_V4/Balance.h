#pragma once

#include <stdint.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <Balboa32U4.h>

// This code was developed for a Balboa unit using 50:1 motors
// and 45:21 plastic gears, for an overall gear ratio of 111.
// Adjust the ratio below to scale various constants in the
// balancing algorithm to match your robot.
const int16_t GEAR_RATIO = 111;

// This constant limits the maximum motor speed.  If your gear
// ratio is lower than what we used, or if you are testing
// changes to the code, you might want to reduce it to prevent
// your robot from zooming away when things go wrong.
//
// If you want to use speeds faster than 300, you should add
// the line "motors.allowTurbo(true);" to setup().
const int16_t MOTOR_SPEED_LIMIT = 300;
const int16_t MOTOR_SPEED_LIMIT_PI = 28000;//millirad/s correspondant a 300 en MOTORSPEEDLIMIT
// This constant relates the angle to its rate of change for a
// robot that is falling from a nearly-vertical position or
// rising up to that position.  The relationship is nearly
// linear.  For example, if you have the 80mm wheels it should be
// about 140, which means that the angle in millidegrees is ~140
// times its rate of change in degrees per second; when the robot
// has fallen by 90 degrees it will be moving at about
// 90,000/140 = 642 deg/s.  See the end of Balancer.ino for one
// way to calibrate this value.
const int16_t ANGLE_RATE_RATIO = 140;

// The following three constants define a PID-like algorithm for
// balancing.  Each one determines how much the motors will
// respond to the corresponding variable being off from zero.
// See the code in Balance.cpp for exactly how they are used.  To
// get it balancing from scratch, start with them all at zero and
// adjust them as follows:

// ANGLE_RESPONSE determines the response to a combination of
// angle and angle_rate; the combination measures how far the
// robot is from a stable trajectory.  To test this, use your
// hand to flick the robot up from a resting position.  With a
// value that is too low, it won't stop itself in time; a high
// value will cause it to slam back into the ground or oscillate
// wildly back and forth.  When ANGLE_RESPONSE is adjusted
// properly, the robot will move just enough to stop itself.
// However, after stopping itself, it will be moving and keep
// moving in the same direction, usually driving faster and
// faster until it reaches its maximum motor speed and falls
// over.  That's where the next constants come in.
const int16_t ANGLE_RESPONSE = 11;

// DISTANCE_RESPONSE determines how much the robot resists being
// moved away from its starting point.  Counterintuitively, this
// constant is positive: to move forwards, the robot actually has
// to first roll its wheels backwards, so that it can *fall*
// forwards.  When this constant is adjusted properly, the robot
// will no longer zoom off in one direction, but it will drive
// back and forth a few times before falling down.
const int16_t DISTANCE_RESPONSE = 73;

// DISTANCE_DIFF_RESPONSE determines the response to differences
// between the left and right motors, preventing undesired
// rotation due to differences in the motors and gearing.  Unlike
// DISTANCE_REPONSE, it should be negative: if the left motor is
// lagging, we need to increase its speed and decrease the speed
// of the right motor.  If this constant is too small, the robot
// will spin left and right as it rocks back and forth; if it is
// too large it will become unstable.
const int16_t DISTANCE_DIFF_RESPONSE = -50;

// SPEED_RESPONSE supresses the large back-and-forth oscillations
// caused by DISTANCE_RESPONSE.  Increase this until these
// oscillations die down after a few cycles; but if you increase
// it too much it will tend to shudder or vibrate wildly.
const int16_t SPEED_RESPONSE = 3300;

// The balancing code is all based on a 100 Hz update rate; if
// you change this, you will have to adjust many other things.
const uint8_t UPDATE_TIME_MS = 10;

// Take 100 measurements initially to calibrate the gyro.
const uint8_t CALIBRATION_ITERATIONS = 100;

// Counts per recolution of the balboa encoders
const uint16_t CPR=1333;

// These values represent the angles from vertical, in
// millidegrees, at which the Balboa will start and stop trying
// to balance.  They are different to add some hysteresis so that
// the robot has to be relatively more upright before beginning
// to balance but can fall a little farther before giving up.
// The default values make the Balboa start balancing at
// 45 degrees from vertical and stop balancing at 70 degrees from
// vertical.
const int32_t START_BALANCING_ANGLE = 45000;
const int32_t STOP_BALANCING_ANGLE = 70000;


const uint8_t dw=5; //distance wheel to center cm
const uint8_t R=4;//Radius of wheel in cm
const int16_t ANGLE_OFFSET=45;


// These variables will be accessible from your sketch.
extern int32_t angle; // units: millidegrees
extern int32_t angleRate; // units: degrees/s (or millidegrees/ms)
extern int16_t motorSpeed; // current (average) motor speed setting

// These variables must be defined in your sketch.
extern LSM6 imu;
extern LIS3MDL mag;
extern Balboa32U4Motors motors;
extern Balboa32U4Encoders encoders;

// Reduced observer variables
// const int32_t L[3] ={9,1723,256};
// const int32_t A21mLA11[3] ={-1784,-1476,-256};
// const int32_t A22mLA12 =741;
// const int32_t B2mLB1 =3991;
// const int32_t L[3] ={20,3842,571};
// const int32_t A21mLA11[3] ={-3292,-3471,-571};
// const int32_t A22mLA12 =607;
// const int32_t B2mLB1 =6030;
const int32_t L[3] ={3,494,73};
const int32_t A21mLA11[3] ={-909,-318,-73};
const int32_t A22mLA12 =819;
const int32_t B2mLB1 =2808;
// position control variables
const int32_t Kssp[4] ={-3791,-448,-146,-161};
const int32_t nr =-146;

// position control integral variables
const int32_t KsspI[4] ={-3948,-475,-175,-172};
const int32_t KIsspI =-103;

// speed control variables
// const int32_t Kssp_s[3] ={-2655,-293,-109};
// const int32_t nr_s =-173;
const int32_t Kssp_s[3] ={-1492,-134,-76};
const int32_t nr_s =-140;

// speed control integral variables
// const int32_t KsspI_s[3] ={-3728,-428,-149};
// const int32_t KIsspI_s =-98;
// const int32_t KsspI_s[3] ={-3093,-344,-128};
// const int32_t KIsspI_s =-74;
// const int32_t KsspI_s[3] ={-2979,-330,-126};
// const int32_t KIsspI_s =-75;
const int32_t KsspI_s[3] ={-3188,-361,-136};
const int32_t KIsspI_s =-94;


//calibration data of the magnetometer
//magnetometer calibration data
const int32_t B[3] = { 237, 417, -5072};

const int32_t Ainv[3][3] = {{  847,  116, -120},
  {  116,  1138,  28},
  { -120,  28,  1244}
};


const int MAX_MESSAGE_LENGTH=12;



//vector math for LIS3MDL.h
template <typename T> struct vector
    {
      T x, y, z;
    };



void Drive();
void readData();

// Call this in your setup() to initialize and calibrate the IMU.
void balanceSetup();

// Call this in loop() to run the full balancing algorithm.
void balanceUpdate();

// Call this function to set a driving speed in ticks/ms.  The
// way it works is that every update cycle we adjust the robot's
// encoder measurements, which will cause it to drive in the
// corresponding direction.  Differing values for left and right
// will result in a turn.
void balanceDrive(int16_t leftSpeed, int16_t rightSpeed);

// Returns true if (according to the balancing algorithm) the
// robot is trying to balance.  When it falls down it shuts off
// the motors, and this function will return false.  If you pick
// the robot up, this function will start returning true again.
bool isBalancing();

// Returns true if the last update cycle was delayed to more than
// UPDATE_TIME_MS+1 milliseconds.  This could indicate
// computations being too long or interrupts that are delaying
// the loop.
bool balanceUpdateDelayed();

// Sometimes you will want to take control of the motors but keep
// updating the balancing code's encoders and angle measurements
// so you don't lose track of the robot's position and angle.
// Call this every 10ms (UPDATE_TIME_MS) to update the sensors,
// and you will be able to resume balancing immediately when you
// are done.
void balanceUpdateSensors();

// Call this function to reset the encoders.  This is useful
// after a large motion, so that robot does not try to make a
// huge correction to get back to "zero".
void balanceResetEncoders();

//read data from magnetometer
void read_mag_data();
//float compute_heading(vector<int16_t> from);
void compute_heading();