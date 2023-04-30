#include <Wire.h>
#include <LIS3MDL.h>
#include "Balance.h"

//#include "vector.h"

int32_t gYZero;
int32_t angle; // millidegrees
int32_t angleRate; // degrees/s
int32_t thetaAccel; //milliradian
int32_t thetaRate; //milliradian/s
int32_t thetaCompl; //milliradian
int32_t phi; //milliradian
int32_t phi_prev; //temp var for debug
int32_t INTEGRAL;

int32_t phiRate; //milliradian/s
int32_t thetaComplPrevious;
int32_t XmLY;

int32_t heading;


const int32_t K=1000*0.01/1;
const int32_t angleOffset=-2500; //milidegrees
int32_t distanceLeft;
int32_t phiLeft;//milliradian
int32_t speedLeft;
int32_t driveLeft;
int32_t distanceRight;
int32_t phiRight;//milliradian
int32_t speedRight;
int32_t driveRight;
int16_t motorSpeed;
bool isBalancingStatus = false;
bool balanceUpdateDelayedStatus;

int i;


int32_t setPoint=0;//position setpoint
int32_t setPoint_s=0;//speed setpoint
int32_t setPointA;
int32_t pos;
int32_t rot;

//Odometry related variables
int32_t x; //position in x mm
int32_t y; //position in y mm
int32_t yaw; //yaw angle microrad

//magnetometer calibration data


// typedef struct vector
// {
// 	int16_t x, y, z;  
// } vector;

vector<int16_t> compass;

void sendOdometry(){
  Serial.print(x/1000); //x;y;yaw\n
  Serial.print(";");
  Serial.print(y/1000);
  Serial.print(";");
  Serial.print(yaw/1000);
  Serial.print("\n");
}


void treatCommand(char message[MAX_MESSAGE_LENGTH]){

  if(String(message)=="DF"){
    setPoint_s=-600000;
  }
  else if(String(message)=="DB"){
    setPoint_s=600000;
  }
  else if(String(message)=="RCC"){
    setPointA+=5000;
  }
  else if(String(message)=="RC"){
    setPointA-=5000;
  }
  else if(String(message)=="O"){
    sendOdometry();
  }
  else if(String(message)=="R"){//Reset odometry
  x=0;
  y=0;
  yaw=0;
  }
}
void ComputeOdom(){

  static int16_t lastCountsLeftO;
  int16_t countsLeftO = encoders.getCountsLeft();
  int32_t dl = R*(countsLeftO - lastCountsLeftO)*(2000*PI)/CPR;//distance in millirad since previous iteration
  lastCountsLeftO = countsLeftO;

  static int16_t lastCountsRightO;
  int16_t countsRightO = encoders.getCountsRight();
  int32_t dr = R*(countsRightO - lastCountsRightO)*(2000*PI)/CPR;//distance in millirad since previous iteration
  lastCountsRightO = countsRightO;
  
  int32_t d=(dl+dr)/2; //distance traveled since last computation 10^-5m =>/1000 to get cm
  //we found by experiment that 180° rotation is 28332 in dr-dl, so rule of third gives us a factor of 1000000*pi/28332 to get to microrad
  //we go to microrad because in 10ms the rotation is very small so we need that accuracy
  int32_t dyaw=1000000*PI*(dr-dl)/28332; //yaw rotation since last computation microrad 
  x+=d*cos((yaw+dyaw/2)/1000000.0); // here yaw is indeed yaw of previous iteration because we only update it after
  y+=d*sin((yaw+dyaw/2)/1000000.0); // divide by 1000000.0 to get radians and to ensure result is a floating point and not a int
  yaw+=dyaw;//microrad
  // Serial.print("yaw: ");
  // Serial.print(yaw/1000);//in millirad
  // Serial.print("  x: ");
  // Serial.print(x/1000);//in cm
  // Serial.print("  y: ");
  // Serial.println(y/1000);//in cm
}


bool isBalancing()
{
  return isBalancingStatus;
}

bool balanceUpdateDelayed()
{
  return balanceUpdateDelayedStatus;
}

void balanceSetup()
{
  // Initialize IMU.
  Serial.begin(600);
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize IMU!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s
  // Initialize Magnetometer
  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
    while (1);
  }
  mag.enableDefault();

  // Wait for IMU readings to stabilize.
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (int i = 0; i < CALIBRATION_ITERATIONS; i++)
  {
    imu.read();
    total += imu.g.y;
    delay(1);
  }

  gYZero = total / CALIBRATION_ITERATIONS;
  thetaComplPrevious=1000*atan2(imu.a.z,imu.a.x);
}

// This function contains the core algorithm for balancing a
// Balboa 32U4 robot.
void balance()
{
  // Adjust toward angle=0 with timescale ~10s, to compensate for
  // gyro drift.  More advanced AHRS systems use the
  // accelerometer as a reference for finding the zero angle, but
  // this is a simpler technique: for a balancing robot, as long
  // as it is balancing, we know that the angle must be zero on
  // average, or we would fall over.
  //angle = angle * 999 / 1000;

  // This variable measures how close we are to our basic
  // balancing goal - being on a trajectory that would cause us
  // to rise up to the vertical position with zero speed left at
  // the top.  This is similar to the fallingAngleOffset used
  // for LED feedback and a calibration procedure discussed at
  // the end of Balancer.ino.
  //
  // It is in units of millidegrees, like the angle variable, and
  // you can think of it as an angular estimate of how far off we
  // are from being balanced.
  int32_t risingAngleOffset = angleRate * ANGLE_RATE_RATIO + angle;

  // Combine risingAngleOffset with the distance and speed
  // variables, using the calibration constants defined in
  // Balance.h, to get our motor response.  Rather than becoming
  // the new motor speed setting, the response is an amount that
  // is added to the motor speeds, since a *change* in speed is
  // what causes the robot to tilt one way or the other.
  motorSpeed += (
    + ANGLE_RESPONSE * risingAngleOffset
    + DISTANCE_RESPONSE * (distanceLeft + distanceRight)
    + SPEED_RESPONSE * (speedLeft + speedRight)
    ) / 100 / GEAR_RATIO;

  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }
  int16_t distanceDiff = distanceLeft - distanceRight;
  int32_t leftSpeed=motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100;
  int32_t rightSpeed= motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100;

  // Adjust for differences in the left and right distances; this
  // will prevent the robot from rotating as it rocks back and
  // forth due to differences in the motors, and it allows the
  // robot to perform controlled turns.
  //int16_t distanceDiff = distanceLeft - distanceRight;

  motors.setSpeeds(
    leftSpeed,
    rightSpeed);
}


void BalanceSS(){


  int32_t dutycycle;

  //POSITION CONTROL
  
  dutycycle=(-Kssp[0]*thetaCompl-Kssp[1]*thetaRate-Kssp[2]*phi-Kssp[3]*phiRate+nr*setPoint)/1000;
  motorSpeed=0.5*dutycycle*MOTOR_SPEED_LIMIT/1000;
  // SATURATION
  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }

  //POSITION CONTROL INTEGRAL

  // INTEGRAL+=KIsspI*(setPoint-phi)*UPDATE_TIME_MS; // Euler backward integration
  // dutycycle=(-KsspI[0]*thetaCompl-KsspI[1]*thetaRate-KsspI[2]*phi-KsspI[3]*phiRate+INTEGRAL/1000)/1000;
  // motorSpeed=0.5*dutycycle*MOTOR_SPEED_LIMIT/1000;
  // // ANTI WIND-UP
  // if(dutycycle> 2000){
  //   INTEGRAL=1000*(1000*2000+KsspI[0]*thetaCompl+KsspI[1]*thetaRate+KsspI[2]*phi+KsspI[3]*phiRate);
  //   motorSpeed=MOTOR_SPEED_LIMIT;
  // }
  // else if(dutycycle< -2000){
  //   INTEGRAL=1000*(-1000*2000+KsspI[0]*thetaCompl+KsspI[1]*thetaRate+KsspI[2]*phi+KsspI[3]*phiRate);
  //   motorSpeed=-MOTOR_SPEED_LIMIT;
  // }
  

  //SPEED CONTROL

  // dutycycle=(-Kssp_s[0]*thetaCompl-Kssp_s[1]*thetaRate-Kssp_s[2]*phiRate+nr_s*setPoint_s/1000)/1000;
  // motorSpeed=0.5*dutycycle*MOTOR_SPEED_LIMIT/1000;
  // //SATURATION
  // if (motorSpeed > MOTOR_SPEED_LIMIT)
  // {
  //   motorSpeed = MOTOR_SPEED_LIMIT;
  // }
  // if (motorSpeed < -MOTOR_SPEED_LIMIT)
  // {
  //   motorSpeed = -MOTOR_SPEED_LIMIT;
  // }

  //SPEED CONTROL INTEGRAL
  
  // setPoint_s=0;
  // INTEGRAL+=(setPoint_s-phiRate)*UPDATE_TIME_MS; // Euler backward integration
  // dutycycle=(-KsspI_s[0]*thetaCompl-KsspI_s[1]*thetaRate-KsspI_s[2]*phiRate+KIsspI_s*INTEGRAL/1000)/1000;
  // motorSpeed=0.5*dutycycle*MOTOR_SPEED_LIMIT/1000;
  // //ANTI WIND-UP
  // if(dutycycle> 2000){
  //   INTEGRAL=1000*(1000*2000+KsspI_s[0]*thetaCompl+KsspI_s[1]*thetaRate+KsspI_s[2]*phi)/KIsspI_s;
  //   motorSpeed=MOTOR_SPEED_LIMIT;
  // }
  // else if(dutycycle< -2000){
  //   INTEGRAL=1000*(-1000*2000+KsspI_s[0]*thetaCompl+KsspI_s[1]*thetaRate+KsspI_s[2]*phi)/KIsspI_s;
  //   motorSpeed=-MOTOR_SPEED_LIMIT;
  // }
  


  //Serial.println(motorSpeed);

  int16_t distanceDiff = distanceLeft - distanceRight;
  int32_t leftSpeed=motorSpeed + distanceDiff * DISTANCE_DIFF_RESPONSE / 100;
  int32_t rightSpeed= motorSpeed - distanceDiff * DISTANCE_DIFF_RESPONSE / 100;
  motors.setSpeeds(
    leftSpeed,
    rightSpeed);
}


void lyingDown()
{
  // Reset things so it doesn't go crazy.
  motorSpeed = 0;
  distanceLeft = 0;
  distanceRight = 0;
  motors.setSpeeds(0, 0);

  if (angleRate > -2 && angleRate < 2)
  {
    // It's really calm, so use the accelerometer to measure the
    // robot's rest angle.  The atan2 function returns a result
    // in radians, so we multiply it by 180000/pi to convert it
    // to millidegrees.
    angle = atan2(imu.a.z, imu.a.x) * 57296;

    distanceLeft = 0;
    distanceRight = 0;
  }
}

void integrateGyro()
{
  // Convert from full-scale 1000 deg/s to deg/s.
  angleRate = (imu.g.y - gYZero) / 29;

  angle += angleRate * UPDATE_TIME_MS;
}

void complemantaryAlgorithm()
{
  thetaAccel=1000*atan2(imu.a.z,imu.a.x); //atan2 so div by 0 does not compute a error
  thetaRate = 1000*(PI/180)*(imu.g.y - gYZero) / 29; //compute angle from accelerometer data
  thetaCompl= ((1000-K)*(thetaComplPrevious+UPDATE_TIME_MS*thetaRate/1000)+K*thetaAccel)/1000; //complementary filter equation
  
  thetaComplPrevious=thetaCompl; //save result for next iteration
  thetaCompl-=ANGLE_OFFSET;

  angle=(180/PI)*thetaCompl;
  angleRate=(180/PI)*thetaRate/1000;
  // yaw=1000*atan2((mag.m.y*cos(thetaCompl/1000)-mag.m.x*sin(thetaCompl/1000)),(cos(thetaCompl/1000)*mag.m.z));
  // Serial.print("Yaw:");
  // Serial.println(yaw*180/PI);
  // Serial.print("Angle:");
  // Serial.println(thetaCompl);
}

void phiRateObserver()
{
  static int16_t previousCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - previousCountsLeft);
  distanceLeft += speedLeft;
  phiLeft = distanceLeft*(2000*PI)/CPR;
  previousCountsLeft = countsLeft;




  static int16_t previousCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - previousCountsRight);
  distanceRight += speedRight;
  phiRight = distanceRight*(2000*PI)/CPR;
  previousCountsRight = countsRight;

  phi=(phiRight+phiLeft)/2;
  int32_t Uk= motorSpeed;
  Uk=2000*Uk/MOTOR_SPEED_LIMIT;//weird 2 phase computation because of variable types

  static int32_t XmLY_prev;
  XmLY=(A22mLA12*phiRate+A21mLA11[0]*thetaCompl+A21mLA11[1]*thetaRate+A21mLA11[2]*phi+B2mLB1*Uk)/1000;
  phiRate=XmLY_prev+(L[0]*thetaCompl+L[1]*thetaRate+L[2]*phi)/1000;
  XmLY_prev=XmLY;
  //Serial.println(XmLY);
  

  // Serial.print("UP:");Serial.println(10000);Serial.print("DOWN:");Serial.println(-10000);
  // Serial.print("phiRate:");
  // Serial.println(phiRate);
  // Serial.print("phi_naive:");
  // Serial.println(1000*(phi-phi_prev)/UPDATE_TIME_MS);
  
  // phiRate=1000*(phi-phi_prev)/UPDATE_TIME_MS;
  // phi_prev=phi;
}


void integrateEncoders()
{
  static int16_t lastCountsLeft;
  int16_t countsLeft = encoders.getCountsLeft();
  speedLeft = (countsLeft - lastCountsLeft);
  distanceLeft += countsLeft - lastCountsLeft;
  lastCountsLeft = countsLeft;

  static int16_t lastCountsRight;
  int16_t countsRight = encoders.getCountsRight();
  speedRight = (countsRight - lastCountsRight);
  distanceRight += countsRight - lastCountsRight;
  lastCountsRight = countsRight;
  //Serial.println(distanceLeft);
}

void balanceDrive(int16_t leftSpeed, int16_t rightSpeed)
{
  driveLeft = leftSpeed;
  driveRight = rightSpeed;
}

void balanceDoDriveTicks()
{
  distanceLeft -= driveLeft;
  distanceRight -= driveRight;
  speedLeft -= driveLeft;
  speedRight -= driveRight;
}

void balanceResetEncoders()
{
  distanceLeft = 0;
  distanceRight = 0;
}

void balanceUpdateSensors()
{
  imu.read();
  read_mag_data();
  //integrateGyro();
  //integrateEncoders();
  phiRateObserver();
  complemantaryAlgorithm();
  //compute_heading();
}

void balanceUpdate()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();
  static uint8_t count = 0;

  // Perform the balance updates at 100 Hz.
  if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  balanceUpdateSensors();
  balanceDoDriveTicks();

  if (isBalancingStatus)
  {
    //balance();
    ComputeOdom();
    BalanceSS();
    if (i>50){
      
      setPoint_s= 0;
      i=0;
    }
    else{
      i++;
    }
    // Stop trying to balance if we have been farther from
    // vertical than STOP_BALANCING_ANGLE for 5 counts.
    if (abs(angle) > STOP_BALANCING_ANGLE)
    {
      if (++count > 5)
      {
        isBalancingStatus = false;
        count = 0;
      }
    }
    else
    {
      count = 0;
    }
  }
  else
  {
    lyingDown();

    // Start trying to balance if we have been closer to
    // vertical than START_BALANCING_ANGLE for 5 counts.
    if (abs(angle) < START_BALANCING_ANGLE)
    {
      if (++count > 5)
      {
        isBalancingStatus = true;
        count = 0;
      }
    }
    else
    {
      count = 0;
    }
  }
}


//everything vector related
void vector_cross(vector<int16_t> *a,vector<int16_t> *b,vector<int16_t> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}
float vector_dot(vector<int16_t> *a, vector<int16_t> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}
void vector_normalize(vector<int16_t> *a)
{ 
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}


//everything compass related
void read_mag_data()
{
  static int32_t x, y, z;
  mag.read();
  x = (int32_t) mag.m.x - B[0];
  y = (int32_t) mag.m.y - B[1];
  z = (int32_t) mag.m.z - B[2];
  compass.x = (Ainv[0][0] * x + Ainv[0][1] * y + Ainv[0][2] * z)/1000;
  compass.y = (Ainv[1][0] * x + Ainv[1][1] * y + Ainv[1][2] * z)/1000;
  compass.z = (Ainv[2][0] * x + Ainv[2][1] * y + Ainv[2][2] * z)/1000;
};

void compute_heading()
{ 
  // int32_t Zl=-sin(thetaCompl/1000)*compass.x+cos(thetaCompl/1000)*compass.z;
  heading = 1000*180.*atan2(compass.y, compass.z) / PI;
  if (heading < 0) heading = heading + 360000;
  //heading=heading(compass);
  Serial.println(heading);
};
//https://github.com/pololu/lsm303-arduino/blob/master/LSM303.h
// float compute_heading(vector<int16_t> from)
// {
//   vector<int16_t> temp_m = {from.x, from.y, from.z};

//   // subtract offset (average of min and max) from magnetometer readings
//   // temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
//   // temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
//   // temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

//   // compute E and N
//   vector<int16_t> E;
//   vector<int16_t> N;
//   vector<int16_t> a={imu.a.x, imu.a.y, imu.a.z};

//   vector_cross(&temp_m, &a, &E);//vector_cross(&from, &a, &E);
//   vector_normalize(&E);
//   vector_cross(&a, &E, &N);
//   // Serial.print(E.x);
//   // Serial.print("  | ");
//   // Serial.print(E.y);
//   // Serial.print("  | ");
//   // Serial.println(E.z);

//   //vector_normalize(&N);

//   // compute heading
//   float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / PI;
//   if (heading < 0) heading += 360;
//   //Serial.println(heading);
//   return heading;
// }
void Drive(){
  uint16_t leftSpeed, rightSpeed;
  
  if (rot<setPointA){
    leftSpeed=-5;
    rightSpeed=5;
    rot++;
  }
  else if(rot>setPointA){
    leftSpeed=5;
    rightSpeed=-5;
    rot--;
  }
  if(pos==setPoint && rot==setPointA){
    leftSpeed=0;
    rightSpeed=0;
  }
  balanceDrive(leftSpeed, rightSpeed);
}

//this function reads the serial without interfering to long with the main code (basically one byte per loop)
void readData(){
  static char message[MAX_MESSAGE_LENGTH];  
  static unsigned int message_pos = 0;
  //String data = Serial.readStringUntil('\n');
  //Read the next available byte in the serial receive buffer
  char inByte = Serial.read();

   //Message coming in (check not terminating character) and guard for over message size
  if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
  {
    //Add the incoming byte to our message
    message[message_pos] = inByte;
    message_pos++;
  }
  //Full message received...
  else
  {
    //Add null character to string
    message[message_pos] = '\0';

    //Print the message (or do other things)
    //Serial.println(message);
    treatCommand(message);
    //Reset for the next message
    message_pos = 0;
  }
}


