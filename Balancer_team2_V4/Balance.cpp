#include <Wire.h>
#include <LIS3MDL.h>
#include "Balance.h"


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


const int32_t K=1000*0.01/1;// for complementory algorithm
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
int32_t x; //position in x 10^-5 m
int32_t y; //position in y 10^-5 m
int32_t yaw; //yaw angle microrad


void sendOdometry(){//sending odometry data with format 'x;y;yaw\n'
  Serial.print(x/1000); //x in cm
  Serial.print(";");
  Serial.print(y/1000); //y in cm
  Serial.print(";");
  Serial.print(yaw/1000);//yaw in millirad
  Serial.print("\n");
}


void treatCommand(char message[MAX_MESSAGE_LENGTH]){//function to perfomr actions depending on the message from the serial com

  if(String(message)=="DF"){
    setPoint_s=-700000;
  }
  else if(String(message)=="DB"){
    setPoint_s=700000;
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
// Balboa 32U4 robot using a state space model with feedback.
void BalanceSS(){


  int32_t dutycycle;

  //POSITION CONTROL
  
  // dutycycle=(-Kssp[0]*thetaCompl-Kssp[1]*thetaRate-Kssp[2]*phi-Kssp[3]*phiRate+nr*setPoint)/1000;
  // motorSpeed=0.5*dutycycle*MOTOR_SPEED_LIMIT/1000;
  // // SATURATION
  // if (motorSpeed > MOTOR_SPEED_LIMIT)
  // {
  //   motorSpeed = MOTOR_SPEED_LIMIT;
  // }
  // if (motorSpeed < -MOTOR_SPEED_LIMIT)
  // {
  //   motorSpeed = -MOTOR_SPEED_LIMIT;
  // }

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

  dutycycle=(-Kssp_s[0]*thetaCompl-Kssp_s[1]*thetaRate-Kssp_s[2]*phiRate+nr_s*setPoint_s/1000)/1000;
  motorSpeed=0.5*dutycycle*MOTOR_SPEED_LIMIT/1000;
  //SATURATION
  if (motorSpeed > MOTOR_SPEED_LIMIT)
  {
    motorSpeed = MOTOR_SPEED_LIMIT;
  }
  if (motorSpeed < -MOTOR_SPEED_LIMIT)
  {
    motorSpeed = -MOTOR_SPEED_LIMIT;
  }

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

void complemantaryAlgorithm()//function to do sensor fusion to get a better angle measurement
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

void phiRateObserver()//reduced observer algorithm. it works but makes to system less stable, 
//so we prefer to use phi rate as computed at the end for performance
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
  
  phiRate=1000*(phi-phi_prev)/UPDATE_TIME_MS;
  phi_prev=phi;
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
  phiRateObserver();
  complemantaryAlgorithm();
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



void Drive(){//function to rotate the balboa around, basically its a simple proportional controller
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
    treatCommand(message);
    //Reset for the next message
    message_pos = 0;
  }
}


