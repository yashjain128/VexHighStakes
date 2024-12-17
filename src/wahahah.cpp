#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS

// Robot configuration code.
motor FL = motor(PORT12, ratio6_1, true);

motor TL = motor(PORT11, ratio6_1, false);

motor BL = motor(PORT13, ratio6_1, true);

motor FR = motor(PORT16, ratio6_1, false);

motor TR = motor(PORT15, ratio6_1, true);

motor BR = motor(PORT18, ratio6_1, false);

inertial InertialSensor = inertial(PORT4);

rotation XOdom = rotation(PORT20, false);

rotation YOdom = rotation(PORT2, false);

controller Controller1 = controller(primary);
motor IntakeMotor = motor(PORT3, ratio18_1, false);

digital_out Clamp = digital_out(Brain.ThreeWirePort.C);
digital_out Doinker = digital_out(Brain.ThreeWirePort.A);

// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}


void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}

// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}


// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       { vnauthor}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

motor_group leftMotors = motor_group(FL, TL, BL);
motor_group rightMotors = motor_group(FR, TR, BR);

//circumference of the odometry wheels in metres
const double CIRC = 0.21944;

competition Competition;

double speed = 0.9;
bool toggleDrive = false;

//Odom test
double theta = 0;

const double XOdomDisFromWall = 0;
const double YOdomDisFromWall = 0;
const double xOdomOffset = 0;
const double yOdomOffset = 0;
const double xCenter = XOdomDisFromWall + xOdomOffset;
const double yCenter = YOdomDisFromWall - yOdomOffset;

//double posX = xCenter, posY = yCenter;
double posX = 0, posY = 0;
const double D_PERP = -0.041275;
const double D_PARA = -0.0762;


const double WHEEL_BASE = 0.3429;

//configure your robot 

const double ONE_MOTOR_TURN = 0.1556; 

const double ONE_ENCODER_TURN = 0.217; 

const int MAX_RPM = 550; 

const double SETTLING_TIME = 0.1;//100ms 
const double GOAL_DISTANCE = 0.1; 
const double GOAL_ANGLE = 0.03;//about half a degree 

//differential drive stuff

double prevX = 0, prevY = 0; 
double currX = 0, currY = 0; 
double prevTheta = 0; 
double xPos = 0; 
double yPos = 0; 

 

void updatePosition()//odometry function 

{ 
    prevX = currX; 
    currX = XOdom.position(turns); 
    prevY = currY; 
    currY = YOdom.position(turns); 

    double ex = ONE_ENCODER_TURN * (currX - prevX); 
    double ey = ONE_ENCODER_TURN * (currY - prevY);     
    double direction = -InertialSensor.rotation(degrees)*M_PI/180; 
    double dTheta = direction - prevTheta; 
    double dx = ex; 
    double dy = ey; 

    if(fabs(dTheta) > 0.002)//0.1 degrees reads fairly consistent => 0.001745... 
    { 
        double hx = 2*(ex/dTheta - ONE_ENCODER_TURN/2)*sin(dTheta/2); 
        double hy = 2*(ey/dTheta + ONE_ENCODER_TURN/2)*sin(dTheta/2); 
        dx = -hy*sin(dTheta/2) + hx*cos(dTheta/2); 
        dy = hy*cos(dTheta/2) + hx*sin(dTheta/2); 
    } 

    xPos += -dy*sin(prevTheta) + dx*cos(prevTheta); 
    yPos += dy*cos(prevTheta) + dx*sin(prevTheta); 
    prevTheta = direction; 
}

double bestCorrection(double headingError) 

{ 

  const double ONE_TURN = 2*M_PI;//360 degrees is 2PI in the radians measurement system 

  int extraTurns = (int)(headingError / ONE_TURN); 

  headingError = headingError - extraTurns * ONE_TURN;//reduced to range -360 to 360 

  double alternativeTurn = ONE_TURN + headingError;//assume right turn 

  if(headingError > 0)//left turn - update alternative value 

    alternativeTurn = headingError - ONE_TURN; 

   

  if( fabs(alternativeTurn) < fabs(headingError) )//choose smallest magnitude 

    return alternativeTurn; 

  else 

    return headingError; 

} 

/**void moveTo(double xG, double yG, double PATH_RATE) 

{ 
    double dx = xG - xPos; 

    double dy = yG - yPos; 

    double thetaI = 0;
    thetaI = InertialSensor.rotation(degrees)*M_PI/180; 

    double dTheta = 2*(atan2(dx, dy) - thetaI); 

    dTheta = bestCorrection( dTheta ); 

    double d = sqrt(dx*dx + dy*dy); 

     

    while(d > GOAL_DISTANCE) 

    { 

      //assume there is no arc and travel at the same rate on both wheels 

      double vLeft = d/PATH_RATE; 

      double vRight = vLeft; 

      thetaI = InertialSensor.rotation(degrees)*M_PI/180; 

      dTheta = bestCorrection(2*(atan2(dx, dy) - thetaI));

      if(fabs(dTheta) > 1e-9)//arc exists 

      { 

        double rCenter = d/(2*sin(dTheta/2)); 

        vLeft = (rCenter - WHEEL_BASE/2)*dTheta/PATH_RATE; 

        vRight = (rCenter + WHEEL_BASE/2)*dTheta/PATH_RATE; 

      }

 

      leftMotors.setVelocity(vLeft * MAX_RPM, rpm); 

      rightMotors.setVelocity(vRight * MAX_RPM, rpm); 

      FL.spin(forward);
      BL.spin(forward);
      TL.spin(forward);
      FR.spin(forward);
      BR.spin(forward);
      TR.spin(forward);

      //leftMotors.spin(forward); 
      //rightMotors.spin(forward); 

       

      wait(SETTLING_TIME,seconds);       

      updatePosition(); 

      dx = xG - xPos; 

      dy = yG - yPos; 

      d = sqrt(dx*dx + dy*dy);
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);  
      Controller1.Screen.print("l = %.4f, r= %.4f",vLeft, vRight); 
      //Controller1.Screen.setCursor(2, 1);
      //Controller1.Screen.print("thetaI = %.4f", fabs(dTheta));

    } 

} */

void moveTo(double xG, double yG, double howFast) 
{ 
    double dx = xPos - xG; 
    double dy = yG - yPos; 
    double thetaI = -InertialSensor.rotation(degrees)*M_PI/180; 
    double dTheta = 2*(atan2(dx, dy) - thetaI); 
    dTheta = bestCorrection( dTheta ); 
    double d = sqrt(dx*dx + dy*dy); 

    while(d > GOAL_DISTANCE) 
    { 
      //assume there is no arc and travel at the same rate on both wheels 
      double vLeft = howFast; 
      double vRight = howFast;
      double tCenter = d/howFast; 
      if(fabs(dTheta) > 1e-7)//arc exists 
      { 
        double rCenter = d/(2*sin(dTheta/2));//radius of the robot's center 
        double dCenter = rCenter*dTheta;//distance the center of the robot will travel a = r*theta 
        tCenter = dCenter / howFast;//t = d/v 
        //the left and right wheel travel different distances on an arc but over the same time as the center 
        vLeft = (rCenter - WHEEL_BASE/2)*dTheta / tCenter; 
        vRight = (rCenter + WHEEL_BASE/2)*dTheta / tCenter; 
      } 

      leftMotors.setVelocity(vLeft * 60/ONE_MOTOR_TURN, rpm);
      rightMotors.setVelocity(vRight * 60/ONE_MOTOR_TURN, rpm);
      leftMotors.spin(forward); 
      rightMotors.spin(forward);
      if(tCenter < SETTLING_TIME)//don't wait longer than necessary 
        wait(tCenter, seconds); 
      else 
        wait(SETTLING_TIME, seconds); 

      updatePosition(); 
      dx = xPos - xG; 
      dy = yG - yPos; 

      thetaI = -InertialSensor.rotation(degrees)*M_PI/180; 

      dTheta = 2*(atan2(dx, dy) - thetaI); 
      dTheta = bestCorrection( dTheta ); 

      d = sqrt(dx*dx + dy*dy); 

      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);  
      Controller1.Screen.print("l = %.4f, r= %.4f",vLeft, vRight); 
      Controller1.Screen.setCursor(2,1);  
      Controller1.Screen.print("d=%.4f",d); 
      Controller1.Screen.setCursor(3,1);  
    } 
    leftMotors.stop(); 
    rightMotors.stop();
} 

void pdTurn(double amount, double p, double d)
{
  double goal = InertialSensor.rotation(degrees) + amount;
  double currError = amount;
  double errorChange = 0;//no change in error yet so no d term yet
  double prevPower = 0;

  const double MAX_ERROR = 0.25;//for a successful turn
  const double MAX_RATE = 10;//turn has slowed down to avoid overshoot
  const double MIN_RESPONSE = 1;
  const double MAX_CHANGE = 4;
  const double RESPONSE_TIME = 0.01;//10ms
  while(fabs(currError) > MAX_ERROR || fabs(InertialSensor.gyroRate(zaxis,dps)) > MAX_RATE)
  {
    errorChange = InertialSensor.gyroRate(zaxis,dps);

    double turnPower = p * currError + d * errorChange;
    if(fabs(turnPower) < MIN_RESPONSE)
    {
      if(turnPower > 0)
        turnPower = MIN_RESPONSE;
      else
        turnPower = -MIN_RESPONSE;
    }
    
    if(fabs(turnPower - prevPower) > MAX_CHANGE)//too much response 
    {
      if(turnPower > prevPower)
        turnPower = prevPower + MAX_CHANGE;
      else
        turnPower = prevPower - MAX_CHANGE;
    }
    prevPower = turnPower;//save current turnPower for next attempt    

    leftMotors.setVelocity(turnPower, percent);
    rightMotors.setVelocity(-turnPower, percent);

    leftMotors.spin(forward);
    rightMotors.spin(forward);

    wait(RESPONSE_TIME, seconds);
    currError = goal - InertialSensor.rotation(degrees);
  }
 
  leftMotors.stop();
  rightMotors.stop(); 
}

double P = 0.555;
double D = 0.013;

int main()
{
    InertialSensor.calibrate(); 
    while(InertialSensor.isCalibrating()){
      wait(0.01,seconds);    
    }

    XOdom.setPosition(0, turns);     
    YOdom.setPosition(0, turns); 
    double time1 = 2;
    /*while(Brain.timer(seconds) < 15) {
      updatePosition();
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1); 
      Controller1.Screen.print("x = %.4f, y= %.4f",xPos, yPos);
      wait(250, msec);
  
    }*/
    moveTo(0, 2, 1.0);
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1); 
    Brain.Screen.print("Location: (%.4f,%.4f)", xPos, yPos); 

}