#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Right1               motor         14              
// Right2               motor         18              
// Left1                motor         21              
// Left2                motor         13              
// bob                  motor         8               
// comer                motor         11              
// lift                 motor         3               
// Controller1          controller                    
// LimitSwitchD         limit         D               
// AutonOtherOne        limit         F               
// ---- END VEXCODE CONFIGURED DEVICES ----
using namespace vex;
competition Competition;


// class containing all the information of the PID
class PID{
//private area of the class, makes it so the user can't directly acess and change the variable on accident
private:
//defult tuning values;
double left_kP = 0.6;
double left_kI = 0.0;
double left_kD = 0.0;
double right_kP = 0.6;
double right_kI = 0.0;
double right_kD = 0.0;

//information that is directly able to be accessed by the user, it is meant to be changed often
public:

//functions to change the values of the information that can't be directly changed
void setLeft_kP(double value){ left_kP = value; }
void setRight_kP(double value){ right_kP = value; }
void setLeft_kI(double value){ left_kI = value; }
void setRight_kI(double value){ right_kI = value; }
void setLeft_kD(double value){ left_kD = value; }
void setRight_kD(double value){ right_kD = value; }

//reset function to reset all the motor values, so it doesn't go off of its previous position
void reset(){
Left1.setPosition(0, deg);
Left2.setPosition(0, deg);
Right1.setPosition(0, deg);
Right2.setPosition(0, deg);
}

//function for the PID on the left side of the drive train
void leftPID(double target){
//definition of the information needed to calculate the robot speed
double currentPosition = 0;
double error = 0; // error is the distance between the current position and the target possition (distance left)
double totalError = 0;
double prevError = 0;
double derivative;
//variable to stop the loop once it reaches the desired target
bool metTarget = true;
//for loop that calculates the robot speed, and makes the robot move
while (metTarget){
currentPosition = (Left1.position(deg)+Left2.position(deg))/2;
error = target-currentPosition;
totalError = error+totalError;
derivative = error-prevError;
double motorPower = (error * left_kP + derivative * left_kD + totalError * left_kI);
Left1.spin(forward, motorPower, voltageUnits::volt);
Left2.spin(forward, motorPower, voltageUnits::volt);
if(fabs(error) < 10){
  Left1.stop();
  Left2.stop();
  metTarget = false;
  break;
}
prevError = error;
}
}

//function for the PID on the right side of the drive train
void rightPID(double target){
//definition of the information needed to calculate the robot speed
double currentPosition = 0;
double error = 0; 
double totalError = 0;
double prevError = 0;
double derivative;
//variable to stop the loop once it reaches the desired target
bool metTarget = true;
//for loop that calculates the robot speed, and makes the robot move
while (metTarget){
currentPosition = (Right1.position(deg)+Right2.position(deg))/2;
error = target-currentPosition;
totalError = error+totalError;
derivative = error-prevError;
double motorPower = (error * right_kP + derivative * right_kD + totalError * right_kI);
Right1.spin(forward, motorPower, voltageUnits::volt);
Right2.spin(forward, motorPower, voltageUnits::volt);
if(fabs(error) < 10){
  Right1.stop();
  Right2.stop();
  metTarget = false;
  break;
}
prevError = error;
}
}

//function for both sides it functions the same as all other functions
void bothPID(double target){
//definition of the information needed to calculate the robot speed
double currentPosition = 0;
double error = 0; 
double totalError = 0;
double prevError = 0;
double derivative;
//variable to stop the loop once it reaches the desired target
bool metTarget = true;
//for loop that calculates the robot speed, and makes the robot move
while (metTarget){
currentPosition = (Right1.position(deg)+Right2.position(deg)+Left2.position(deg)+Left1.position(deg))/4;
error = target-currentPosition;
totalError = error+totalError;
derivative = error-prevError;
double motorLeftPower = (error * left_kP + derivative * left_kD + totalError * left_kI);
double motorRightPower = (error * right_kP + derivative * right_kD + totalError * right_kI);
Left1.spin(forward, motorLeftPower, voltageUnits::volt);
Left2.spin(forward, motorLeftPower, voltageUnits::volt);
Right1.spin(forward, motorRightPower, voltageUnits::volt);
Right2.spin(forward, motorRightPower, voltageUnits::volt);
if(fabs(error) < 10){
  Left1.stop();
  Left2.stop();
  Right1.stop();
  Right2.stop();
  metTarget = false;
  return;
  break;
}
prevError = error;
}
}

//function to enable the robot to turn using PID
void turnPID(double target){
//definition of the information needed to calculate the robot speed
double currentPosition = 0;
double error = 0; 
double totalError = 0;
double prevError = 0;
double derivative;
//variable to stop the loop once it reaches the desired target
bool metTarget = true;
//for loop that calculates the robot speed, and makes the robot move
while (metTarget){
currentPosition = (0-Right1.position(deg)-Right2.position(deg)+Left2.position(deg)+Left1.position(deg))/4;
error = target-currentPosition;
totalError = error+totalError;
derivative = error-prevError;
double motorLeftPower = (error * left_kP + derivative * left_kD + totalError * left_kI);
double motorRightPower = (error * right_kP + derivative * right_kD + totalError * right_kI);
Left1.spin(forward, motorLeftPower, voltageUnits::volt);
Left2.spin(forward, motorLeftPower, voltageUnits::volt);
Right1.spin(reverse, motorRightPower, voltageUnits::volt);
Right2.spin(reverse, motorRightPower, voltageUnits::volt);
if(fabs(error) < 10){
  Left1.stop();
  Left2.stop();
  Right1.stop();
  Right1.stop();
  metTarget = false;
  break;
}
prevError = error;
}
}
};

void pre_auton(void) {
  //resets motor positions
Left1.setPosition(0, deg);
Left2.setPosition(0, deg);
Right1.setPosition(0, deg);
Right2.setPosition(0, deg);
autonSelecter = autonStuff();
}

void autonomous(void) {
//gives the class an object
PID s;
//ex : s.bothPID(100);
}

void usercontrol(void) {
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
