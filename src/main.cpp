/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Authors:      str(theEnd) = null, JustGaming8                           */
/*    Owner: str(theEnd) = null                                               */
/*    Created:      9/14/2025, 12:43:44 PM                                    */
/*    Description:  7700N Noodlebot's V5 Pushback Code                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// MARK: Includes
#include "vex.h"

using namespace vex;
float wheelDiamiter = 2.75;
float gearRatioExternal = 1;
float pi = 3.141;
// A global instance of competition
competition Competition;

// Global Definitions 

brain Brain;
controller Controller1;
// FOR PISTON: digital_out  = digital_out(Brain.ThreeWirePort.(port number));

// MARK: --
// MARK: vv Porting vv

int autonSelected = 0;
int autonMin = 0;
int autonMax = 4;

// MARK: Drivetrain
/* Motor 4 */ motor leftRear = motor(PORT13, ratio18_1, true);
/* Motor 9 */motor leftMiddle = motor(PORT11, ratio18_1, true);
/* Motor 10 */ motor leftFront = motor(PORT1, ratio18_1, true);
/* Motor 7 */ motor rightRear = motor(PORT4, ratio18_1, false);
/* Motor 11 */ motor rightMiddle = motor(PORT3, ratio18_1, false);
/* Motor 8 */motor rightFront = motor(PORT2, ratio18_1, false);

// MARK:  Intake + Conveyer 

motor intakePlusBottomConveyer = motor(PORT10, ratio18_1, true);
motor topConveyer = motor(PORT18, ratio18_1, true);
digital_out scoringRamp = digital_out(Brain.ThreeWirePort.B);

// MARK: Unloader

digital_out unloader = digital_out(Brain.ThreeWirePort.C);

// MARK: Inertial Sensor

inertial InertialSensor = inertial(PORT12);
 


//MARK: --
// MARK: vv Functions vv
float speed(0);
bool speedIsLocked = false;
const double kp = 0.00625;
const double ki = 0.000015; 
const double turnkp = 0.0375;
void driveTrainStop() {
  leftFront.stop();
  leftMiddle.stop();
  leftRear.stop();
  rightFront.stop();
  rightMiddle.stop();
  rightRear.stop();
}


void setBrakeMode(brakeType mode) {
  leftFront.setStopping(mode);
  leftMiddle.setStopping(mode);
  leftRear.setStopping(mode);
  rightFront.setStopping(mode);
  rightMiddle.setStopping(mode);
  rightRear.setStopping(mode);
}

// MARK: Drivetrain
void driveVolts(int lSpeed, int rSpeed, int waitTime) {
  lSpeed = lSpeed * 12.00;
  rSpeed = rSpeed * 12.00;
  leftFront.spin(fwd, lSpeed, volt);
  leftMiddle.spin(fwd, lSpeed, volt);
  leftRear.spin(fwd, lSpeed, volt);
  rightFront.spin(fwd, rSpeed, volt);
  rightMiddle.spin(fwd, rSpeed, volt);
  rightRear.spin(fwd, rSpeed, volt);
  wait(waitTime, msec);
}



// MARK: Auton

void inchDrive(double inches) {
  int debugCounter = 0;
  double currPos = leftFront.position(degrees);
  double startPos = currPos;
  double targetPos = currPos + (inches * 360) / (pi * wheelDiamiter);
  double error = targetPos - currPos;
  double acc_error = 0;

  while (fabs(error) > 40) {
    currPos = leftFront.position(degrees);
    error = targetPos - currPos;
    if (fabs(error) < 100) {
      acc_error += error;
    } else {
      acc_error = 0;
    }
    speed = error * kp + acc_error * ki;
    driveVolts(speed, speed, 10);
    if (debugCounter ++ % 20 == 0) {
      // DEBUG
     printf("inchDrive %0.2f error: %0.2f [%0.2f]  speed: %0.2f\n", inches, (leftFront.position(degrees) - targetPos) * (pi * wheelDiamiter) / 360, error, speed);
    }
  }
  driveTrainStop();
  printf("inchDrive %0.2f Error: %0.2f\n", inches, (leftFront.position(degrees) - startPos) * (pi * wheelDiamiter) / 360);
}

void autonTurn(double degrees) {
// FOR TURINING RIGHT, USE POSITIVE
 int debugCounter = 0;
 double currRotation = InertialSensor.rotation(deg);
 double targetRotation = currRotation + degrees;
 double startRotation = currRotation;
 double error = targetRotation - currRotation;

 while (fabs(error) > 1) {
  currRotation = InertialSensor.rotation(deg);
  error = targetRotation - currRotation;
  speed = error * turnkp;
  driveVolts(speed, -speed, 10);

  if (debugCounter ++ % 20 == 0) {
    printf("autonTurn %0.2f error: %0.2f [%0.2f]  speed: %0.2f\n", degrees, InertialSensor.rotation(deg) - targetRotation, error, speed);
  }
 }
  driveTrainStop();
  printf("autonTurn finished! autonTurn: %0.2f speed: %0.2f\n", degrees, InertialSensor.rotation(deg) - startRotation, speed);
}

// MARK: Motor Monitor

double YOFFSET = 20; //offset for the display

//Writes a line for the diagnostics of a motor on the Brain (Motor Monitor)
void MotorDisplay(double y, double curr, double temp)
{
	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
	
	if (curr < 1){
		Brain.Screen.setFillColor(green);
	    Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	} else if(curr >= 1 && curr  <= 2.5) {
		Brain.Screen.setFillColor(yellow);
	    Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	} else {
		Brain.Screen.setFillColor(red);
		Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	}

	
	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);

	// Brain rectangle colors for motor temperature
	if (temp < 45){
		Brain.Screen.setFillColor(green);
	} else if(temp <= 50 && temp  >= 45){
		Brain.Screen.setFillColor(yellow);
	} else {
		Brain.Screen.setFillColor(red);
	}
	Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
	Brain.Screen.setFillColor(transparent);
}


// Displays information on the brain
void Display()
{
	double leftFrontCurr = leftFront.current(amp);
	double leftFrontTemp = leftFront.temperature(celsius);
	double leftBackCurr = leftRear.current(amp);
	double leftBackTemp = leftRear.temperature(celsius);
  double leftMiddleTemp = leftMiddle.temperature(celsius);
  double leftMiddleCurr = leftMiddle.current(amp);
  double rightMiddleTemp = rightMiddle.temperature(celsius);
  double rightMiddleCurr = rightMiddle.current(amp);
	double rightFrontCurr = rightFront.current(amp);
	double rightFrontTemp = rightFront.temperature(celsius);
	double rightBackCurr = rightRear.current(amp);
	double rightBackTemp = rightRear.temperature(celsius);
  double intakeCurr = intakePlusBottomConveyer.current(amp);
	double intakeTemp = intakePlusBottomConveyer.temperature(celsius);
  double intakeConveyorCurr = topConveyer.current(amp);
  double intakeConveyorTemp = topConveyer.temperature(celsius);

	if (leftFront.installed()){
		MotorDisplay(1, leftFrontCurr, leftFrontTemp);
		Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront - DISCONNECTED");
	}
	
	if (leftMiddle.installed()) {
    MotorDisplay(16, leftMiddleCurr, leftMiddleTemp);
    Brain.Screen.printAt(300, YOFFSET + 16, "LeftMiddle");
  } else {
    Brain.Screen.printAt(5, YOFFSET + 16, "LeftMiddle - DISCONNECTED");
  }

	if (leftRear.installed()){
		MotorDisplay(31, leftBackCurr, leftBackTemp);
		Brain.Screen.printAt(300, YOFFSET + 31, "LeftBack");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 31, "LeftBack - DISCONNECTED");
  }

	if (rightFront.installed()) {
		MotorDisplay(46, rightFrontCurr, rightFrontTemp);
		Brain.Screen.printAt(300, YOFFSET + 46, "RightFront");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 46, "RightFront - DISCONNECTED");
	}

  if (rightMiddle.installed()) {
    MotorDisplay(61, rightMiddleCurr, rightMiddleTemp);
    Brain.Screen.printAt(300, YOFFSET + 61, "RightMiddle");
  } else {
    Brain.Screen.printAt(5, YOFFSET + 61, "RightMiddle - DISCONNECTED");
  }

  if (rightRear.installed()) {
    MotorDisplay(76, rightBackCurr, rightBackTemp);
    Brain.Screen.printAt(300, YOFFSET + 76, "RightRear");
  } else {
    Brain.Screen.printAt(5, YOFFSET + 76, "RightRear - DISCONNECTED");
  }
	
	if (intakePlusBottomConveyer.installed()) {
		MotorDisplay(91, intakeCurr, intakeTemp);
		Brain.Screen.printAt(300, YOFFSET + 91, "Intake");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 91, "Intake - DISCONNECTED");
	}

  if (topConveyer.installed()){
		MotorDisplay(106, intakeConveyorCurr, intakeConveyorTemp);
		Brain.Screen.printAt(300, YOFFSET + 106, "IntakeConveyor");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 106, "IntakeConveyor - DISCONNECTED");
	}
}




// MARK: Unloader

//void unloaderToggle() {
  //unloaderL.set(!unloaderL.value());
  //unloaderR.set(!unloaderR.value());
//}



// void speedLimiter(int lSpeed, int rSpeed, int maxSpeed) {

//   if(rSpeed || lSpeed < maxSpeed) {
//     rSpeed = 0;
//     lSpeed = 0;
//     Controller1.rumble("..");
//     Controller1.Screen.clearScreen();
//     Controller1.Screen.setCursor(1,1);
//     Controller1.Screen.print("Speed Locked. SLOW DOWN IDIOT!");
//     wait(2.5, sec);
//   }
// }
// MARK: --

// MARK: Pre Auton
void pre_auton(void) {

}


//MARK: Auton

void autonomous() {
 setBrakeMode(brakeType::brake);
 inchDrive(5);
//  inchDrive(-5);
//  inchDrive(10);
//  inchDrive(-10);
//  inchDrive(20);
//  inchDrive(-20);
//  inchDrive(40);
//  inchDrive(-40);
}

// MARK: --
// MARK: vv Usercontrol vv
void usercontrol(void) {

  // Drive
  while (1) {
    Display(); 
    int lStick = Controller1.Axis3.position(pct);
    int rStick = Controller1.Axis2.position(pct);
    driveVolts(rStick, lStick, 10);
    
    // MARK: Intake + Conveyer

    if (Controller1.ButtonR1.pressing()) {
      intakePlusBottomConveyer.spin(fwd, 1000, rpm);
      topConveyer.spin(fwd, 1000, rpm); 
    }

    else if (Controller1.ButtonR2.pressing()) {
      intakePlusBottomConveyer.spin(fwd, -1000, rpm);
      topConveyer.spin(fwd, -1000, rpm);
    }

    else {
      intakePlusBottomConveyer.stop();
      topConveyer.stop();
    }
    
    // MARK: Use ramp
    if (Controller1.ButtonL1.pressing()) { 
      scoringRamp.set(!scoringRamp.value());
      wait(300, msec);
    }

    // MARK:  Use Unloader

    if (Controller1.ButtonL2.pressing()) {
      unloader.set(!unloader.value());
      wait(300, msec);
    }
    
    //MARK: Top Conveyer Only

    if (Controller1.ButtonUp.pressing()) {
      topConveyer.spin(fwd, -1000, rpm); 
    }
    

  else {
    wait(10, msec);
  }

 }
   
}
// MARK: --
// MARK: Main
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
