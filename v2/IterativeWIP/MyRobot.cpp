#include "WPILib.h"
#include "math.h"
/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class RobotDemo : public IterativeRobot
{
private:
	//SpeedController* LF, LR, RF, RR;
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	//Talon* shooter;
	double joyX;
	double joyY;
	double joyZ;
	double rotation;
	double threshold;
	double shooterSpeed;


public:
	RobotDemo():
		
		//LF(1), LR(2), FF(3),  FR(4), 
		//myRobot(LF, LR, FF, FR),	// these must be initialized in the same order
		myRobot(new Talon(1), new Talon(2), new Talon(3), new Talon(4)),
		stick(1)//,		// as they are declared above.
		//shooter(5)
		
	{
		//LF.SetRaw(0);
		//LR.SetRaw(0);
		//FF.SetRaw(0);
		//FR.SetRaw(0);
		joyX = 0;
		joyY = 0;
		joyZ = 0;
		rotation = 0;
		threshold = 0.15;
		//shooter = new Talon(5);
		
	
		myRobot.SetExpiration(0.1);
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}

/**
 * Robot-wide initialization code should go here.
 * 
 * Use this method for default Robot-wide initialization which will
 * be called when the robot is first powered on.  It will be called exactly 1 time.
 */
void RobotDemo::RobotInit() {
}

/**
 * Initialization code for disabled mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters disabled mode. 
 */
void RobotDemo::DisabledInit() {
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void RobotDemo::DisabledPeriodic() {
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */
void RobotDemo::AutonomousInit() {
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RobotDemo::AutonomousPeriodic() {
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void RobotDemo::TeleopInit() {
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RobotDemo::TeleopPeriodic() {
	myRobot.SetSafetyEnabled(true);
	
		joyX = -stick.GetX();
		if(abs((int)(joyX*100)) < threshold*100)
			joyX = 0;
		joyY = -stick.GetY();
		if(abs((int)(joyY*100)) < threshold*100)
			joyY = 0;//-0.02;
/*		if(abs((int)(joyY*100)) > 100){
			if(joyY > 0)
				joyY = 0.5;
			else 
				joyY = -1;
		}*/
		joyZ = -stick.GetRawAxis(3);
		if(abs((int)(joyZ*100)) < threshold*100)
			joyZ = 0;
/*		if(sqrt(pow((int)(stick.GetRawAxis(4)*100)) + pow((int)(stick.GetRawAxis(5)*100))) < 70)
			rotation = ((atan2(stick.GetRawAxis(4), -stick.GetRawAxis(5))/PI)*360)+180;*/

		joyX = pow(joyX, 3.0);
		joyY = pow((joyY), 3.0);
		joyZ = pow(joyZ, 3.0);//*0.65;
		myRobot.MecanumDrive_Cartesian(joyX, joyY, -joyZ, rotation); // drive with mechanum style :D (use right stick)
		
		/*
		if(stick.GetRawButton(1)){
			shooter.Set(1);
		} else if(stick.GetRawButton(2)) {
			shooter.Set(-1);
		} else {
			if(abs((int)(stick.GetRawAxis(5)*100) > 15))
				shooter.Set(pow(stick.GetRawAxis(5), 3));
			else
				shooter.Set(0);
		}*/
		
}
	
/**
 * Initialization code for test mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters test mode.
 */
/*void RobotDemo::TestInit() {
}

/**
 * Periodic code for test mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in test mode.
 *
void RobotDemo::TestPeriodic() {
}

};
*/
};
START_ROBOT_CLASS(RobotDemo);

