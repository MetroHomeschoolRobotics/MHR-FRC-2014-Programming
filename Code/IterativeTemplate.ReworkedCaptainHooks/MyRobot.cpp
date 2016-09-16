#include "WPILib.h"
#include "math.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 

/******************************************\
| Captain Hooks Single-Player Controls     |
| X-Box 360 Controller                     |
| Left Joystick: Strafe                    |
| LT/RT: Spin                              |
| Y/A: Raise/Lower Shooter                 |
| X/B: Extend/Retract Shooter Arm          |
| RB: Power Shooter Wheel                  |
\******************************************/

class RobotDemo : public IterativeRobot
{
	Victor *FL, *RL, *FR, *RR;
	RobotDrive *myRobot; // robot drive system
	Victor *shooterWheel;
	Jaguar *shooterFlipper;
	Jaguar *lift;
	Compressor *compressor;
	Solenoid *hooksOne;
	Solenoid *hooksTwo;
	SolenoidBase *hookstwo;
	Joystick *stick; // only joystick
	
	double joyX, joyY, joyZ, threshold;

public:
	RobotDemo()
	{
		FL = new Victor(1);
		RL = new Victor(3);
		FR = new Victor(2);
		RR = new Victor(4);
		myRobot = new RobotDrive(FL, RL, FR, RR);
		myRobot->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		myRobot->SetExpiration(0.1);
		
		shooterWheel = new Victor(5);
		shooterFlipper = new Jaguar(7);
		lift = new Jaguar(6);
		compressor = new Compressor(9, 2);
		hooksOne = new Solenoid(1);
		hooksTwo = new Solenoid(2);
		
		joyX = 0; 
		joyY = 0; 
		joyZ = 0;
		threshold = 0.15;
		
		stick = new Joystick(1);
		
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
	shooterWheel->Set(0);
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
	joyX = stick->GetX();
	if(fabs(joyX) < threshold)
		joyX = 0;
	joyY = stick->GetY();
	if(fabs(joyY) < threshold)
		joyY = 0;
	joyZ = stick->GetRawAxis(3);
	if(fabs(joyZ) < threshold)
		joyZ = 0;

	joyX = pow(joyX, 3.0);
	joyY = pow(joyY, 3.0);
	joyZ = pow(joyZ, 3.0);
	//myRobot.ArcadeDrive(stick->GetRawAxis(1), stick->GetRawAxis(2), stick->GetRawAxis(3), 0); // drive with arcade style
	myRobot->MecanumDrive_Cartesian(joyX, joyY, -joyZ, 0);
	
	if(stick->GetRawButton(6)) {
		shooterWheel->Set(-1);
	} else {
		shooterWheel->Set(0);
	}
	
	if(stick->GetRawButton(3)) {
		shooterFlipper->Set(0.8);
	} else if(stick->GetRawButton(2)) {
		shooterFlipper->Set(-0.8);
	} else {
		shooterFlipper->Set(0);
	}
	
	if(stick->GetRawButton(1)) {
		lift->Set(0.8);
	} else if(stick->GetRawButton(4)) {
		lift->Set(-0.8);
	} else {
		lift->Set(0);
	}
}

/**
 * Initialization code for test mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters test mode.
 */
void RobotDemo::TestInit() {
}

/**
 * Periodic code for test mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in test mode.
 */
void RobotDemo::TestPeriodic() {
}

};

START_ROBOT_CLASS(RobotDemo);

