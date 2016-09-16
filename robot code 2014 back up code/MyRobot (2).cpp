#include "WPILib.h"
#include "math.h"
#define PI 3.14159265;
/**
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
const double pi = 3.14159265;


class CyborgsHolonomicDrive {
private:
	//int onePort, twoPort, threePort, fourPort, fivePort, sixPort, sevenPort, eightPort;
	//double oneAngle, twoAngle, threeAngle, fourAngle, fiveAngle, sixAngle, sevenAngle, eightAngle;
	//double oneSpeed, twoSpeed, threeSpeed, fourSpeed, fiveSpeed, sixSpeed, sevenSpeed, eightSpeed;
	double joyAngle;
	double joyMagnitude;
	double largestPower;
	int wheelQuantity;
	int* ports;
	double* angles;
	double* speeds;
	//RobotDrive driveOne, driveTwo, driveThree, driveFour;
	RobotDrive datDrive;
	
public:
	CyborgsHolonomicDrive(int wheelOnePort, double wheelOneForceDirection, int wheelTwoPort, double wheelTwoForceDirection, int wheelThreePort, double wheelThreeForceDirection, int wheelFourPort, double wheelFourForceDirection):
	//driveOne(1, 5), driveTwo(3, 6), driveThree(2, 7), driveFour(4, 8)
	datDrive(1, 3, 2, 4)
	{
		/*// four-wheel omni/mecanum drive
		onePort = wheelOnePort;
		twoPort = wheelTwoPort;
		threePort = wheelThreePort;
		fourPort = wheelFourPort;
		//fivePort, sixPort, sevenPort, eightPort = 0;
		oneAngle = wheelOneForceDirection;
		twoAngle = wheelTwoForceDirection;
		threeAngle = wheelThreeForceDirection;
		fourAngle = wheelFourForceDirection;
		//fiveAngle, sixAngle, sevenAngle, eightAngle = 0;
		oneAngle = fabs(oneAngle);*/
		
		wheelQuantity = 4;

		ports = new int(wheelQuantity);
		ports[0] = wheelOnePort;
		ports[1] = wheelTwoPort;
		ports[2] = wheelThreePort;
		ports[3] = wheelFourPort;
		angles = new double(wheelQuantity);
		angles[0] = wheelOneForceDirection-90;
		angles[1] = wheelTwoForceDirection-90;
		angles[2] = wheelThreeForceDirection-90;
		angles[4] = wheelFourForceDirection-90;
		speeds = new double(wheelQuantity);
		for(int i = 0; i < wheelQuantity; i++)
			speeds[i] = 0;
		/*driveOne.SetSafetyEnabled(false);
		driveTwo.SetSafetyEnabled(false);
		driveThree.SetSafetyEnabled(false);
		driveFour.SetSafetyEnabled(false);*/
		datDrive.SetSafetyEnabled(false);
	}
	
	void normalizeSpeeds() {
		largestPower = 0;
		for(int i = 0; i < wheelQuantity; i++) {
			if(fabs(speeds[i]) > largestPower) {
				largestPower = fabs(speeds[i]);
			}
		}
		for(int i = 0; i < wheelQuantity; i++) {
			speeds[i] = speeds[i]/largestPower;
		}
	}
	
	void drive(double joyX, double joyY, double rotation, double gyro = 0.0) {
		joyMagnitude = sqrt(pow(joyX, 2) + pow(joyY, 2));
		joyAngle = atan2(joyY, joyX);
		
		for(int i = 0; i < wheelQuantity; i++) {
			speeds[i] = sin(pi*(angles[i]-joyAngle-gyro)/180)-rotation;
		}
		normalizeSpeeds();
		for(int i = 0; i < wheelQuantity; i++) {
			speeds[i] *= joyMagnitude;
		}
		/*driveOne.TankDrive(speeds[0], speeds[0]);
		driveTwo.TankDrive(speeds[1], speeds[1]);
		driveThree.TankDrive(speeds[2], speeds[2]);
		driveFour.TankDrive(speeds[3], speeds[3]);*/
		
	}
};


class RobotDemo : public IterativeRobot
{
private:
	//SpeedController* LF, LR, RF, RR;
	RobotDrive myRobot; // robot drive system
	//CyborgsHolonomicDrive mecanumDrive;
	//RobotDrive robotFront;
	//RobotDrive robotBack;
	Joystick stick; // only joystick
	//Talon* shooter;
	Gyro fieldOrientation;
	double joyX;
	double joyY;
	double joyZ;
	double joyAngle;
	double joyMagnitude;
	double wheelFLAngle;
	double wheelFLSpeed;
	double wheelFRAngle;
	double wheelFRSpeed;
	double wheelRLAngle;
	double wheelRLSpeed;
	double wheelRRAngle;
	double wheelRRSpeed;
	double maxSpeed;
	double rotation;
	double threshold;
	double shooterSpeed;
	double temp;


public:
	RobotDemo():
		
		//LF(1), LR(2), FF(3),  FR(4), 
		//myRobot(LF, LR, FF, FR),	// these must be initialized in the same order
		//myRobot(new Talon(1), new Talon(2), new Talon(3), new Talon(4)),
		myRobot(1, 2, 3, 4),
		//mecanumDrive(1, 225, 3, 315, 2, 135, 4, 45),
		//robotFront(1, 3),
		//robotBack(2, 4),
		stick(1),		// as they are declared above.
		//shooter(5)
		fieldOrientation(1)
		
	{		
		//LF.SetRaw(0);
		//LR.SetRaw(0);
		//FF.SetRaw(0);
		//FR.SetRaw(0);
		joyX = 0;
		joyY = 0;
		joyZ = 0;
		joyAngle = 0;
		joyMagnitude = 0;
		
		
		wheelFLAngle = 315;
		wheelFLSpeed = 0;
		wheelFRAngle = 225;
		wheelFRSpeed = 0;
		wheelRLAngle = 45;
		wheelRLSpeed = 0;
		wheelRRAngle = 135;
		wheelRRSpeed = 0;
		
		
		rotation = 0;
		threshold = 0.15;
		//shooter = new Talon(5);
		
		myRobot.SetExpiration(0.1);
		//robotFront.SetExpiration(0.1);
		//robotBack.SetExpiration(0.1);
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
		fieldOrientation.Reset();
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
	
	myRobot.SetSafetyEnabled(false);
	myRobot.MecanumDrive_Cartesian(0, 0, 0, 0);
	for(double i = 0; i <= 1; i = i+0.01) {
		myRobot.MecanumDrive_Cartesian(i, i, 0, 0);
		Wait(0.05);
	}
	for(double i = 1; i >= -1; i = i-0.01) {
		myRobot.MecanumDrive_Cartesian(i, i, 0, 0);
		Wait(0.05);
	}
	for(double i = -1; i < 0; i = i+0.01) {
		myRobot.MecanumDrive_Cartesian(i, i, 0, 0);
		Wait(0.05);
	}
	
	
	for(double i = 0; i <= 1; i = i+0.01) {
		myRobot.MecanumDrive_Cartesian(i, -i, 0, 0);
		Wait(0.05);
	}
	for(double i = 1; i >= -1; i = i-0.01) {
		myRobot.MecanumDrive_Cartesian(i, -i, 0, 0);
		Wait(0.05);
	}
	for(double i = -1; i < 0; i = i+0.01) {
		myRobot.MecanumDrive_Cartesian(i, -i, 0, 0);
		Wait(0.05);
	}
	
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
	fieldOrientation.Reset();
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RobotDemo::TeleopPeriodic() {
	//robotFront.SetSafetyEnabled(true);
	//robotBack.SetSafetyEnabled(true);
	myRobot.SetSafetyEnabled(false);
	
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
		//if(stick.GetRawButton(2))
		//	myRobot.MecanumDrive_Cartesian(joyX, joyY, -joyZ, 0); // drive with mechanum style :D (use right stick)
		//else	
		//	myRobot.MecanumDrive_Cartesian(joyX, joyY, -joyZ, fieldOrientation.GetAngle()); // drive with mechanum style :D (use right stick)
		if(stick.GetRawButton(1)) {
			fieldOrientation.Reset();
		}
		if(stick.GetRawButton(3)) {
			temp = fieldOrientation.GetAngle();
			while(temp < -180) 
				temp += 360;
			while(temp >= 180)
				temp -= 360;
			if(temp >= -180 && temp < -4)
				myRobot.MecanumDrive_Cartesian(joyX, joyY, -0.4, temp);
			else if(temp < 180 && temp > 4)
				myRobot.MecanumDrive_Cartesian(joyX, joyY, 0.4, temp);
			else
				myRobot.MecanumDrive_Cartesian(joyX, joyY, 0, temp);
		}
		else if(stick.GetRawButton(2))
			myRobot.MecanumDrive_Cartesian(joyX, joyY, -joyZ, 0); // drive with mechanum style :D (use right stick)
		else	
			myRobot.MecanumDrive_Cartesian(joyX, joyY, -joyZ, fieldOrientation.GetAngle()); // drive with mechanum style :D (use right stick)
		//mecanumDrive.drive(joyX, joyY, joyZ, rotation);
		
		/*
		joyMagnitude = sqrt(pow(joyX, 2) + pow(joyY, 2));
		joyAngle = ((atan2(joyX, -joyY)/pi)*360)+180;
		while(joyAngle < 0)
			joyAngle = joyAngle + 360;
		while(joyAngle > 360)
			joyAngle = joyAngle - 360;
		
		
		
		wheelFLSpeed = (cos((((joyAngle+wheelFRAngle)/360)*2*pi))+sin((((joyAngle+wheelFRAngle)/360)*2*pi)))*joyMagnitude;
		
		
		robotFront.SetLeftRightMotorOutputs(wheelFLSpeed, 0);
		robotBack.SetLeftRightMotorOutputs(0, 0);*
		
		
		
		
		
		
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

