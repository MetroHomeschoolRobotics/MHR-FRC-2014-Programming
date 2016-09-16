#include "WPILib.h"
#include <string>
#include <fstream>
#include <string>
#include <sstream>
#include "math.h"
#define PI 3.14159265
/**asdf
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */
/*
class CyborgsHolonomicDrive {
private:
	double joyAngle;
	double joyMagnitude;
	double largestPower;
	double maxSpeed;
	
	int samples;
	double *joyXsamples;
	double *joyYsamples;
	double *joyZsamples;
	
	
	int wheelQuantity;
	int* ports;
	double* angles;
	double* speeds;
	Talon** motors;
	
public:
	CyborgsHolonomicDrive(double maxSpeedInput, int wheelQuantityInput, Talon *motorInput[], double angleInput[], int gracefulness=10) {
		wheelQuantity = wheelQuantityInput;
		//motors = *motorInput;
		motors = new Talon*[4];
		motors[0] = new Talon(1);
		motors[1] = new Talon(2);
		motors[2] = new Talon(3);
		motors[3] = new Talon(4);
		angles = angleInput;
		maxSpeed = maxSpeedInput;
		samples = gracefulness;
		joyXsamples = new double(samples+1);
		joyYsamples = new double(samples+1);
		joyZsamples = new double(samples+1);
		for(int i = 0; i < samples+1; i++) {
			joyXsamples[i] = 0;
			joyYsamples[i] = 0;
			joyZsamples[i] = 0;
		}
		SetSafetyEnabled(true);
	}
	
	void SetSafetyEnabled(bool input) {
		for(int i = 0; i < wheelQuantity; i++)
			motors[i]->SetSafetyEnabled(input);
	}
	
	void SetExpiration(double seconds) {
		for(int i = 0; i < wheelQuantity; i++)
			motors[i]->SetExpiration(seconds);
	}

	void normalizeSpeeds() {
		largestPower = 0;
		for(int i = 0; i < wheelQuantity; i++) {
			if(fabs(speeds[i]) > largestPower) {
				largestPower = fabs(speeds[i]);
			}
		}
		if(largestPower != 0)
			for(int i = 0; i < wheelQuantity; i++) {
				speeds[i] = speeds[i]/largestPower;
			}
	}

	
	void drive(double joyX, double joyY, double rotation, double gyro = 0.0) {
		addSamples(joyX, joyY, rotation);
		joyX = joyXsamples[samples];
		joyY = joyYsamples[samples];
		rotation = joyZsamples[samples];
		joyMagnitude = sqrt(pow(joyX, 2) + pow(joyY, 2));
		joyAngle = atan2(joyY, joyX);
		
		for(int i = 0; i < wheelQuantity; i++) {
			speeds[i] = cos((PI*(angles[i]-gyro)/180)-joyAngle)-rotation;
		}
		normalizeSpeeds();
		for(int i = 0; i < wheelQuantity; i++) {
			speeds[i] *= joyMagnitude;
		}
		for(int i = 0; i < wheelQuantity; i++) {
			motors[i]->SetSpeed(speeds[i]*maxSpeed);
		}
	}
};
*/

class RobotDemo : public IterativeRobot
{
private:
	Talon *FL, *RL, *FR, *RR;
	Talon *roller;
	Talon *shooter;
	Victor *up_down;
	DriverStationLCD *userMessages;
	//Talon *driveMotors[4];
	double driveMotorAngles[4];
	//CyborgsHolonomicDrive *datRobot;
	RobotDrive *myRobot;
	Joystick *stick; // only joystick
	Gyro *fieldOrientation;
	double joyX;
	double joyY;
	double joyZ;
	double rotation;
	double threshold;
	double shooterSpeed;
	double temp;
	char distance[10];
	AnalogChannel *range;
	Ultrasonic *ultraRange;
	
	int samples;
	double *joyXsamples;
	double *joyYsamples;
	double *joyZsamples;
	
	Encoder *shooterEncoder;
	
	double polarSqrt(double in, double root = 1.6){
		if(in < 0)
			return -pow(-in, 1/root);
		else
			return pow(in, 1/root);
	}
	
	double limit(double input, double limit = 1) {
		if(input > limit)
			return limit;
		if(input < -limit)
			return -limit;
		return input;
	}
	
	void addSamples(double x, double y, double z) {
		for(int i = 0; i < samples-1; i++) {
			joyXsamples[i] = joyXsamples[i+1];
			joyYsamples[i] = joyYsamples[i+1];
			joyZsamples[i] = joyZsamples[i+1];
		}
		joyXsamples[samples-1] = x;
		joyYsamples[samples-1] = y;
		joyZsamples[samples-1] = z;
		joyXsamples[samples] = 0;
		joyYsamples[samples] = 0;
		joyZsamples[samples] = 0;
		for(int i = 0; i < samples; i++) {
			joyXsamples[samples] += (joyXsamples[i]-joyXsamples[samples])/(i+1);
			joyYsamples[samples] += (joyYsamples[i]-joyYsamples[samples])/(i+1);
			joyZsamples[samples] += (joyZsamples[i]-joyZsamples[samples])/(i+1);
		}
	}
	
	double GetDistance() {
		return range->GetVoltage()*100;
	}
	
	void GeneralPeriodic() {
		addSamples(range->GetVoltage()*100, ultraRange->GetRangeInches(), 0);
		sprintf(distance, "%f", joyXsamples[samples]);
		userMessages->PrintfLine((DriverStationLCD::Line) 1, distance);
		sprintf(distance, "%f", joyYsamples[samples]);
		userMessages->PrintfLine((DriverStationLCD::Line) 2, distance);
		sprintf(distance, "%f", shooterEncoder->GetDistance());
		userMessages->PrintfLine((DriverStationLCD::Line) 4, distance);
		sprintf(distance, "%f", pow ( limit( -( atan( ( shooterEncoder->GetDistance()+465 ) / 40 ) ) ), 3) *0.4);
		userMessages->PrintfLine((DriverStationLCD::Line) 5, distance);
		userMessages->UpdateLCD();
	}

public:
	RobotDemo()//:
		
	{	
		//for(int i = 0; i < 4; i++) {
		//	driveMotors[i] = new Talon(i+1);
		//}
		driveMotorAngles[0] = 315;
		driveMotorAngles[1] = 225;
		driveMotorAngles[2] = 45;
		driveMotorAngles[3] = 135;
				
		//datRobot = new CyborgsHolonomicDrive(1.0, 4/*, driveMotors*/, driveMotorAngles);
		//datRobot->SetExpiration(300);

		FL = new Talon(1);
		RL = new Talon(2);
		FR = new Talon(3);
		RR = new Talon(4);
		roller = new Talon(5);
		shooter = new Talon(6);
		up_down = new Victor(7);   // up down for el torro lift delete if broken code 
		myRobot = new RobotDrive(FL, RL, FR, RR);
		myRobot->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot->SetExpiration(10);
		myRobot->SetSafetyEnabled(false);

		roller->Set(0);
		roller->SetExpiration(10);

		shooter->Set(0);
		shooter->SetExpiration(10);
		
		userMessages = DriverStationLCD::GetInstance();

		//userMessages->PrintfLine((DriverStationLCD::Line) 0, "|---\\ -----  /-\\  |-\\  \\   /  |");
		//userMessages->PrintfLine((DriverStationLCD::Line) 1, "|   | |     /   \\ |  \\  \\ /   |");
		//userMessages->PrintfLine((DriverStationLCD::Line) 2, "|---/ ----- |---| |  |   |    |");
		//userMessages->PrintfLine((DriverStationLCD::Line) 3, "|  \\  |     |   | |  /   |    |");
		//userMessages->PrintfLine((DriverStationLCD::Line) 4, "|   \\ ----- |   | |_/    |    o");
		
		//userMessages->PrintfLine((DriverStationLCD::Line) 1, "-_-_-_-_,----------, ");
		//userMessages->PrintfLine((DriverStationLCD::Line) 2, "_-_-_-_-|     /\\_/\\");
		//userMessages->PrintfLine((DriverStationLCD::Line) 3, "-_-_-_-~|__( ^ .^)");
		//userMessages->PrintfLine((DriverStationLCD::Line) 4, "_-_-_-_-\"\"         \"\"");

		joyX = 0;
		joyY = 0;
		joyZ = 0;
		
		stick = new Joystick(1);
		
		rotation = 0;
		threshold = 0.15;
		

		samples = 1;
		joyXsamples = new double(samples+1);
		joyYsamples = new double(samples+1);
		joyZsamples = new double(samples+1);
		for(int i = 0; i < samples; i++) {
			joyXsamples[i] = 0;
			joyYsamples[i] = 0;
			joyZsamples[i] = 0;
		}


		fieldOrientation = new Gyro(1);
		fieldOrientation->Reset();
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
		
		range = new AnalogChannel(1,2);
		ultraRange = new Ultrasonic(13, 14);
		ultraRange->SetAutomaticMode(true);
		
		shooterEncoder = new Encoder(2, 1, false, Encoder::k4X);
		shooterEncoder->Start(); // dot tart
	}
	~RobotDemo() {
		//delete[] FL, RL, FR, RR, shooter, roller, fieldOrientation, myRobot, stick, joyXsamples, joyYsamples, joyZsamples;
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
	//myRobot->SetSafetyEnabled(true);
}

/**
 * Periodic code for disabled mode should go here.
 * 
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in disabled mode.
 */
void RobotDemo::DisabledPeriodic() {
	///*myRobot->MecanumDrive_Cartesian*/datRobot->drive(0, 0.2, 0, 0);
	//userMessages->Printf(DriverStationLCD::kUser_Line1, 1, "Hello World: ", GetDistance());
	//userMessages->
	GeneralPeriodic();
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */

void RobotDemo::AutonomousInit() {
	roller->SetSafetyEnabled(false);
	shooter->SetSafetyEnabled(false);
	myRobot->SetSafetyEnabled(false);
	//datRobot->drive(0, 0, 0, 0);
	myRobot->MecanumDrive_Cartesian(0, 0, 0, 0);
	Wait(3);
	for(double i = 0; i <= 1; i = i+0.01) {
		cout << i;
		//datRobot->drive(i, i, 0, 0);
		myRobot->MecanumDrive_Cartesian(i, i, 0, 0);
		roller->Set(i);
		shooter->Set(i);
		Wait(0.01);
	}
	for(double i = 1; i >= -1; i = i-0.01) {
		//datRobot->drive(i, i, 0, 0);
		myRobot->MecanumDrive_Cartesian(i, i, 0, 0);
		roller->Set(i);
		shooter->Set(i);
		Wait(0.01);
	}
	for(double i = -1; i < 0; i = i+0.01) {
		//datRobot->drive(i, i, 0, 0);
		myRobot->MecanumDrive_Cartesian(i, i, 0, 0);
		roller->Set(i);
		shooter->Set(i);
		Wait(0.01);
	}
	
	
	for(double i = 0; i <= 1; i = i+0.01) {
		//datRobot->drive(i, -i, 0, 0);
		myRobot->MecanumDrive_Cartesian(i, -i, 0, 0);
		roller->Set(i);
		shooter->Set(i);
		Wait(0.01);
	}
	for(double i = 1; i >= -1; i = i-0.01) {
		//datRobot->drive(i, -i, 0, 0);
		myRobot->MecanumDrive_Cartesian(i, -i, 0, 0);
		roller->Set(i);
		shooter->Set(i);
		Wait(0.01);
	}
	for(double i = -1; i <= 0; i = i+0.01) {
		//datRobot->drive(i, -i, 0, 0);
		myRobot->MecanumDrive_Cartesian(i, -i, 0, 0);
		roller->Set(i);
		shooter->Set(i);
		Wait(0.01);
	}
}

/**
 * Periodic code for autonomous mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in autonomous mode.
 */
void RobotDemo::AutonomousPeriodic() {
	GeneralPeriodic();
}

/**
 * Initialization code for teleop mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters teleop mode.
 */
void RobotDemo::TeleopInit() {
	fieldOrientation->Reset();
	for(int i = 0; i < samples; i++) {
		joyXsamples[i] = 0;
		joyYsamples[i] = 0;
		joyZsamples[i] = 0;
	}
	shooterEncoder->Reset();
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RobotDemo::TeleopPeriodic() {
	//datRobot->SetSafetyEnabled(true);
	myRobot->SetSafetyEnabled(false);
	shooter->SetSafetyEnabled(false);
	
		joyX = -stick->GetX();
		if(fabs(joyX) < threshold)
			joyX = 0;
		joyY = -stick->GetY();
		if(fabs(joyY) < threshold)
			joyY = 0;
		joyZ = -stick->GetRawAxis(3);
		if(fabs(joyZ) < threshold)
			joyZ = 0;

		joyX = pow(joyX, 3.0);
		joyY = pow(joyY, 3.0);
		joyZ = pow(joyZ, 3.0);//*0.65;

		/*addSamples(joyX, joyY, joyZ);
		
		joyX = joyXsamples[samples];
		joyY = joyYsamples[samples];
		joyZ = joyZsamples[samples];
*/
		if(stick->GetRawButton(1)) {
			fieldOrientation->Reset();
		}
		
		if(stick->GetRawButton(2))
			//datRobot->drive(joyX, joyY, -polarSqrt(sin(fieldOrientation->GetAngle()/360)), fieldOrientation->GetAngle());
			//myRobot->MecanumDrive_Cartesian(joyX, joyY, polarSqrt(sin((fieldOrientation->GetAngle()/180)*PI)), fieldOrientation->GetAngle());
			myRobot->MecanumDrive_Cartesian(joyX, joyY, polarSqrt(-cos(fmod(((fieldOrientation->GetAngle()/360)*PI)+(0.5*PI), PI))), fieldOrientation->GetAngle());
		else if(stick->GetRawButton(3))
			//datRobot->drive(joyX, joyY, -joyZ, 0); // drive with mechanum style :D (use right stick)
			//myRobot->MecanumDrive_Cartesian(joyX, joyY, -joyZ, 0);
			shooterEncoder->Reset();
		else	
			//datRobot->drive(joyX, joyY, -joyZ, fieldOrientation->GetAngle()); // drive with mechanum style :D (use right stick)
			myRobot->MecanumDrive_Cartesian(joyX, joyY, -joyZ, fieldOrientation->GetAngle());
		/*
		if(stick->GetRawButton(6)){
			shooter->Set(1);
		} else if(stick->GetRawButton(5)) {
			shooter->Set(-0.45);
		} else {
			if(fabs(stick->GetRawAxis(5)) > .15)
				shooter->Set(pow(stick->GetRawAxis(5), 3));
			else
				shooter->Set(0);
		}*/
		//de-1337 ALL the things!!!
		
		if(stick->GetRawButton(4)){
			roller->Set(1);
		} else if(stick->GetRawButton(8)) {
			roller->Set(-1);
		} else {
			if(fabs(stick->GetRawAxis(4)) > .15)
				roller->Set(pow(stick->GetRawAxis(4), 3));
			else
				roller->Set(0);
		}
		if(stick->GetRawAxis(8) < 0){
			shooterEncoder->Reset();
		}	
		
		if (stick->GetRawAxis(7) > 0) { //test ''
			up_down->Set (0.5);   // this is  test delete if broken code 
		}

		else if (stick->GetRawAxis(7) < 0) { //test ''
			up_down->Set(-0.5);   // this is  test delete if broken code 
		}
		
		/*if(stick->GetRawButton(6)) {
			shooter->Set(1);
		} else if(shooterEncoder->GetDistance() = -10){
			shooter->Set (0);
		}*/
		if(stick->GetRawButton(5)) {
			shooter->Set( pow ( limit( -( atan( ( shooterEncoder->GetDistance()+490 ) / 60 ) ) ), 3) *0.45);
		} else if(stick->GetRawButton(6) && shooterEncoder->GetDistance() > -20){
			shooter->Set(1);
		} else if(stick->GetRawButton(7)){
			shooter->Set(0.25); //Much tired.  Very code.  Wow.
		} else {
			shooter->Set(0);
		}
	/*
	 * 	if(SoftwareIssue(True){
	 * 		programmers->FixSpeed(1);
	 * 	} else if(HardwareIssue(True){
	 * 		programmers->FixSpeed(0);
	 */
			
		GeneralPeriodic();
		
}
	
/**
 * Initialization code for test mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters test mode.
 */
/*void RobotDemo::TestInit() {
}*/

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

