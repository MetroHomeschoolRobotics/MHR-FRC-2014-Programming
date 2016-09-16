#include "WPILib.h"	
#include "math.h"
#define PI 3.14159265
/**asdf
 * This is a demo program showing the use of the RobotBase class.
 * The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
//const double pi = 3.14159265;

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
	Talon* motors;
	
public:
	CyborgsHolonomicDrive(double maxSpeedInput, int wheelQuantityInput, Talon *motorInput[], double angleInput[], int gracefulness=10) {
		wheelQuantity = wheelQuantityInput;
		motors = *motorInput;
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
			motors[i].SetSafetyEnabled(input);
	}
	
	void SetExpiration(double seconds) {
		for(int i = 0; i < wheelQuantity; i++)
			motors[i].SetExpiration(seconds);
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
	void addSamples(double x, double y, double z) {
		joyXsamples[samples] = 0;
		joyYsamples[samples] = 0;
		joyZsamples[samples] = 0;
		for(int i = 0; i < samples-1; i++) {
			joyXsamples[i] = joyXsamples[i+1];
			joyYsamples[i] = joyYsamples[i+1];
			joyZsamples[i] = joyZsamples[i+1];
		}
		for(int i = 0; i < samples; i++) {
			joyXsamples[samples] += (joyXsamples[i]-joyXsamples[samples])/(i+1);
			joyYsamples[samples] += (joyYsamples[i]-joyYsamples[samples])/(i+1);
			joyZsamples[samples] += (joyZsamples[i]-joyZsamples[samples])/(i+1);
		}
		joyXsamples[samples-1] = x;
		joyYsamples[samples-1] = y;
		joyZsamples[samples-1] = z;
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
			motors[i].SetSpeed(speeds[i]*maxSpeed);
		}
	}
};


class RobotDemo : public IterativeRobot
{
private:
	Talon *shooter;
	Talon *driveMotors[4];
	double driveMotorAngles[4];
	CyborgsHolonomicDrive *datRobot;
	Joystick *stick; // only joystick
	Gyro *fieldOrientation;
	double joyX;
	double joyY;
	double joyZ;
	double rotation;
	double threshold;
	double shooterSpeed;
	double temp;
	
	double polarSqrt(double in){
		if(in < 0)
			return -sqrt(-in);
		else
			return sqrt(in);
	}

public:
	RobotDemo()//:
		
	{	
		for(int i = 0; i < 4; i++) {
			driveMotors[i] = new Talon(i+1);
		}
		driveMotorAngles[0] = 315;
		driveMotorAngles[1] = 225;
		driveMotorAngles[2] = 45;
		driveMotorAngles[3] = 135;
		
		datRobot = new CyborgsHolonomicDrive(1.0, 4, driveMotors, driveMotorAngles);
		datRobot->SetExpiration(300);
		
		shooter = new Talon(5);
		shooter->SetSpeed(0);
		shooter->SetExpiration(300);

		joyX = 0;
		joyY = 0;
		joyZ = 0;
		
		stick = new Joystick(1);
		
		rotation = 0;
		threshold = 0.15;


		fieldOrientation = new Gyro(1);
		fieldOrientation->Reset();
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
}

/**
 * Initialization code for autonomous mode should go here.
 * 
 * Use this method for initialization code which will be called each time
 * the robot enters autonomous mode.
 */

void RobotDemo::AutonomousInit() {
	shooter->SetSafetyEnabled(false);
	//myRobot->SetSafetyEnabled(false);
	datRobot->drive(0, 0, 0, 0);
	for(double i = 0; i <= 1; i = i+0.01) {
		cout << i;
		datRobot->drive(i, i, 0, 0);
		Wait(0.02);
	}
	for(double i = 1; i >= -1; i = i-0.01) {
		datRobot->drive(i, i, 0, 0);
		Wait(0.02);
	}
	for(double i = -1; i < 0; i = i+0.01) {
		datRobot->drive(i, i, 0, 0);
		Wait(0.02);
	}
	
	
	for(double i = 0; i <= 1; i = i+0.01) {
		datRobot->drive(i, -i, 0, 0);
		Wait(0.02);
	}
	for(double i = 1; i >= -1; i = i-0.01) {
		datRobot->drive(i, -i, 0, 0);
		Wait(0.02);
	}
	for(double i = -1; i < 0; i = i+0.01) {
		datRobot->drive(i, -i, 0, 0);
		Wait(0.02);
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
	fieldOrientation->Reset();
}

/**
 * Periodic code for teleop mode should go here.
 *
 * Use this method for code which will be called periodically at a regular
 * rate while the robot is in teleop mode.
 */
void RobotDemo::TeleopPeriodic() {
	datRobot->SetSafetyEnabled(true);
	shooter->SetSafetyEnabled(true);
	
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
		joyY = pow((joyY), 3.0);
		joyZ = pow(joyZ, 3.0);//*0.65;
		

		if(stick->GetRawButton(1)) {
			fieldOrientation->Reset();
		}
		
		if(stick->GetRawButton(3))
			datRobot->drive(joyX, joyY, -polarSqrt(sin(fieldOrientation->GetAngle()/360)), fieldOrientation->GetAngle());
		else if(stick->GetRawButton(2))
			datRobot->drive(joyX, joyY, -joyZ, 0); // drive with mechanum style :D (use right stick)
		else	
			datRobot->drive(joyX, joyY, -joyZ, fieldOrientation->GetAngle()); // drive with mechanum style :D (use right stick)
		
		if(stick->GetRawButton(1)){
			shooter->SetSpeed(1);
		} else if(stick->GetRawButton(2)) {
			shooter->SetSpeed(-1);
		} else {
			if(fabs(stick->GetRawAxis(5)) > .15)
				shooter->SetSpeed(pow(stick->GetRawAxis(5), 3));
			else
				shooter->SetSpeed(0);
		}
		
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

