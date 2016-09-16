#include "WPILib.h"
#include <cmath>
#include "VisionTargets.cpp"
//#define PI 3.14159265

/**
 * This is a demo program showing the use of the RobotBase class.
 *The IterativeRobot class is the base of a robot application that will automatically call your
 * Periodic methods for each packet based on the mode.
 */ 
class RobotDemo : public IterativeRobot
{
	WaflVision *cameraTargetThingamabob;
	
	Talon *FL, *RL, *FR, *RR;
	RobotDrive *myRobot; // robot drive system
	Talon *roller;
	Talon *shooter;
	Victor *upDown;
	PWM *redLED;
	PWM *greenLED;
	PWM *blueLED;
	Joystick *DriverPad;
	Joystick *ManipulatorPad;

	Gyro *fieldOrientation;
	AnalogChannel *range, *shooterPot;
	DigitalInput *shooterSwitch;
	Encoder *shooterEncoder;
	
	double joyX, joyY, joyZ, joyAngle, joyMagnitude;
	double threshold;
	double temp;
	char distance[10];
	double fullShot;
	double driveSpeed;
	
	double elTorroBack;
	double elTorroResting;
	double elTorroExtended;//Everbody do the flop *FLOP*
	
	bool isShooting;
	bool isElTorroInOperation;
	bool isElTorroReset;
	bool manualControl;
	bool manualControlToggling;

	DriverStationLCD *userMessages;

public:
	RobotDemo() {
		cameraTargetThingamabob = new WaflVision();
		driveSpeed = 1;
		FL = new Talon(1);
		RL = new Talon(2);
		FR = new Talon(3);
		RR = new Talon(4);
		redLED = new PWM(8);
		greenLED = new PWM(9);
		blueLED = new PWM(10);
		
		
		myRobot = new RobotDrive(FL, RL, FR, RR);
		myRobot->SetExpiration(0.1);
		
		myRobot->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		
		roller = new Talon(5);
		roller->Set(0);
		roller->SetExpiration(10);
		
		shooter = new Talon(6);
		shooter->Set(0);
		shooter->SetExpiration(10);
		
		upDown = new Victor(7);
		upDown->Set(0);
		shooter->SetExpiration(10);

		DriverPad = new Joystick(1);
		ManipulatorPad = new Joystick(2);
		
		
		fieldOrientation = new Gyro(1);
		
		range = new AnalogChannel(3);
		shooterPot = new AnalogChannel(2);	
		
		shooterSwitch = new DigitalInput(3);
		
		shooterEncoder = new Encoder(2, 1, false, Encoder::k4X);
		shooterEncoder->Start();
		
		threshold = 0.1;
		temp = 0;
		fullShot = 500; // Previous, 400
		
		elTorroBack = 80;
		elTorroResting = 200;
		elTorroExtended = 330;
		
		driveSpeed = 1.0;
		
		isShooting = false;		
		isElTorroInOperation = false;
		isElTorroReset = false;
		manualControl = false;
		manualControlToggling = false;
		
		userMessages = DriverStationLCD::GetInstance();
		
		this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
	}
	
private:
	
	enum ShotType {
		NORMAL,
		CLOSE,
		PASS
	};
	
	double polarSqrt(double in, double root = 1.8) {
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
	
	double GetDistance() {
		double inTemp;
		inTemp = 0;
		for(int i = 1; i <= 5; i++) {
			inTemp += ((range->GetVoltage()*100)-inTemp)/i;
		}
		return inTemp; 
	}
	
	void AutoShot(double distance = 1, double power = 1, bool instantRetract = false, double shotWait = 0, double retractSpeedOne = 0.75, double retractSpeedTwo = 0.4) {
		distance = limit(distance);
		power = limit(power);
		isShooting = true;
		myRobot->MecanumDrive_Cartesian(0, 0, 0, 0);
		while(shooterPot->GetVoltage()*100 < 245) {
			upDown->Set(-0.75);
			roller->Set(0.5);
		}
		upDown->Set(0);
		roller->Set(0);
		shooter->Set(0);
		
		if(shooterSwitch->Get()) {
			if(shooterEncoder->GetDistance() > fullShot*0.3) {
				
				while(shooterSwitch->Get() && shooterEncoder->GetDistance() > fullShot*0.2 && !DriverPad->GetRawButton(10)) {
					shooter->Set(-retractSpeedOne);
				}
				while(shooterSwitch->Get() && !DriverPad->GetRawButton(10)) {
					shooter->Set(-retractSpeedTwo);
				}
				shooter->Set(0);
				shooterEncoder->Reset();
				
			} else {
				temp = GetTime();
				while(shooterSwitch->Get() && GetTime()-temp < 0.4 && !DriverPad->GetRawButton(10)) {
					shooter->Set(-retractSpeedOne);
				}
				while(shooterSwitch->Get() && !DriverPad->GetRawButton(10)) {
					shooter->Set(-(retractSpeedTwo*1.25));
				}
				shooter->Set(0);
				shooterEncoder->Reset();
			}
			
		} else {
			Wait(shotWait);
			temp = GetTime();
			while(shooterEncoder->GetDistance() < fullShot*distance && GetTime()-temp < limit(0.5/pow(power, 1/3), 2)) {
				shooter->Set(power);
			}
			shooter->Set(0);
			if(!instantRetract)
				Wait(0.5/distance);
			while(shooterSwitch->Get() && shooterEncoder->GetDistance() > fullShot*0.2 && !DriverPad->GetRawButton(10)) {
				shooter->Set(-retractSpeedOne);
			}
			while(shooterSwitch->Get() && !DriverPad->GetRawButton(10)) {
				shooter->Set(-retractSpeedTwo);
			}
			shooter->Set(0);
			shooterEncoder->Reset();
		}		
		while(shooterPot->GetVoltage()*100 > 200) {
			upDown->Set(0.7);
		}
		upDown->Set(0);
		isElTorroReset = true;
		isShooting = false;
	}
	
	void shotSequence(ShotType type = NORMAL) {
		if(DriverStation::GetInstance()->GetDigitalIn(7)) { // NO BANDS
			if(DriverStation::GetInstance()->GetDigitalIn(6)) { // NORMAL MODE
				if(type == NORMAL)
					if(DriverStation::GetInstance()->GetDigitalIn(2)) {
						AutoShot(1, 1, false, 0, 0.05, 0.3);
					} else {
						AutoShot(DriverStation::GetInstance()->GetAnalogIn(1)/5, DriverStation::GetInstance()->GetAnalogIn(2)/5, DriverStation::GetInstance()->GetDigitalIn(3), 0, 0.05, 0.3);
					}
				else if(type == CLOSE)
					AutoShot(0.4-((DriverStation::GetInstance()->GetBatteryVoltage()-12.5)*0.15), 1, true, 0, 0.05, 0.3);
				else if(type == PASS)
					AutoShot(0.5, 0.55, false, 0, 0.05, 0.3);
			} else { // SAFE MODE
				if(type == NORMAL)
					if(DriverStation::GetInstance()->GetDigitalIn(2)) {
						AutoShot(0.8, 0.8, false, 0, 0.05, 0.3);
					} else {
						AutoShot(DriverStation::GetInstance()->GetAnalogIn(1)/5, DriverStation::GetInstance()->GetAnalogIn(2)/5, DriverStation::GetInstance()->GetDigitalIn(3), 0, 0.05, 0.3);
					}
					//AutoShot(1, 1, true, 0, 0.05, 0.3);
				else if(type == CLOSE)
					AutoShot(0.4-((DriverStation::GetInstance()->GetBatteryVoltage()-12.5)*0.15), 1, true, 0, 0.05, 0.3);
				else if(type == PASS)
					AutoShot(0.5, 0.55, false, 0, 0.05, 0.3);
			}
		} else { // BANDS
			if(DriverStation::GetInstance()->GetDigitalIn(6)) { // NORMAL MODE
				if(type == NORMAL)
					if(DriverStation::GetInstance()->GetDigitalIn(2)) {
						AutoShot(0.75, 1, false, 0, 0.75, 0.4);
					} else {
						AutoShot(DriverStation::GetInstance()->GetAnalogIn(1)/5, DriverStation::GetInstance()->GetAnalogIn(2)/5, DriverStation::GetInstance()->GetDigitalIn(3), 0, 0.75, 0.4);
					}
				else if(type == CLOSE)
					AutoShot(0.4-((DriverStation::GetInstance()->GetBatteryVoltage()-12.5)*0.15), 1, true, 0, 0.75, 0.4);
				else if(type == PASS)
					AutoShot(0.5, 0.55, false, 0, 0.75, 0.4);
			} else { // SAFE MODE
				if(type == NORMAL)
					if(DriverStation::GetInstance()->GetDigitalIn(2)) {
						AutoShot(0.7, 0.75, true, 0, 0.75, 0.4);
					} else {
						AutoShot(DriverStation::GetInstance()->GetAnalogIn(1)/5, DriverStation::GetInstance()->GetAnalogIn(2)/5, DriverStation::GetInstance()->GetDigitalIn(3), 0, 0.75, 0.4);
					}
				else if(type == CLOSE)
					AutoShot(0.4-((DriverStation::GetInstance()->GetBatteryVoltage()-12.5)*0.15), 1, true, 0, 0.75, 0.4);
				else if(type == PASS)
					AutoShot(0.5, 0.55, false, 0, 0.75, 0.4);
			}
		}
	}
	
	void GeneralPeriodic() {
		if(manualControl) {
			userMessages->PrintfLine((DriverStationLCD::Line) 0, "");
			userMessages->PrintfLine((DriverStationLCD::Line) 1, "Manual");
		} else {
			userMessages->PrintfLine((DriverStationLCD::Line) 0, "Automatic");
			userMessages->PrintfLine((DriverStationLCD::Line) 1, "");
		}
		sprintf(distance, "%f", shooterEncoder->GetDistance());
		userMessages->PrintfLine((DriverStationLCD::Line) 3, distance);
		sprintf(distance, "%f", shooterPot->GetVoltage()*100);
		userMessages->PrintfLine((DriverStationLCD::Line) 4, distance);
		sprintf(distance, "%f", GetDistance());
		userMessages->PrintfLine((DriverStationLCD::Line) 5, distance);
		
		userMessages->UpdateLCD();

		if(shooterSwitch->Get())
			DriverStation::GetInstance()->SetDigitalOut(1, true);
		else
			DriverStation::GetInstance()->SetDigitalOut(1, false);
	}
	
public:
	/**
	 * Robot-wide initialization code should go here.
	 * 
	 * Use this method for default Robot-wide initialization which will
	 * be called when the robot is first powered on.  It will be called exactly 1 time.
	 */
	void RobotDemo::RobotInit() {
		userMessages->PrintfLine((DriverStationLCD::Line) 1, "DI7: Bands");
		userMessages->PrintfLine((DriverStationLCD::Line) 2, "DI6: Kid Mode");
		userMessages->UpdateLCD();
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
		//GeneralPeriodic();
	}

	/**
	 * Initialization code for autonomous mode should go here.
	 * 
	 * Use this method for initialization code which will be called each time
	 * the robot enters autonomous mode.
	 */
	void RobotDemo::AutonomousInit() {
		myRobot->SetSafetyEnabled(false);
		shooter->SetSafetyEnabled(false);
		roller->SetSafetyEnabled(false);
		upDown->SetSafetyEnabled(false);
		shooterEncoder->Reset();
		fieldOrientation->Reset();
		
		redLED->SetRaw(255);
		greenLED->SetRaw(255);
		blueLED->SetRaw(0);
		
		temp = GetTime();
		while(GetTime()-temp < 1) {
			myRobot->MecanumDrive_Cartesian(0, -0.6, -polarSqrt(-cos(fmod(((fieldOrientation->GetAngle()/360)*PI)+(0.5*PI), PI))), fieldOrientation->GetAngle());
		}
		AutoShot(1, 1, false, 0.75, 0.75, 0.4);
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
		shooterEncoder->Reset();
		manualControl = false;
		redLED->SetRaw(0);
		greenLED->SetRaw(255);
		blueLED->SetRaw(0);
	}

	/**
	 * Periodic code for teleop mode should go here.
	 *
	 * Use this method for code which will be called periodically at a regular
	 * rate while the robot is in teleop mode.
	 */
	void RobotDemo::TeleopPeriodic() {
		myRobot->SetSafetyEnabled(false);
		shooter->SetSafetyEnabled(false);

		if(DriverStation::GetInstance()->GetDigitalIn(6)) {
			if(DriverPad->GetRawButton(5)) {
				driveSpeed = 0.7;
			} else if(DriverPad->GetRawButton(10)) {
				driveSpeed = 0.8;
			} else {
				driveSpeed = 1;
			}
		} else {
			driveSpeed = 0.55;
		}
		
		joyX = DriverPad->GetX();
		if(fabs(joyX) < threshold)
			joyX = 0;
		joyY = DriverPad->GetY();
		if(fabs(joyY) < threshold)
			joyY = 0;
		joyZ = DriverPad->GetRawAxis(3);
		if(fabs(joyZ) < threshold)
			joyZ = 0;

		joyX = pow(joyX, 3.0)*driveSpeed;
		joyY = pow(joyY, 3.0)*driveSpeed;
		joyZ = pow(joyZ, 3.0)*driveSpeed;
		joyMagnitude = DriverPad->GetMagnitude();
		joyAngle = DriverPad->GetDirectionDegrees();
		
		if(DriverPad->GetRawButton(1)) {
			fieldOrientation->Reset();
		}
		
		if(DriverPad->GetRawButton(2))
			myRobot->MecanumDrive_Cartesian(joyX, joyY, -polarSqrt(-cos(fmod(((fieldOrientation->GetAngle()/360)*PI)+(0.5*PI), PI))), fieldOrientation->GetAngle());
		else if(DriverStation::GetInstance()->GetDigitalIn(6))
			myRobot->MecanumDrive_Cartesian(joyX/2, joyY/2, -joyZ/2, fieldOrientation->GetAngle());
		else	
			myRobot->MecanumDrive_Cartesian(joyX, joyY, -joyZ, fieldOrientation->GetAngle());
		
		if(ManipulatorPad->GetRawButton(9)) {
			if(!manualControlToggling) {
				manualControlToggling = true;
				if(!manualControl) {
					manualControl = true;
					isElTorroInOperation = false;
					isElTorroReset = true;
				} else if(ManipulatorPad->GetRawButton(9)) {
					manualControl = false;
				}
			}
		} else {
			manualControlToggling = false;
		}
		if(manualControl) {
			if(ManipulatorPad->GetRawButton(3)) {
				roller->Set(1);
			} else if(ManipulatorPad->GetRawButton(2)) {
				roller->Set(-1);
			} else {
				roller->Set(0);
			}
			if (ManipulatorPad->GetRawButton(1)) {
				upDown->Set (-0.5);
			} else if (ManipulatorPad->GetRawButton(4)) {
				upDown->Set(0.75);
			} else {
				upDown->Set(0);
			}
			
			
			if(ManipulatorPad->GetRawButton(5) && shooterSwitch->Get()) {
				shooter->Set(-0.75);
			} else if(ManipulatorPad->GetRawButton(6) && shooterEncoder->GetDistance() < fullShot){
				shooter->Set(1);
			} else if(ManipulatorPad->GetRawButton(8)){
				shooter->Set(0.55); //Much tired.  Very code.  Wow. (y)
			} else if(ManipulatorPad->GetRawButton(7)) {
				shooter->Set(-0.55);
			} else {
				shooter->Set(0);
			}
			if((ManipulatorPad->GetRawButton(7) || ManipulatorPad->GetRawButton(5)) && !shooterSwitch->Get()) {
				shooterEncoder->Reset();
			}
		
		} else {
			if(ManipulatorPad->GetRawButton(3)) {
				isElTorroInOperation = true;
				isElTorroReset = false;
				roller->Set(-1);
				if(shooterPot->GetVoltage()*100 > elTorroBack) {
					upDown->Set(0.75);
				} else {
					upDown->Set(0);
				}
			} else if(ManipulatorPad->GetRawButton(4)) {
				isElTorroInOperation = true;
				isElTorroReset = false;
				roller->Set(1);
				if(shooterPot->GetVoltage()*100 < elTorroExtended) {
					upDown->Set(-0.5);
				} else {
					upDown->Set(0);
				}
			} 
			
			if(!isElTorroInOperation && !isElTorroReset) {
				if(shooterPot->GetVoltage()*100 < elTorroResting-15) {
					upDown->Set(-0.5);
				} else if(shooterPot->GetVoltage()*100 > elTorroResting+15) {
					upDown->Set(0.65);
				} else {
					upDown->Set(0);
					isElTorroReset = true;
					roller->Set(0);
				}
			}
			if(!(ManipulatorPad->GetRawButton(3) || ManipulatorPad->GetRawButton(4))) {
				isElTorroInOperation = false;
			}
			
			if(!isShooting && !isElTorroInOperation && isElTorroReset) {
				if(ManipulatorPad->GetRawButton(6)) {
					//isShooting = true;
					shotSequence(NORMAL);
				} else if(ManipulatorPad->GetRawButton(7)) {
					shotSequence(CLOSE);
				} else if(ManipulatorPad->GetRawButton(8)) {
					shotSequence(PASS);
				}
			}
		}
		
		
		
		if(!isShooting && !isElTorroInOperation && isElTorroReset) {
			if(DriverPad->GetRawButton(6)) {
				//isShooting = true;
				shotSequence(NORMAL);
			} else if(DriverPad->GetRawButton(7)) {
				shotSequence(CLOSE);
			} else if(DriverPad->GetRawButton(8)) {
				shotSequence(PASS);
			} else if(!manualControl) {
				roller->Set(0);
			}
		}
		
		
		
		if(DriverPad->GetRawButton(3)) {
			isElTorroInOperation = true;
			isElTorroReset = false;
			roller->Set(-1);
			if(shooterPot->GetVoltage()*100 > elTorroBack) {
				upDown->Set(0.75);
			} else {
				upDown->Set(0);
			}
		} else if(DriverPad->GetRawButton(4)) {
			isElTorroInOperation = true;
			isElTorroReset = false;
			roller->Set(1);
			if(shooterPot->GetVoltage()*100 < elTorroExtended) {
				upDown->Set(-0.5);
			} else {
				upDown->Set(0);
			}
		} 
		
		if(!isElTorroInOperation && !isElTorroReset) {
			if(shooterPot->GetVoltage()*100 < elTorroResting-15) {
				upDown->Set(-0.5);
			} else if(shooterPot->GetVoltage()*100 > elTorroResting+15) {
				upDown->Set(0.65);
			} else {
				upDown->Set(0);
				isElTorroReset = true;
				roller->Set(0);
			}
		}
		if(!(DriverPad->GetRawButton(3) || DriverPad->GetRawButton(4))) {
			isElTorroInOperation = false;
		}
		
		GeneralPeriodic();
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
		greenLED->SetRaw((int)(DriverStation::GetInstance()->GetAnalogIn(2)*51));
		redLED->SetRaw((int)(DriverStation::GetInstance()->GetAnalogIn(1)*51));
		if(DriverStation::GetInstance()->GetDigitalIn(8))
			blueLED->SetRaw((int)(DriverStation::GetInstance()->GetAnalogIn(3)*51));
		else
			blueLED->SetRaw(1);
		GeneralPeriodic();
	}

};

START_ROBOT_CLASS(RobotDemo);