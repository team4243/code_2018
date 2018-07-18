///MISSING: Limit Switches do not stop Raise and Lower cage motor functions (Yet).

//!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\\	IMPORTANT		|
// VERY IMPORTANT NOTICE: For the sake of the comments, do not EVER use Ctrl+A and then Ctrl+I	//		IMPORTANT	|
//!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\\	IMPORTANT 		|

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <iostream>
#include <string>
#include <TimedRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "AHRS.h"
#include "Timer.h"
//#include <MecanumDrive.h>

static double maxSliderValue = 5.0;
static double currentEncoderPositionInches = -1.0;

static const double JoyInStepDown = 0.5;
int autoLoopCounter = 0;
double ToAutoLineSpeed= .2;

const int lf_DeviceNumber = 53;
const int lr_DeviceNumber = 62;
const int rf_DeviceNumber = 60;
const int rr_DeviceNumber = 59;

const int gyroResetButton = 5;

class Robot: public TimedRobot {
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//BEGIN ELEVATOR RELATED DECLARATIONS:
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	Timer timer;

	//Cage Variables.
	//!*!*! SET THESE FOR REAL, THEY ARE DUMMY VALUES!!!
	const double middleCageSpan = 40.5;		//Value in inches
	const double cubeCageSpan = 35.0;			//Value in inches


	const int fullRotation = 4096;  // Full rotation in encoder units.
	const int halfRotation = fullRotation / 2;
	const double rotationsPerInch = 2.0;

	//Cube Gatherer Logic
	bool previousLeftBumperButtonValue = false;
	bool gathererSwitchStateOnLeftBumperButtonPress = false;

	//Limit Switches
	const int cubeInputSwitchChannel = 0;
	const int lowerMiddleCageSwitchChannel = 4;
	const int upperMiddleCageSwitchChannel = 3;
	const int lowerCubeCageSwitchChannel = 1;
	const int upperCubeCageSwitchChannel = 2;
	DigitalInput cubeInputSwitch;
	DigitalInput lowerMiddleSwitch;
	DigitalInput upperMiddleSwitch;
	DigitalInput lowerCubeSwitch;
	DigitalInput upperCubeSwitch;

	//RangeSensors
	const int frontRangeSensorChannel = 0;
	AnalogInput frontRangeSensor;

	//Gear Ratio Stuff (From the number of teeth in the gearbox.)
	const double encoderToLeadScrewMiddleCageGearRatio = 16.0 / 18.0;		//Setting this as " 16.0 / 18.0 " Means:
	const double encoderToLeadScrewCubeCageGearRatio = 16.0 / 18.0;			//For every 16 encoder rotations,
																			//the lead screw will have rotated 18 times.

	//This is the maxumum value in Inches the encoder of the middle cage can reach.
	const double maxMiddleCageEncoderRot = middleCageSpan * rotationsPerInch * encoderToLeadScrewMiddleCageGearRatio;
	//This is the maximum value in Native Units the encoder of the middle cage can theoretically reach.
	const double maxMiddleCageEncoderNU = maxMiddleCageEncoderRot * fullRotation;
	//This is the maxumum value in Inches the encoder of the cube cage can reach.
	const double maxCubeCageEncoderRot = cubeCageSpan * rotationsPerInch * encoderToLeadScrewCubeCageGearRatio;
	//This is the maximum value in Native Units the encoder of the cube cage can theoretically reach.
	const double maxCubeCageEncoderNU = maxCubeCageEncoderRot * fullRotation;

	//Joystick
	Joystick gamePad;
	const static int gamePadChannel = 0;	//GamePad object on USB port 0 (elevator)
	const static int joystickChannel = 1; 	//Joystick object on USB port 1 (mecanum drive)

	//elevatorGamePad buttons
	const static int GreenBtn = 1;		//buttons #s start at 1
	const static int RedBtn = 2;
	const static int BlueBtn = 3;
	const static int YellowBtn = 4;
	const static int LeftBumperBtn = 5;
	const static int RightBumperBtn = 6;
	const static int BackBtn = 7;
	const static int StartBtn = 8;

	const static int LeftXAxis = 0;
	const static int LeftYAxis = 1;
	const static int LeftTriggerAxis = 2;
	const static int RightTriggerAxis = 3;
	const static int RightXAxis = 4;
	const static int RightYAxis = 5;

	const static int POVAngle = 0;

	//Misc
	double elevatorSpeed = 1;				//This must always be positive and range from 0 to 1
	const static int errorLogChronomanagerLength = 32;
	//So the ERROR and WARNING messages don't spamm but also run when they have to run. It is an array of individual timers.
	int errorLogChronomanager[errorLogChronomanagerLength] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };
	long int loopNow = 0;		//Rudimentary Time Keeping Value
	const int loopWait = 50;	//Delay Length
	const double floorOffset = 2; //In inches, how much is the 0 height below the floor?.
	std::string plateAlignement = "NaN";
	double currentTime = 0.0;

	//Price of Automation
	int autoMode = 0; 	//0 Means the robot will only cross the auto line.
						//1 Means the robot will capture the left <- Switch Plate
						//2 Means the robot will capture the right -> Switch Plate
						//Left and Right are relative to the driver station perspective.

	int startingPosition = -1; 	//-1 Means UNKNOWN
								//0 is LEFTMOST
								//1 is MIDDLE
								//2 is RIGHTMOST

	double autoX = 0;	//X "Joystick" value for AUTONOMOUS mode. Domain: [-1, 1]
	double autoY = 0;	//Y "Joystick" value for AUTONOMOUS mode. Domain: [-1, 1]
	double autoZ = 0;	//Z "Joystick" value for AUTONOMOUS mode. Domain: [-1, 1]

	//An "Autonomous Event" is a series of steps. Like: 0=Drive Forward, 1=Turn Right, 2=Stop and Drop Cube
	int autonomousSuccession = 0; 	//Autonomous events recorder.
	double autonomousDelay = 0.0;	//Delay between autonomous events.

	//Quest 4 the Absolute Zero
	bool hasHitTheLimit = false;
	bool cubeZero = false;
	bool middleZero = false;

	//Main motors are controlled by code, non-main motors follow their corresponding main motor.
	//  Diagonal Motor follows Straight. Diagonal refers to the way the cube gatherer is built.
	const int middleCageMotorChannel = 51;
	const int middleCageMotorMainChannel = 50;
	const int cubeCageMotorChannel = 54;
	const int cubeCageMotorMainChannel = 56;
	const int cubeGathererMotorLeftChannel = 58;
	const int cubeGathererMotorRightChannel = 57;

	TalonSRX middleCageMotor;
	TalonSRX middleCageMotorMain;
	TalonSRX cubeCageMotor;
	TalonSRX cubeCageMotorMain;
	TalonSRX cubeGathererMotorLeft;
	TalonSRX cubeGathererMotorRight;

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//END ELEVATOR RELATED DECLARATIONS
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//BEGIN DRIVE-TRAIN RELATED DECLARATIONS:
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	WPI_TalonSRX *lf = new WPI_TalonSRX(lf_DeviceNumber); //left front
	WPI_TalonSRX *lr = new WPI_TalonSRX(lr_DeviceNumber);//left rear
	WPI_TalonSRX *rf = new WPI_TalonSRX(rf_DeviceNumber); //right front
	WPI_TalonSRX *rr = new WPI_TalonSRX(rr_DeviceNumber); //right rear

	/*
class Robot: public IterativeRobot {
	TalonSRX *lf = new TalonSRX(2); //left front
	TalonSRX *lr = new TalonSRX(3);//left rear
	TalonSRX *rf = new TalonSRX(4); //right front
	TalonSRX *rr = new TalonSRX(5); //right rear
*/
	//initalizing camera

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//END DRIVE-TRAIN RELATED DECLARATIONS
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

public:
	MecanumDrive *m_robotDrive;	// RobotDrive object using PWM 1-4 for drive motors
	//RobotDrive m_robotDrive;	// RobotDrive object using PWM 1-4 for drive motors
	Joystick *m_driveStick;
	AHRS *navxGyro;
	CameraServer *cam0;

	/**
	 * Constructor for this "Robot" Class.
	 */
	Robot(void)  :
		cubeInputSwitch(cubeInputSwitchChannel),
		lowerMiddleSwitch(lowerMiddleCageSwitchChannel),
		upperMiddleSwitch(upperMiddleCageSwitchChannel),
		lowerCubeSwitch(lowerCubeCageSwitchChannel),
		upperCubeSwitch(upperCubeCageSwitchChannel),
		frontRangeSensor(frontRangeSensorChannel),
		gamePad(gamePadChannel),
		middleCageMotor(middleCageMotorChannel),
		middleCageMotorMain(middleCageMotorMainChannel),
		cubeCageMotor(cubeCageMotorChannel),
		cubeCageMotorMain(cubeCageMotorMainChannel),
		cubeGathererMotorLeft(cubeGathererMotorLeftChannel),
		cubeGathererMotorRight(cubeGathererMotorRightChannel)
	{
	   // m_robotDrive(lf_DeviceNumber, lr_DeviceNumber,
	   // 		rf_DeviceNumber, rr_DeviceNumber),
		m_robotDrive = new MecanumDrive(*lf, *lr, *rf, *rr);

	    m_driveStick = new Joystick(joystickChannel);
		navxGyro = new AHRS(SPI::Port::kMXP);

		/* Set every Talon to reset the motor safety timeout. */
		lf->Set(ControlMode::PercentOutput, 0);
		lr->Set(ControlMode::PercentOutput, 0);
		rf->Set(ControlMode::PercentOutput, 0);
		rr->Set(ControlMode::PercentOutput, 0);

		//In MecanumDrive, right side motors are automatically inverted. In RobotDrive they are not.
		//Invert Right Side  //In testing, we see the need to invert.
		rf->SetInverted(true);
		rr->SetInverted(true);
		lf->SetInverted(true);
		lr->SetInverted(true);

		m_robotDrive-> SetExpiration(0.5);
		m_robotDrive->SetSafetyEnabled(false); {
			cam0 = CameraServer::GetInstance();
			cam0->StartAutomaticCapture();
		}
	}

	void AutonomousInit() {
		// Assumes robot is aligned North at start of match.
		navxGyro->Reset();
		timer.Reset();
		timer.Start();

		plateAlignement = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if (plateAlignement.c_str()[0] == 'L') {
			Log("WARNING: Switch is Left. Delivering Cube.", -1);
			autoMode = 1;
		} else if (plateAlignement.c_str()[0] == 'R') {
			Log("WARNING: Switch is Right. Delivering Cube.", -1);
			autoMode = 2;
		} else {
			Log("ERROR: Our switch was not found! Only crossing auto line.", -1);
			std::cout << "WARNING: Plate Alignement Values: " << plateAlignement.c_str()[0] << plateAlignement.c_str()[1] << plateAlignement.c_str()[2] << std::endl;
			autoMode = 0;
		}

		//////////////////////////////////////////// GET STARTING POSITION! ///////////////////////////////////////////

		if (m_driveStick->GetThrottle() > 0.5) { startingPosition = -1; }
		else if (m_driveStick->GetThrottle() > -0.5) { startingPosition = 1; }
		else {startingPosition = -1; }


		std::cout << "WARNING) STarting Postion is: " << startingPosition << std::endl;

		 if (startingPosition == -1) {
			 Log("ERROR: Starting Position Could not be Determined. Will just drive forward in an attempt to cross auto line.", -1);
			 autoMode = 0;
		 }
		autonomousSuccession = 0;
	}

	void AutonomousPeriodic() {
		currentTime = timer.Get();
		/*  simplest case: Drive forward ~12ft
			Pivot Right or Left to face switch
			Move fwd a few feet (bump into wall or use range sensor)
			Eject Cube
			Do a victory dance!!!!!!
	 	 */


		std::cout << "Range: " << frontRangeSensor.GetVoltage() << std::endl;

		//std::cout << "AutonomousSuccession: " << autonomousSuccession << std::endl;

		if (autoMode == 1) {
			//Left Switch Capture
			//SendCagesToHeight(20 + floorOffset);
			//The switch's fence height is 18.75 inches (Fence is always taller than switch).
			//The Exchange Portal's Height is 20 inches. Portal Heights are the same.
			Log("WARNING: Positioning cages at SWITCH Height.", 21);
			if (startingPosition == 0) {		//Leftmost
				if ((autonomousSuccession == 0) && (autonomousDelay < currentTime)) {
					autoY = 1;
					autonomousSuccession++;
					autonomousDelay = currentTime + 8.0;
				}
				if (autonomousSuccession == 1) {
					autoY = 0;
					autoZ = 1;
					autonomousSuccession++;
				}
				if ((autonomousSuccession == 2) && (navxGyro->GetAngle() > 88)) {
					autoZ = 0;
					autoY = 1;
					autonomousSuccession++;
				}
				if ((autonomousSuccession == 3) && (frontRangeSensor.GetVoltage() > 1)) {
					autoZ = 0;
					autoY = 0;
					EjectCube();
				}
			} else if (startingPosition == 1) {	//Middle
				if ((autonomousSuccession == 0) && (autonomousDelay < currentTime)) {
					autoY = 0.5; //Super cool Diagonal Strafe
					autoX = -1;
					autonomousSuccession++;
					autonomousDelay = currentTime + 1.0;
				}
				if ((autonomousSuccession == 1) && (autonomousDelay < currentTime)) {
					autoY = -1;
					autoX = 0;
					autonomousSuccession++;
				}
				if ((autonomousSuccession == 2) && (frontRangeSensor.GetVoltage() > 1)) {
					autoY = 0;
					EjectCube();
				}
			} else if (startingPosition == 2) { //Rightmost
				//Not enough time to make the robot go to the opposite side so it will just cross auto line.
				autoMode = 0;
			}
		} else if (autoMode == 2) { //Right Switch Capture

			//SendCagesToHeight(20 + floorOffset);
			//The switch's fence height is 18.75 inches (Fence is always taller than switch).
			//The Exchange Portal's Height is 20 inches. Portal Heights are the same.
			//Log("WARNING: Positioning cages at SWITCH Height.", 21);

			if (startingPosition == 0) {		//Leftmost
				//Not enough time to make the robot go to the opposite side so it will just cross auto line.
				autoMode = 0;
			} else if (startingPosition == 1) {	//Middle
				if ((autonomousSuccession == 0) && (autonomousDelay < currentTime)) {
					autoY = 0.5; //Super cool Diagonal Strafe
					autoX = 1;
					autonomousSuccession++;
					autonomousDelay = currentTime + 1.0;
				}
				if ((autonomousSuccession == 1) && (autonomousDelay < currentTime)) {
					autoY = 1;
					autoX = 0;
					autonomousSuccession++;
				}
				if ((autonomousSuccession == 2) && (frontRangeSensor.GetVoltage() > 1)) {
					autoY = 0;
					EjectCube();
				}
			} else if (startingPosition == 2) { //Rightmost
				if ((autonomousSuccession == 0) && (autonomousDelay < currentTime)) {
					autoY = 1;
					autonomousSuccession++;
					autonomousDelay = currentTime + 8.0;
				}
				if (autonomousSuccession == 1) {
					autoY = 0;
					autoZ = -1;
					autonomousSuccession++;
				}
				if ((autonomousSuccession == 2) && (navxGyro->GetAngle() > 88)) {
					autoZ = 0;
					autoY = 1;
					autonomousSuccession++;
				}
				if ((autonomousSuccession == 3) && (frontRangeSensor.GetVoltage() > 1)) {
					autoZ = 0;
					autoY = 0;
					EjectCube();
				}
			}
		} else {
			//Auto Line Cross
			if ((autonomousSuccession == 0) && (autonomousDelay <= currentTime)) {
				autoY = -0.5;
				autonomousSuccession++;
				autonomousDelay = currentTime + 3.0;
			}
			if ((autonomousSuccession == 1) && (autonomousDelay <= currentTime)) {
				autoY = 0;
				autonomousSuccession++;
				autonomousDelay = currentTime + 70.0;
			}
		}

		m_robotDrive->DriveCartesian( autoX, autoY, autoZ, navxGyro->GetAngle());
	}

	void TeleopInit() {
		hasHitTheLimit = false;
		middleZero = false;
		cubeZero = false;
		//navxGyro->Reset();	//Only do this when robot is facing "north"
		//middleCageMotorMain.SetSelectedSensorPosition(0, 0, 10);	//Resets all important sensor positions.	//
		//cubeGathererMotorRight.SetSelectedSensorPosition(0, 0, 10);												//
		elevatorSpeed = 0.2;										//This should be controlled by the control, this is to be removed	//
		loopNow = 0;												//Resets 'LoopNow'		//
		ResetErrorTimers();											//Explained by Itself	//
		//Follow motors are on opposite sides, so they must be inverted.
		middleCageMotor.SetInverted(true);	//Makes the subordinate motor rotate to the opposite direction the Main is rotating.	//
		cubeCageMotor.SetInverted(true);	//Makes the subordinate motor rotate to the opposite direction the Main is rotating.	//
		cubeGathererMotorLeft.SetInverted(true);	//Same as Above.	//
		middleCageMotorMain.SetInverted(false);		//Making sure they are not the reversed
		cubeCageMotorMain.SetInverted(false);		//Making sure they are not the reversed
		cubeGathererMotorRight.SetInverted(false);	//Same	//

		SmartDashboard::PutString("DB/String 0", "Fake Cage Values 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ");
		SmartDashboard::PutString("DB/String 1", "Slider 0 is Mid");
		SmartDashboard::PutString("DB/String 2", "Slider 1 is Cube"	);

		if (m_driveStick->GetThrottle() > 0.5) { startingPosition = 0; }
		else if (m_driveStick->GetThrottle() > -0.5) { startingPosition = 1; }
		else {startingPosition = 2; }

		std::cout << "WARNING: STarting Postion is: " << startingPosition << std::endl;

	}
	/** @return 10% deadband */
	//Let the record show that Richard would rewrite this...
	double Db(double axisVal) {
		if (axisVal < -0.15)
			return axisVal;
		if (axisVal > +0.15)
			return axisVal;
		return 0;
	}
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void) {
		//Limit Switch Tester
		//std::cout << "MiddleUp: " << upperMiddleSwitch.Get() << ", MiddleDown: " << lowerMiddleSwitch.Get() << ", CubeUP: " << upperCubeSwitch.Get() << ", CubeDown: " << lowerCubeSwitch.Get()  << std::endl;


		//std::cout <<"WARNING: Pos = " << middleCageMotorMain.GetSensorCollection().GetQuadraturePosition() << std::endl;

		if (!hasHitTheLimit) { ResearchForVoid(); } //If it hasn't hit the limit, it searches for the nothing.

		//Makes elevator motor pairs follow/sync with each other.
		middleCageMotor.Follow(middleCageMotorMain);
		//cubeCageMotor.Follow(cubeCageMotorMain);
		middleCageMotor.Set(ControlMode::PercentOutput, 0);
		//middleCageMotorMain.Set(ControlMode::PercentOutput, 0);
		cubeCageMotor.Set(ControlMode::PercentOutput, 0);
		//cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
		//std::cout << "Range: " << frontRangeSensor.GetVoltage() << std::endl;

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		///This Code Block is to make the Cages travel to a preespecified position and make them rise or fall.//
		////////////////////////////////////////////////////////////////////////////////////////////////////////


		if (hasHitTheLimit) {
			if (gamePad.GetPOV() == 0 || m_driveStick->GetRawButton(11)) {		//Raise both cages if the UpArrow is being pressed.
				RaiseCages();
				Log("WARNING: Raising Cages.", 18);
			} else if (gamePad.GetPOV() == 180 || m_driveStick->GetRawButton(12)) { //Lower both cages if the DownArrow is being pressed
				LowerCages();
				Log("WARNING: Lowering Cages.", 19);
			} else if (gamePad.GetRawButton(GreenBtn)) {//Green button sends cages to the a height of 0 inches. (Floor)
				middleCageMotorMain.Set(ControlMode::Position, fullRotation);

				//SendCagesToHeight(0 + floorOffset);		//Well... its the floor so, not much to describe. Its height is 0 inches :P
				Log("WARNING: Positioning cages at FLOOR Height.", 20);
			} else if (gamePad.GetRawButton(RedBtn)) {	//Red Button sends cages to a height of 20 inches. (Exchange Portal / Switch / Portal)
				middleCageMotorMain.Set(ControlMode::Position, fullRotation * 2);
				//SendCagesToHeight(20 + floorOffset);	//The switch's fence height is 18.75 inches (Fence is always taller than switch).
												//The Exchange Portal's Height is 20 inches. Portal Heights are the same.
				Log("WARNING: Positioning cages at PORTAL Height.", 21);
			} else if (gamePad.GetRawButton(BlueBtn)) { //Blue button sends cages to a height of 64 inches. (Scale)
				middleCageMotorMain.Set(ControlMode::Position, fullRotation * 3);
				//SendCagesToHeight(64 + floorOffset);	//Height of Scale = 5ft = 60in; Height of Fence = 3.5in
													//True height of the scale including the fence is 63.5 inches
				Log("WARNING: Positioning cages at SCALE Height.", 22);
			} else if (gamePad.GetRawButton(YellowBtn)) { 	//Yellow Button sends cages to a height of 81 inches. (Rung)
				middleCageMotorMain.Set(ControlMode::Position, fullRotation * 10);
				//SendCagesToHeight(81 + floorOffset);		//Height of the top of the Rung from the Carpet = 84in; Height of Platform = 3.5in
														//True height from the platform to the top of the rung is 80.5 inches.'
				Log("WARNING: Positioning cages at RUNG Height.", 23);
			} else {
				middleCageMotorMain.Set(ControlMode::PercentOutput, 0);
				cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
				cubeCageMotor.Set(ControlMode::PercentOutput, 0);
			}
		}


		///////////////////////////////////////////////////////////////////////
		///This Code Block is to make the cube Gatherer Fetch or Expel cubes.//
		///////////////////////////////////////////////////////////////////////

		//The frame the Left Bumper Button is pressed, it saves the current press state of the cube gatherer limit switch.

		if (gamePad.GetRawButton(LeftBumperBtn) && !previousLeftBumperButtonValue) { gathererSwitchStateOnLeftBumperButtonPress = cubeInputSwitch.Get(); }

		if (gamePad.GetRawAxis(LeftTriggerAxis) > 0.1 || m_driveStick->GetRawButton(9)) {	//If the Left Trigger is pressed, with deadband of 0.1
			AbsorbCube();
			Log("WARNING: Absorbing Cube.", 24);
		} else if (gamePad.GetRawAxis(RightTriggerAxis) > 0.1  || m_driveStick->GetRawButton(10)) {	//If the Right Trigger is pressed, with deadband of 0.1
			EjectCube();
			Log("WARNING: Ejecting Cube.", 25);
		} else if (gamePad.GetRawButton(LeftBumperBtn)) {	//If no trigger is pressed and the Left Bumper Is.
			if (gathererSwitchStateOnLeftBumperButtonPress) {
				EjectCube();
				Log("WARNING: Ejecting Cube.", 25);
			}	//If there was a cube detected, it ejects.
			if (!gathererSwitchStateOnLeftBumperButtonPress) {
				AbsorbCube();
				Log("WARNING: Absorbing Cube.", 24);
			}	//if there was no cube detected, it absorbs.
		} else {
			cubeGathererMotorRight.Set(ControlMode::PercentOutput, 0);
			cubeGathererMotorLeft.Set(ControlMode::PercentOutput, 0);
		}
		previousLeftBumperButtonValue = gamePad.GetRawButton(LeftBumperBtn);

		loopNow++; //Increments loopsNow.


		double x = m_driveStick->GetX(), y = m_driveStick->GetY(), z =  m_driveStick->GetZ();

		if (m_driveStick->GetThrottle() >= 0) { x = gamePad.GetRawAxis(0);  y = gamePad.GetRawAxis(1); z = gamePad.GetRawAxis(4); }


		//float angle = navxGyro->GetAngle();
		//std::cout << "Angle : " << angle << std::endl;
		//if (middleCageMotorMain.GetSensorCollection().GetQuadraturePosition() == 0 && cubeCageMotorMain.GetSensorCollection().GetQuadraturePosition() == 0) {
		m_robotDrive->DriveCartesian(			-Db(JoyInStepDown *x),
												Db(JoyInStepDown * y),
												-Db(JoyInStepDown * z),
												0.0);
		//} else if (m_driveStick->GetX() > 0.5 || m_driveStick->GetY() > 0.5 || m_driveStick->GetZ() > 0.5) { LowerCages(); }

		//m_robotDrive->DriveCartesian(m_driveStick->GetX(), m_driveStick->GetY(), m_driveStick->GetZ(),0);
		// my right side motors need to drive negative to mve robot forward

		// on button 5, reset gyro angle to zero

		if (m_driveStick->GetRawButton(gyroResetButton))
			navxGyro->Reset();
	}

	////Target Cage Position Functions////--------------------------------------------------------------------------------|	O

	/*Sends the cages to the targetHeight.								//
	//It is based on the fastest way to get to the target height, this	//
	//involves moving both cages at the same time so it goes 2x faster	//
	//than moving one and then the other, we also want them to move		//
	//simultaneously the most possible to take the biggest advantage.	*/
	//targetHeight is the height in inches the bottom of the cube cage must reach	//
	//No errors return a 0. No errors can happen here.					//
	int SendCagesToHeight(double targetHeight) {
		double targetCubeCageHeight = 0, targetMiddleCageHeight = 0;

		if (((targetHeight * 0.5) < cubeCageSpan) && ((targetHeight * 0.5) < middleCageSpan)) {
			//It divides equally the distance that must be traveled, as long as half the		//
			//total distance is not greater than any of the cage spans.							//
			//E.g. the target height is 30, then each cage climbs 15 inches, therefore			//
			//the task is acomplished twice as fast than having one cage go up the 30 inches.	//
			targetCubeCageHeight = (targetHeight * 0.5);
			targetMiddleCageHeight = (targetHeight * 0.5);



			//If half the target height cannot be equally divided because the span of one cage is not long enough, then...			//
		} else if (middleCageSpan > cubeCageSpan) {
			//If the cube cage span is the shortest, then the cube cage will extend completely.			//
			targetCubeCageHeight = cubeCageSpan;

			//And then, the middle cage will attempt to cover the remaining distance.					//
			targetMiddleCageHeight = (targetHeight - cubeCageSpan);
			//Note that I say "attempt" because the distance might just be too long for the total cage span.	//
			//This case is handled below.																		//
		} else {
			//If the middle cage span is the shortest, then the middle cage will extend completely.			//
			targetMiddleCageHeight = middleCageSpan;
			//And then, the cube cage will attempt to cover the remaining distance.					//
			targetCubeCageHeight = (targetHeight - middleCageSpan);
			//Note that I say "attempt" because the distance might just be too long for the total cage span.	//
			//This case is handled below.																		//
		}

		//Now, the middle and cube cages are told to go to their calculated heights...			//
		//SendMiddleCageTo(targetMiddleCageHeight);												//
		//SendCubeCageTo(targetCubeCageHeight);													//
		/*But what if one of the two cages' target height is too long or a negative number?		//
		//This cases are handled in each cage's respective function.							//
		//We probably want it this current way for testing purposes, but during the real match,	//
		//I strongly recommed replacing those two lines for:									*/
		SendMiddleCageTo(targetMiddleCageHeight, true, true);									//
		SendCubeCageTo(targetCubeCageHeight, true, true);										//
		//This will make them error proof. We want the errors in testing to know somehting is	//
		//weird though.																			*/

		return 0;
	}

	/*Sends commands the main middle cage motor to rotate the lead screws 		//
	//to where the cage reaches the targetHeightInches height.					/
	/targetHeightInches is the height the middle cage must go to.								//
	//negativeToZero, if set to true, will make negative target heights make the cage go to 0	//
	//infinityToMax, if set to true, will make heights greater than max height become the max	/
	/Negative numbers will result in an error (Returned value will be -1)						//
	//Note that this error will be ignored if negativeToZero is set to true						//
	//E.g. because zero is always the closest acceptable height to a negative, we have the 		//
	//option to if the height is for some reason neagtive, the cage targets 0 height instead.	/
	/Too large numbers will result in an error (Returned value will be -2)						//
	//Note that this error will be ignored if infinityToMax is set to true						//
	//E.g. because the max height is always the closest acceptable height to a larger target,	//
	//we have the option to if the height is for some reason greater than the max, the cage		//
	//targets its maximum height instead.														/
	/No error returns a 0														*/
	int SendMiddleCageTo(double targetHeightInches, bool negativeToZero = false, bool infinityToMax = false) {
		double targetEncoderRevolutions = targetHeightInches * rotationsPerInch * encoderToLeadScrewMiddleCageGearRatio;
														//Transforms the target inches into the number of revolutions	//
														//the motor has to target.										//
														//The current lead screw gives two revolutions to go up 1 inch.	//

		double nativeUnits = targetEncoderRevolutions * fullRotation;//The number of revolutions is transformed into native units.	//
														//There are 4096 native units per revolution.					//

		//These error codes will dectect whether there is something weird going on.			//
		//They are detected after the conversions just in case anything happens in there.	//

		//This error would happen whenever the nativeUnits is negative, as we cant send cages down into forever!		//
		if (nativeUnits < 0) {
			if (!negativeToZero) {			//If it is not supposed to send anything too far down, it simply returns	//
				Log("ERROR: Do not attempt to send the Middle Cage to a negative height! (Minimum must be 0)", 1);
				cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
				return -1;
			}
			if (negativeToZero) { 			//If we want it like it, we instead send it to the min height, kinda logical//
				Log("WARNING: Do not attempt to send the Middle Cage to a negative height! (Minimum must be 0)", 2);
				Log("WARNING: Sending Middle Cage to 0 inches height instead", 3);
				nativeUnits = 0;
			}
		}
		//This error fires whenever the target height exceeds the maximum height, we cant send cages up endlessly!		//
		//Maximum height is calculated by the inches it can go, times 2 rotations per inch and then into native units.	//
		if (nativeUnits > maxMiddleCageEncoderNU) {
			if (!infinityToMax) {			//If we dont want it to move if this happens.								//
				Log("ERROR: Do not try to send the Middle Cage to infinity and beyond! (Exceeds Max Height)", 4);
				cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
				return -2;
			}
			if (infinityToMax) {			//If we dont want it to move if this happens.								//
				Log("WARNING: Do not try to send the Middle Cage to infinity and beyond! (Exceeds Max Height)", 5);
				Log("WARNING: Sending Middle Cage to max height instead", 6);
				nativeUnits = maxMiddleCageEncoderNU;

			}
		}

		if  ((nativeUnits > 4096) && upperMiddleSwitch.Get()) {
			middleCageMotorMain.Set(ControlMode::PercentOutput, 0);
			Log("ERROR: Upper Middle Cage Switch is Presssed, Stopping Middle Cage", 26);
			return -3;
		}
		if  ((nativeUnits < 4096) && lowerMiddleSwitch.Get()) {
			middleCageMotorMain.Set(ControlMode::PercentOutput, 0);
			Log("ERROR: Lower Middle Cage Switch is Presssed, Stopping Middle Cage", 27);
			return -3;
		}
		//std::cout << "TargetNativeUnits: " << nativeUnits << std::endl; //Uncomment to see the Target Native Units realtime
		middleCageMotorMain.Set(ControlMode::Position, nativeUnits);
		//Only the Main Motor is told to move to the value because  the other one is in Follow Mode.					//
		/*Feel like it should calculate some sort of difference in position? Click to Expand and know why not!
		//There is no need to calculate any kind of height difference because this position is absolute.	//
		//E.g. If you send first the motor to a height of 10 inches, the final position of the motor in		//
		//native units will be 81920 (10*2*4096), then, if you send it to 15 inches, the target position	//
		//in native units will be 122800 and the SRX calculates the difference itself. Likewise if you		//
		//send it to 5 inches, the target position in native units will be 40960 and the SRX will go to		//
		//that position no matter if it was originally at 0 native units or 20,000,000 native units.		*/

		return 0;
	}

	/*Sends commands the main cube cage motor to rotate the lead screws 		//
	//to where the cage reaches the targetHeightInches height.					/
	/targetHeightInches is the height the cube cage must go to.								//
	//negativeToZero, if set to true, will make negative target heights make the cage go to 0	//
	//infinityToMax, if set to true, will make heights greater than max height become the max	/
	/Negative numbers will result in an error (Returned value will be -1)						//
	//Note that this error will be ignored if negativeToZero is set to true						//
	//E.g. because zero is always the closest acceptable height to a negative, we have the 		//
	//option to if the height is for some reason neagtive, the cage targets 0 height instead.	/
	/Too large numbers will result in an error (Returned value will be -2)						//
	//Note that this error will be ignored if infinityToMax is set to true						//
	//E.g. because the max height is always the closest acceptable height to a larger target,	//
	//we have the option to if the height is for some reason greater than the max, the cage		//
	//targets its maximum height instead.														/
	/No error returns a 0														*/
	int SendCubeCageTo(double targetHeightInches, bool negativeToZero = false, bool infinityToMax = false) {
		double targetEncoderRevolutions = targetHeightInches * rotationsPerInch * encoderToLeadScrewCubeCageGearRatio;
														//Transforms the target inches into the number of revolutions	//
														//the motor has to target.										//
														//The current lead screw gives two revolutions to go up 1 inch.	//

		double nativeUnits = targetEncoderRevolutions * fullRotation;//The number of revolutions is transformed into native units.	//
														//There are 4096 native units per revolution.					//

		//These error codes will dectect whether there is something weird going on.			//
		//They are detected after the conversions just in case anything happens in there.	//

		//This error would happen whenever the nativeUnits is negative, as we cant send cages down into forever!		//
		if (nativeUnits < 0) {
			if (!negativeToZero) {			//If it is not supposed to send anything too far down, it simply returns	//
				Log("ERROR: Do not attempt to send the Cube Cage to a negative height! (Minimum must be 0)", 7);
				cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
				return -1;
			}
			if (negativeToZero) { 			//If we want it like it, we instead send it to the min height, kinda logical//
				Log("WARNING: Do not attempt to send the Cube Cage to a negative height! (Minimum must be 0)", 8);
				Log("WARNING: Sending Cube Cage to 0 inches height instead", 9);
				nativeUnits = 0;
			}
		}
		//This error fires whenever the target height exceeds the maximum height, we cant send cages up endlessly!		//
		//Maximum height is calculated by the inches it can go, times 2 rotations per inch and then into native units.	//
		if (nativeUnits > maxCubeCageEncoderNU) {
			if (!infinityToMax) {			//If we dont want it to move if this happens.								//
				Log("ERROR: Do not try to send the Cube Cage to infinity and beyond! (Exceeds Max Height)", 10);
				cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
				return -2;
			}
			if (infinityToMax) {			//If we dont want it to move if this happens.								//
				Log("WARNING: Do not try to send the Cube Cage to infinity and beyond! (Exceeds Max Height)", 11);
				Log("WARNING: Sending Cube Cage to max height instead", 12);
				nativeUnits = maxCubeCageEncoderNU;

			}
		}

		if  ((nativeUnits > 4096) && upperCubeSwitch.Get()) {
			cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
			Log("ERROR: Upper Cube Cage Switch is Presssed, Stopping Middle Cage", 28);
			return -3;
		}
		if  ((nativeUnits < 4096) && lowerCubeSwitch.Get()) {
			cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
			Log("ERROR: Lower Cube Cage Switch is Presssed, Stopping Middle Cage", 29);
			return -3;
		}
		cubeCageMotorMain.Set(ControlMode::Position, nativeUnits);
		//Only the Main Motor is told to move to the value because  the other one is in Follow Mode.					//
		/*Feel like it should calculate some sort of difference in position? Click to Expand and know why not!
		//There is no need to calculate any kind of height difference because this position is absolute.	//
		//E.g. If you send first the motor to a height of 10 inches, the final position of the motor in		//
		//native units will be 81920 (10*2*4096), then, if you send it to 15 inches, the target position	//
		//in native units will be 122800 and the SRX calculates the difference itself. Likewise if you		//
		//send it to 5 inches, the target position in native units will be 40960 and the SRX will go to		//
		//that position no matter if it was originally at 0 native units or 20,000,000 native units.		*/

		return 0;
	}

	////Lower Cage Functions////------------------------------------------------------------------------------------------|	O

	//Lower both Cages.
	int LowerCages() {
		LowerCubeCage();
		LowerMiddleCage();
		return 0;
	}

	/*As the name says, lowers the Cube Cage at a "constant" speed.			//
	//Speed Modified with the elevatorSpeed double.							*/
	//Error code -1 is that it is already at position 0 so it cant be lowered more.	//
	//No error returns a 0.															//
	int LowerCubeCage() {
		//Gets the current cage position.			//
		double currentEncoderNU =  cubeCageMotorMain.GetSensorCollection().GetQuadraturePosition();

		Log("WARNING: Step 0", 0);
		double currentCubeCagePositionInches = (currentEncoderNU / fullRotation) * ( 1.0 / encoderToLeadScrewCubeCageGearRatio) * ( 1.0 / rotationsPerInch);
		double cubeCageSliderValue = (currentCubeCagePositionInches  / cubeCageSpan) * maxSliderValue;
		SmartDashboard::PutNumber("DB/Slider 1", cubeCageSliderValue );

		if  (lowerCubeSwitch.Get()) {
			Log("WARNING: Step 1", 1);
			cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
			cubeCageMotorMain.SetSelectedSensorPosition(0, 0, 10);
			Log("ERROR: Lower Cube Cage Switch is Presssed, Stopping Middle Cage", 29);
			return -3;
		}

		//Half a rotation is a quarter of an inch, unncessary precision.
		if (currentEncoderNU > halfRotation) {
			Log("WARNING: Step 2", 2);//If the position is greater than half a turn,		//
			cubeCageMotorMain.Set(ControlMode::PercentOutput, -1);	//The cage lowers at a constant speed.				//
		} else if (currentEncoderNU > 0) {
			Log("WARNING: Step 3", 3);//But if the position is less than half a turn but greater than 0,	//
			cubeCageMotorMain.Set(ControlMode::Position, 0);						//The cage just lowers completely to position 0.					//
		} else {
			Log("WARNING: Step 4", 4);
			Log("ERROR: Do not attempt to lower the Cube Cage if it is already at the bottom!", 13);	//However, if the position is 0,								//
			cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);										//Then it logs an error because it cant really lower it more.	//
			return -1;
		}

		/*Note that the GetQuadraturePosition is not updated instantly always, thus if we wait till the GetQuadraturePosition reads a 0				//
		//to stop telling it to go lower, it might end up trying to go lower than 0 for a while. That is why when it is half a turn from			//
		//the minimum, or well, 0.25 inches from 0 height, we command it to go to 0 height instead.													//
		//E.g. The motor lets say its at position 8192 (2 Turns | 1 inch) and we tell it to lower, then it will lower at a constant speed			//
		//in PercentOutput mode into the position 2048 (0.5 Turns | 0.25 inches) and then, if we keep lowering it, it will change to				//
		//Position control mode and target position 0. If this didn't happen and we continued in PercentOutput control mode, then, because			//
		//it takes a while for the GetQuatradurePosition() to update to the real value, we might end up in the actual position 0 and telling		//
		//it to go to negative values with the PercentOutput, however because it is now targetting the 0 with Position control mdoe, it will		//
		//never go to a negative height.																											*/
		return 0;
	}

	/*As the name says, lowers the Middle Cage at a "constant" speed.			//
	//Speed Modified with the elevatorSpeed double.								*/
	//Error code -1 is that it is already at position 0 so it cant be lowered more.	//
	//No error returns a 0.															//
	int LowerMiddleCage() {
		//Gets the current cage position.			//
		double currentEncoderNU =  middleCageMotorMain.GetSensorCollection().GetQuadraturePosition();


		double currentMiddleCagePositionInches = (currentEncoderNU / fullRotation) * ( 1.0 / encoderToLeadScrewMiddleCageGearRatio) * ( 1.0 / rotationsPerInch);
		double middleCageSliderValue = (currentMiddleCagePositionInches  / middleCageSpan) * maxSliderValue;
		SmartDashboard::PutNumber("DB/Slider 0", middleCageSliderValue );

		//double currentCagePosition = middleCageMotorMain.GetSensorCollection().GetQuadraturePosition();

		if  (lowerMiddleSwitch.Get()) {
			middleCageMotorMain.Set(ControlMode::PercentOutput, 0);
			Log("ERROR: Lower Middle Cage Switch is Presssed, Stopping Middle Cage", 27);
			middleCageMotorMain.SetSelectedSensorPosition(0, 0, 10);
			return -3;
		}

		if (currentEncoderNU > halfRotation) { 								//If the position is greater than half a turn,		//
			middleCageMotorMain.Set(ControlMode::PercentOutput, -1);	//The cage lowers at a constant speed.				//
		} else if (currentEncoderNU > 0) {									//But if the position is less than half a turn but greater than 0,	//
			middleCageMotorMain.Set(ControlMode::Position, 0);						//The cage just lowers completely to position 0.					//
		} else {
			Log("ERROR: Do not attempt to lower the Middle Cage if it is already at the bottom!", 14);	//However, if the position is 0,								//
			middleCageMotorMain.Set(ControlMode::PercentOutput, 0);									//Then it logs an error because it cant really lower it more.	//
			return -1;
		}

		/*Note that the GetQuadraturePosition is not updated instantly always, thus if we wait till the GetQuadraturePosition reads a 0				//
		//to stop telling it to go lower, it might end up trying to go lower than 0 for a while. That is why when it is half a turn from			//
		//the minimum, or well, 0.25 inches from 0 height, we command it to go to 0 height instead.													//
		//E.g. The motor lets say its at position 8192 (2 Turns | 1 inch) and we tell it to lower, then it will lower at a constant speed			//
		//in PercentOutput mode into the position 2048 (0.5 Turns | 0.25 inches) and then, if we keep lowering it, it will change to				//
		//Position control mode and target position 0. If this didn't happen and we continued in PercentOutput control mode, then, because			//
		//it takes a while for the GetQuatradurePosition() to update to the real value, we might end up in the actual position 0 and telling		//
		//it to go to negative values with the PercentOutput, however because it is now targetting the 0 with Position control mdoe, it will		//
		//never go to a negative height.																											*/
		return 0;
	}

	////Raise Cage Functions////------------------------------------------------------------------------------------------|	O

	//Raises both Cages.
	int RaiseCages() {
		RaiseCubeCage();
		RaiseMiddleCage();
		return 0;
	}

	/*As the name says, raises the Cube Cage at a "constant" speed.			//
	//Speed Modified with the elevatorSpeed double.							*/
	//Error code -1 is that it is already at max position so it cant be raised more.	//
	//No error returns a 0.															//
	int RaiseCubeCage() {
		//Gets the current cage position.	(NU equals Native Units from the encoder)
		double currentEncoderNU =  cubeCageMotorMain.GetSensorCollection().GetQuadraturePosition();

		double currentCubeCagePositionInches = (currentEncoderNU / fullRotation) * ( 1.0 / encoderToLeadScrewCubeCageGearRatio) * ( 1.0 / rotationsPerInch);
		double cubeCageSliderValue = (currentCubeCagePositionInches  / cubeCageSpan) * maxSliderValue;
		SmartDashboard::PutNumber("DB/Slider 1", cubeCageSliderValue );

		if  (upperCubeSwitch.Get()) {
			cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
			Log("WARNING:  Step 2", 2);
			Log("ERROR: Upper Cube Cage Switch is Presssed, Stopping Cube Cage", 28);
			return -3;
		}

		if (currentEncoderNU < (maxCubeCageEncoderNU - halfRotation)) {
			Log("WARNING:  Step 3", 3);																			//If the position is less than half a turn under the max,	//
			cubeCageMotorMain.Set(ControlMode::PercentOutput, 1);			//The cage raises at a constant speed.						//
			cubeCageMotor.Set(ControlMode::PercentOutput, -1);			//The cage raises at a constant speed.						//
		} else if (currentEncoderNU < maxCubeCageEncoderNU) {					//But if the position is greater than half a turn under the max,	//
			Log("WARNING:  Step 4", 4);
			cubeCageMotorMain.Set(ControlMode::Position, maxCubeCageEncoderNU);		//The cage just completely raises into the max height.				//
		} else {
			Log("ERROR: Do not attempt to raise the Cube Cage if it is already at the top!", 15);	//However, if the position is the max height,					//
			cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);									//Then it logs an error because it cant really raise it more.	//
			cubeCageMotor.Set(ControlMode::PercentOutput, 0);									//Then it logs an error because it cant really raise it more.	//
			Log("WARNING:  Step 1", 1);
			//std::cout << "CurrentEncoderNU: " << currentEncoderNU << ", CurrentCubeCagePositionInches: " << currentCubeCagePositionInches << std::endl;
			return -1;
		}

		// If you want to know why the last half turn is using Position control mode, look at this comment in LowerMiddleCage() or LowerCubeCage().			//																										*/
		return 0;
	}

	/*As the name says, raises the Middle Cage at a "constant" speed.			//
	//Speed Modified with the elevatorSpeed double.								*/
	//Error code -1 is that it is already at max position so it cant be raised more.	//
	//No error returns a 0.															//
	int RaiseMiddleCage() {
		//Gets the current cage position.	(NU equals Native Units from the encoder)
		double currentEncoderNU =  middleCageMotorMain.GetSensorCollection().GetQuadraturePosition();

		double currentMiddleCagePositionInches = (currentEncoderNU / fullRotation) * ( 1.0 / encoderToLeadScrewMiddleCageGearRatio) * ( 1.0 / rotationsPerInch);
		double middleCageSliderValue = (currentMiddleCagePositionInches  / middleCageSpan) * maxSliderValue;
		SmartDashboard::PutNumber("DB/Slider 0", middleCageSliderValue );

		if  (upperMiddleSwitch.Get()) {
			middleCageMotorMain.Set(ControlMode::PercentOutput, 0);
			Log("ERROR: Upper Middle Cage Switch is Presssed, Stopping Middle Cage", 26);
			return -3;
		}

		if (currentEncoderNU < (maxMiddleCageEncoderNU - halfRotation)) {
																						//If the position is less than half a turn under the max,	//
			middleCageMotorMain.Set(ControlMode::PercentOutput, 1);			//The cage raises at a constant speed.						//
		} else if (currentEncoderPositionInches < maxMiddleCageEncoderNU) {					//But if the position is greater than half a turn under the max,	//
			middleCageMotorMain.Set(ControlMode::Position, maxMiddleCageEncoderNU);	//The cage just completely raises into the max height.				//
		} else {
			Log("ERROR: Do not attempt to raise the Middle Cage if it is already at the top!", 16);	//However, if the position is the max height,					//
			middleCageMotorMain.Set(ControlMode::PercentOutput, 0);									//Then it logs an error because it cant really raise it more.	//
			return -1;
		}

		// If you want to know why the last half turn is using Position control mode, look at this comment in LowerMiddleCage() or LowerCubeCage().			//																										*/
		return 0;
	}



	////Cube Handling Functions////---------------------------------------------------------------------------------------|	O

	//Activates the cube gathering tank track like things and stops when the limit switch is pressed.	//
	int AbsorbCube() {
		//Detects the limit switch that detects the cube collision with the back of the cage.
		if (!cubeInputSwitch.Get() || cubeInputSwitch.Get()) { //If the cube has not activated the limit switch at the back of the cube cage.		//
			cubeGathererMotorRight.Set(ControlMode::Velocity, 512); //Activates the motors, the value might be mutiplied by -1 if it is spitting the cube instead.
			cubeGathererMotorLeft.Set(ControlMode::Velocity, 512); //Activates the motors, the value might be mutiplied by -1 if it is spitting the cube instead.
		} else {
			cubeGathererMotorRight.Set(ControlMode::PercentOutput, 0);
			cubeGathererMotorLeft.Set(ControlMode::PercentOutput, 0);
			Log("WARNING: Cube completely inside the cube cage.", 17);
		}
		return 0;
	}

	//Activates the cube gathering tank tracks.
	int EjectCube() {
		cubeGathererMotorRight.Set(ControlMode::Velocity, -512); 	//Activates the motors, the value might be mutiplied by -1 if it is sucking the cube instead.
		cubeGathererMotorLeft.Set(ControlMode::Velocity, -512); 	//Activates the motors, the value might be mutiplied by -1 if it is sucking the cube instead.
																	//Also this might be too fast and launch the cubes way too hard.
		return 0;
	}


	////Misc Functions////------------------------------------------------------------------------------------------------|	O

	//Just to log things, shorter than writing the whole std::cout and std::endl every time.
	void Log(std::string log, int messageId) {
		return;
		//If the messageId is negative, it ignores the timer. If it is positive or 0, it checks if the message has been called in the near past.	//
		if (messageId < 0) { std::cout << log << std::endl; } else if (errorLogChronomanager[messageId] < loopNow) {
			std::cout << log << std::endl; 						//If the particular message has not been called in 30 loops, it runs.		//
			errorLogChronomanager[messageId] = (loopNow + loopWait);	//The loops timer is reset.												//
		}

		//But... why such an odd code to do a simple print?
		//Well, the previous code was only the print message, but the same message could be called every frame and it annoyed a lot.
		//Now, the same message will only be loged once after at least 30 loops. But, while one message is waiting to be available again,
		//other messages are not impeded from triggering so they will trigger instantly as fast as they are called.
		//In other words, each error message has its own individual timer. All the timers are for 30 loops.
	}

	//Sets all values in the errorLogChronomanager Array to 0
	void ResetErrorTimers() {
		for (int i = 0; i < errorLogChronomanagerLength; i++) { errorLogChronomanager[i] = 0; }
	}

	void ResearchForVoid() {
		if (!lowerMiddleSwitch.Get()) {
			middleCageMotorMain.Set(ControlMode::PercentOutput, -1);
		} else {
			middleCageMotorMain.SetSelectedSensorPosition(0, 0, 10);
			Log("WARNING: Middle Cage Sensor position set to 0", 31);
			middleCageMotorMain.Set(ControlMode::PercentOutput, 0);
			middleZero = true;
		}
		if (middleZero) {
			if (!lowerCubeSwitch.Get()) {
				cubeCageMotorMain.Set(ControlMode::PercentOutput, -1);
			} else {
				cubeCageMotorMain.SetSelectedSensorPosition(0, 0, 10);
				Log("WARNING: Cube Cage Sensor position set to 0", 30);
				cubeCageMotorMain.Set(ControlMode::PercentOutput, 0);
				cubeZero = true;
			}
		}
		hasHitTheLimit = (cubeZero && middleZero);
	}

private:
};
START_ROBOT_CLASS(Robot)
