/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.
 *
 * 2018_06_06:  Test PWM motor controller with a servo.  I'm trying to use
 * PWMSpeedController, but it doesn't compile.  The constructor seems to be
 * protected, and I get errors no matter how I try to use it.
 *----------------------------------------------------------------------------*/

#include <IterativeRobot.h>
#include <Joystick.h>
#include <Spark.h>
#include <PWMSpeedController.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>



/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together.
 */
class Robot : public frc::IterativeRobot {

public:
	void TeleopInit() override {
		SmartDashboard::PutNumber("DB/Slider 1", m_stick.GetY());
		//Easy to miss:  Function/method calls must be inside the methods (curly braces)
	}//end TeleopInit()

	void TeleopPeriodic() override {
		m_tiltMotor.Set(m_stick.GetY());								//tilts servo
		//m_tiltMotor.Get()
		//continuousServo.Set(m_stick.GetY());
		SmartDashboard::PutNumber("DB/Slider 1", m_stick.GetY() );		//displays y postion
		m_panMotor.Set(m_stick.GetZ());									//pans servo
		SmartDashboard::PutNumber("DB/Slider 2", m_stick.GetZ() );		//displays z position
	}
	// void TeleopPeriodic() override { continuousServo.Set(m_stick.GetY()); }
	//void TeleopPeriodic() override { continuousServo->Set(m_stick.GetY()); }  //Pointer version.

	// frc::PWMSpeedController continuousServo(0);  //The non-pointer version doesn't compile either.
	// frc::PWMSpeedController* continuousServo = new frc::PWMSpeedController(0);  //Pointer version.
			// The pointer version generates "frc::PWMSpeedConttroller::PWMSpeedController(int) is protected"

private:
	frc::Joystick m_stick{0};
	frc::Spark m_tiltMotor{0};
	frc::Spark m_panMotor{1};
	//frc::PWMSpeedController continuousServo(0);
};

START_ROBOT_CLASS(Robot)
