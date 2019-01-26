/**
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The CANTalon class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is an IterativeRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * or find Motion Profile Generator.xlsx in the Project folder.
 * 
 * Logitech Gamepad mapping, use left y axis to drive Talon normally.  
 * Press and hold top-left-shoulder-button5 to put Talon into motion profile control mode.
 * This will start sending Motion Profile to Talon while Talon is neutral. 
 * 
 * While holding top-left-shoulder-button5, tap top-right-shoulder-button6.
 * This will signal Talon to fire MP.  When MP is done, Talon will "hold" the last setpoint position
 * and wait for another button6 press to fire again.
 * 
 * Release button5 to allow PercentOutput control with left y axis.
 */

package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {

	/** The Talon we want to motion profile. */
	TalonSRX rightFrontMotor= new TalonSRX(13); //right
	TalonSRX rightRearMotor = new TalonSRX(14);
	
	TalonSRX leftFrontMotor = new TalonSRX(11);//left
	TalonSRX leftRearMotor = new TalonSRX(12);
	/** some example logic on how one can manage an MP */
	MotionProfileExample _example = new MotionProfileExample(leftFrontMotor, rightFrontMotor);
	
	/** joystick for testing */
	Joystick _joy = new Joystick(0);

	/**
	 * cache last buttons so we can detect press events. In a command-based
	 * project you can leverage the on-press event but for this simple example,
	 * lets just do quick compares to prev-btn-states
	 */
	boolean[] _btnsLast = {false, false, false, false, false, false, false, false, false, false};

	/** run once after booting/enter-disable */
	public void disabledInit() {

		leftFrontMotor.setNeutralMode(NeutralMode.Brake);
		leftRearMotor.setNeutralMode(NeutralMode.Brake);
		
		rightFrontMotor.setNeutralMode(NeutralMode.Brake);
		rightRearMotor.setNeutralMode(NeutralMode.Brake);
		
		leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		leftFrontMotor.setSensorPhase(false);
		leftFrontMotor.setInverted(true);
		leftRearMotor.setInverted(true);
		
		rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		rightFrontMotor.setSensorPhase(false);
		rightFrontMotor.setInverted(false);
		rightRearMotor.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		rightRearMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		
		/* set the peak and nominal outputs */
		leftFrontMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		leftFrontMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		leftFrontMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		leftFrontMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		leftFrontMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		rightFrontMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		rightFrontMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightFrontMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightFrontMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightFrontMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		leftRearMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		leftRearMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		leftRearMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		leftRearMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		leftRearMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		rightRearMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		rightRearMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		rightRearMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		rightRearMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		rightRearMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		/* set closed loop gains in slot0 - see documentation */
		leftFrontMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		leftFrontMotor.config_kF(0, 1023/9926.8, Constants.kTimeoutMs);//0.9728 left side
		leftFrontMotor.config_kP(0, 0.6704, Constants.kTimeoutMs);//2.472 left side 
		leftFrontMotor.config_kI(0, 0, Constants.kTimeoutMs);
		leftFrontMotor.config_kD(0, 10230/9926.8, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		leftFrontMotor.configMotionCruiseVelocity((int) (.8*9926.8), Constants.kTimeoutMs);
		leftFrontMotor.configMotionAcceleration((int) (4346/1.5), Constants.kTimeoutMs);
		/* zero the sensor */
		leftFrontMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		rightFrontMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		rightFrontMotor.config_kF(0, 1023/9926.8, Constants.kTimeoutMs);//0.9728 left side
		rightFrontMotor.config_kP(0, 0.6704, Constants.kTimeoutMs);//2.472 left side 
		rightFrontMotor.config_kI(0, 0, Constants.kTimeoutMs);
		rightFrontMotor.config_kD(0, 10230/9926.8, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		rightFrontMotor.configMotionCruiseVelocity((int) (0.8*9926.8), Constants.kTimeoutMs);
		rightFrontMotor.configMotionAcceleration((int) (4346/1.5), Constants.kTimeoutMs);
		/* zero the sensor */
		rightFrontMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		
	}

	/** function is called periodically during operator control */
	public void teleopPeriodic() {
		/* get buttons */
		
		boolean[] btns = new boolean[_btnsLast.length];
		for (int i = 1; i < _btnsLast.length; ++i)
			btns[i] = _joy.getRawButton(i);

		/* get the left joystick axis on Logitech Gampead */
		double leftYjoystick = -1 * _joy.getY(); /* multiple by -1 so joystick forward is positive */

		/*
		 * call this periodically, and catch the output. Only apply it if user
		 * wants to run MP.
		 */
		_example.control();

		rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());
		leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());
		// Check button 5 (top left shoulder on the logitech gamead). 
		if (btns[5] == false) {
			/*
			 * If it's not being pressed, just do a simple drive. This could be
			 * a RobotDrive class or custom drivetrain logic. The point is we
			 * want the switch in and out of MP Control mode.
			 */

			/* button5 is off so straight drive */
			rightFrontMotor.set(ControlMode.PercentOutput, leftYjoystick*leftYjoystick);
			leftFrontMotor.set(ControlMode.PercentOutput, leftYjoystick*leftYjoystick);
			
			_example.reset();
			
		} else {
			/*
			 * Button5 is held down so switch to motion profile control mode =>
			 * This is done in MotionProfileControl. When we transition from
			 * no-press to press, pass a "true" once to MotionProfileControl.
			 */

			SetValueMotionProfile setOutput = _example.getSetValue();

			rightFrontMotor.set(ControlMode.MotionProfile, setOutput.value);
			leftFrontMotor.set(ControlMode.MotionProfile, setOutput.value);

			/*
			 * if btn is pressed and was not pressed last time, In other words
			 * we just detected the on-press event. This will signal the robot
			 * to start a MP
			 */
			if ((btns[6] == true) && (_btnsLast[6] == false)) {
				/* user just tapped button 6 */

				// --- We could start an MP if MP isn't already running ----//
				_example.startMotionProfile();
			}
		}

		/* save buttons states for on-press detection */
		for (int i = 1; i < 10; ++i)
			_btnsLast[i] = btns[i];

	}

	/** function is called periodically during disable */
	public void disabledPeriodic() {
		/*
		 * it's generally a good idea to put motor controllers back into a known
		 * state when robot is disabled. That way when you enable the robot
		 * doesn't just continue doing what it was doing before. BUT if that's
		 * what the application/testing requires than modify this accordingly
		 */
		rightFrontMotor.set(ControlMode.PercentOutput, 0);
		rightRearMotor.set(ControlMode.PercentOutput, 0);
		/* clear our buffer and put everything into a known state */
		_example.reset();
	}
}
