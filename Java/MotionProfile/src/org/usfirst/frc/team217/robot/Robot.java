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
	TalonSRX _talon = new TalonSRX(1); //right
	TalonSRX _talon2 = new TalonSRX(2);
	
	TalonSRX _talon3 = new TalonSRX(3);//left
	TalonSRX _talon4 = new TalonSRX(4);
	/** some example logic on how one can manage an MP */
	MotionProfileExample _example = new MotionProfileExample(_talon3, _talon);
	
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

		_talon3.setNeutralMode(NeutralMode.Brake);
		_talon4.setNeutralMode(NeutralMode.Brake);
		
		_talon.setNeutralMode(NeutralMode.Brake);
		_talon2.setNeutralMode(NeutralMode.Brake);
		
		_talon3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon3.setSensorPhase(false);
		_talon3.setInverted(false);
		_talon4.setInverted(false);
		
		_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon.setSensorPhase(false);
		_talon.setInverted(true);
		_talon2.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon3.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon3.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		
		/* set the peak and nominal outputs */
		_talon3.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		_talon3.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon3.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon3.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon3.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		_talon4.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		_talon4.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon4.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon4.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon4.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		_talon2.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		_talon2.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon2.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon2.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon2.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		/* set closed loop gains in slot0 - see documentation */
		_talon3.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon3.config_kF(0, 0.9728, Constants.kTimeoutMs);//0.9728 left side
		_talon3.config_kP(0, 2.472, Constants.kTimeoutMs);//2.472 left side 
		_talon3.config_kI(0, 0, Constants.kTimeoutMs);
		_talon3.config_kD(0, 24.72, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		_talon3.configMotionCruiseVelocity(788, Constants.kTimeoutMs);
		_talon3.configMotionAcceleration(788, Constants.kTimeoutMs);
		/* zero the sensor */
		_talon3.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(0, 0.9728, Constants.kTimeoutMs);//0.9728 left side
		_talon.config_kP(0, 2.472, Constants.kTimeoutMs);//2.472 left side 
		_talon.config_kI(0, 0, Constants.kTimeoutMs);
		_talon.config_kD(0, 24.72, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(788, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(788, Constants.kTimeoutMs);
		/* zero the sensor */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		
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

		_talon2.set(ControlMode.Follower, _talon.getDeviceID());
		_talon4.set(ControlMode.Follower, _talon3.getDeviceID());
		/* Check button 5 (top left shoulder on the logitech gamead). */
		if (btns[5] == false) {
			/*
			 * If it's not being pressed, just do a simple drive. This could be
			 * a RobotDrive class or custom drivetrain logic. The point is we
			 * want the switch in and out of MP Control mode.
			 */

			/* button5 is off so straight drive */
			_talon.set(ControlMode.PercentOutput, leftYjoystick*leftYjoystick);
			_talon3.set(ControlMode.PercentOutput, leftYjoystick*leftYjoystick);
			
			_example.reset();
			
		} else {
			/*
			 * Button5 is held down so switch to motion profile control mode =>
			 * This is done in MotionProfileControl. When we transition from
			 * no-press to press, pass a "true" once to MotionProfileControl.
			 */

			SetValueMotionProfile setOutput = _example.getSetValue();

			_talon.set(ControlMode.MotionProfile, setOutput.value);
			_talon3.set(ControlMode.MotionProfile, setOutput.value);

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
		_talon.set(ControlMode.PercentOutput, 0);
		_talon2.set(ControlMode.PercentOutput, 0);
		/* clear our buffer and put everything into a known state */
		_example.reset();
	}
}
