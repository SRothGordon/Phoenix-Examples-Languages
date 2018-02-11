/**
 * Example demonstrating the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * and followed the walk-through in the Talon SRX Software Reference Manual,
 * use button1 to motion-magic servo to target position specified by the gamepad stick.
 */
package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends IterativeRobot {
	TalonSRX _talon = new TalonSRX(3);
	TalonSRX _talon4 = new TalonSRX(4);
	TalonSRX _talon1 = new TalonSRX(1);
	TalonSRX _talon2 = new TalonSRX(2);
	Joystick _joy = new Joystick(0);
	StringBuilder _sb = new StringBuilder();

	public void robotInit() {

		/* first choose the sensor */
		
		_talon.setNeutralMode(NeutralMode.Brake);
		_talon4.setNeutralMode(NeutralMode.Brake);
		
		_talon1.setNeutralMode(NeutralMode.Brake);
		_talon2.setNeutralMode(NeutralMode.Brake);
		
		_talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon.setSensorPhase(false);
		_talon.setInverted(false);
		_talon4.setInverted(false);
		
		_talon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_talon1.setSensorPhase(false);
		_talon1.setInverted(true);
		_talon2.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		_talon1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		
		/* set the peak and nominal outputs */
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		_talon1.configNeutralDeadband(0.001, Constants.kTimeoutMs);
		_talon1.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon1.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

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
		
		_talon1.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon1.config_kF(0, 0.9728, Constants.kTimeoutMs);//0.9728 left side
		_talon1.config_kP(0, 2.472, Constants.kTimeoutMs);//2.472 left side 
		_talon1.config_kI(0, 0, Constants.kTimeoutMs);
		_talon1.config_kD(0, 24.72, Constants.kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		_talon1.configMotionCruiseVelocity(788, Constants.kTimeoutMs);
		_talon1.configMotionAcceleration(788, Constants.kTimeoutMs);
		/* zero the sensor */
		_talon1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* get gamepad axis - forward stick is positive */
		_talon4.set(ControlMode.Follower, _talon.getDeviceID());
		_talon2.set(ControlMode.Follower, _talon1.getDeviceID());
		
		double leftYstick = -1.0 * _joy.getY();
		
		leftYstick*=Math.abs(leftYstick);
		
		/* calculate the percent motor output */
		double motorOutput = _talon.getMotorOutputPercent();
		double motorOutput1 = _talon1.getMotorOutputPercent();
		
		/* prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		
		_sb.append("\tOut1%:");
		_sb.append(motorOutput1);
		_sb.append("\tVel1:");
		_sb.append(_talon1.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		
		if (_joy.getRawButton(1)) {
			/* Motion Magic - 4096 ticks/rev * 10 Rotations in either direction */
			double targetPos = leftYstick * 1000 * 3.0;
			_talon.set(ControlMode.MotionMagic, targetPos);
			_talon1.set(ControlMode.MotionMagic, targetPos);
			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			
			_sb.append("\terr1:");
			_sb.append(_talon1.getClosedLoopError(Constants.kPIDLoopIdx));
			
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else {
			/* Percent voltage mode */
			_talon.set(ControlMode.PercentOutput, leftYstick);
			_talon1.set(ControlMode.PercentOutput, leftYstick);
		}
		/* instrumentation */
		Instrum.Process(_talon, _sb);
		try {
			TimeUnit.MILLISECONDS.sleep(10);
		} catch (Exception e) {
		}
	}
}
