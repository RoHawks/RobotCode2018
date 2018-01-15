/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3419.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import sensors.RotationInputter;
import sensors.TalonAbsoluteEncoder;
import sensors.USDigitalMA3Encoder;

public class RobotTest extends SampleRobot {

	private WPI_TalonSRX[] talonArray;
	private XboxController mController;

	public RobotTest() {

	}

	@Override
	public void robotInit() {
		talonArray = new WPI_TalonSRX[20];
		for (int i = 0; i < 20; i++) {
			talonArray[i] = new WPI_TalonSRX(i);
		}
		mController = new XboxController(0);
		for (int i = 0; i < 4; i++) {
			// talonArray[2*i+1].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
			// 0, 10);
		}
	}

	@Override
	public void autonomous() {

	}

	@Override
	public void operatorControl() {
		int count = 0;
/*		talonArray[0].config_kP(0, 0.05, 10);
		talonArray[0].config_kI(0, 0.05, 10);
		talonArray[0].config_kD(0, 0.05, 10);
		talonArray[0].config_kF(0, 0.05, 10);
		talonArray[0].\*/
		while (isOperatorControl() && isEnabled()) {
//			if(mController.getYButton()){
//				for(int i = 0 ; i < 4; i++){
//					talonArray[2*i+1].set(0.5);
//				}
//			}
//			if(mController.getBackButton()){
//				for(int i = 0 ; i < 4; i++){
//					talonArray[2*i].set(0.5);
//				}
//			}
//			if (mController.getAButtonReleased()) {
//				count+=2;
//			}
//			if(mController.getStartButton()){
//				talonArray[count%8].set(0.5);
//			}
			//talonArray[count].set(ControlMode.PercentOutput, Math.pow(mController.getTriggerAxis(Hand.kRight), 2));
			SmartDashboard.putNumber("Count", count);
//			if (mController.getBButton()) {
//				talonArray[0].set(ControlMode.Position, 2049);
//			}
			if (mController.getXButton()) {

				talonArray[8].set(ControlMode.PercentOutput, 0.7);
				// talonArray[].set(ControlMode.PercentOutput,
				talonArray[11].set(ControlMode.PercentOutput, -0.7);
			}
			else { 
				//talonArray[8].set(ControlMode.PercentOutput, Math.pow(mController.getTriggerAxis(Hand.kRight), 2));
				// talonArray[].set(ControlMode.PercentOutput,
				//talonArray[11].set(ControlMode.PercentOutput, -1* Math.pow(mController.getTriggerAxis(Hand.kRight), 2));
				talonArray[8].set(ControlMode.PercentOutput, 0.7 * Math.pow(mController.getTriggerAxis(Hand.kLeft), 2));
				// talonArray[].set(ControlMode.PercentOutput,
				talonArray[11].set(ControlMode.PercentOutput, -1 * Math.pow(mController.getTriggerAxis(Hand.kLeft), 2));
			}
			// The motors will be updated every 5ms
			Timer.delay(0.005);
		}
	}

	/**
	 * Runs during test mode.
	 */
	@Override
	public void test() {
	}
}
