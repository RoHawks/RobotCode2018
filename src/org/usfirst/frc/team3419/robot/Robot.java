package org.usfirst.frc.team3419.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import constants.DriveConstants;
import constants.Ports;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import robotcode.driving.Wheel;
import robotcode.driving.DriveTrain;
import robotcode.driving.SwerveDrive;
import sensors.TalonAbsoluteEncoder;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
@SuppressWarnings("deprecation")
public class Robot extends SampleRobot {

	private XboxController mController = new XboxController(0);

	private DriveTrain mDriveTrain;

	private Wheel[] mWheel = new Wheel[4];
	private WPI_TalonSRX[] mTurn = new WPI_TalonSRX[4];
	private WPI_TalonSRX[] mDrive = new WPI_TalonSRX[4];
	private TalonAbsoluteEncoder[] mEncoder = new TalonAbsoluteEncoder[4];

	private WPI_TalonSRX mLeft;
	private WPI_TalonSRX mRight;

	private AHRS mNavX;

	public Robot() {
	}

	@Override
	public void robotInit() {
		for (int i = 0; i < 4; i++) {

			mTurn[i] = new WPI_TalonSRX(Ports.TURN[i]);
			mTurn[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
			mTurn[i].setNeutralMode(NeutralMode.Brake);
			mTurn[i].setSensorPhase(DriveConstants.Modules.TALON_ENCODER_REVERSED[i]);//commented for non talon
			mTurn[i].setInverted(DriveConstants.Modules.TALON_TURN_INVERTED[i]); //commented for non talon
			mTurn[i].config_kP(0, 0.2/*DriveConstants.PID_Constants.ROTATION_P[i]*/, 10);
			mTurn[i].config_kI(0, 0.002/*DriveConstants.PID_Constants.ROTATION_I[i]*/, 10);
			mTurn[i].config_kD(0, DriveConstants.PID_Constants.ROTATION_D[i], 10);
			mTurn[i].config_IntegralZone(0, DriveConstants.PID_Constants.ROTATION_IZONE[i], 10);

			mDrive[i] = new WPI_TalonSRX(Ports.DRIVE[i]);
			mDrive[i].setInverted(DriveConstants.Modules.INVERTED[i]);

			mLeft = new WPI_TalonSRX(Ports.LEFT_INTAKE);
			mRight = new WPI_TalonSRX(Ports.RIGHT_INTAKE);

			mEncoder[i] = new TalonAbsoluteEncoder(mTurn[i],
					DriveConstants.Modules.TALON_ENCODER_REVERSED[i]/*
																 * Not sure
																 * about this
																 */,
					ResourceFunctions.tickToAngle(DriveConstants.Modules.OFFSET[i]));
			// Offset needs to be in degrees
			mWheel[i] = new Wheel(mTurn[i], mDrive[i], mEncoder[i], DriveConstants.Modules.TALON_TURN_INVERTED[i]);
		}

		mNavX = new AHRS(Port.kUSB);

		mDriveTrain = new DriveTrain(mWheel, mController, mNavX);

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomous() {
	}

	public void operatorControl() {
		// CrabDrive();
		//SwerveDrive();
		// TankDrive();
		TalonPIDTest();
	}

	public void TankDrive() {
		while (isOperatorControl() && isEnabled()) {
			mDriveTrain.driveTank();
		}
	}

	/**
	 * Runs the motors with arcade steering.
	 */

	public void CrabDrive() {
		double intakeSpeed = 0;
		boolean isIntaking = false;
		double x;
		double y;
		double joystickAngle = 0;

		while (isOperatorControl() && isEnabled()) {
			x = mController.getX(Hand.kLeft);
			y = -1 * mController.getY(Hand.kLeft);

			double triggerValue = mController.getTriggerAxis(Hand.kRight);
			double linearVelocity = Math.pow(triggerValue, 2); // Quadratic
																// velocity
			SmartDashboard.putNumber("TriggerAxis", triggerValue);

			if (Math.abs(y) >= DriveConstants.MIN_DIRECTION_MAG || Math.abs(x) >= DriveConstants.MIN_DIRECTION_MAG) {
				joystickAngle = -Math.toDegrees(Math.atan2(y, x)) + 90;
				joystickAngle = ResourceFunctions.putAngleInRange(joystickAngle); // puts
																					// angle
																					// between
																					// zero
																					// and
																					// 360
				SmartDashboard.putNumber("Joystick angle", joystickAngle);
				// for (Wheel wheel : mWheel) {
				// // Max speed is set in the wheel class
				// wheel.setAngle(joystickAngle);
				// }
			}
			// else{
			// for (WPI_TalonSRX turn : mTurn){
			// turn.set(0);
			// }
			// }

			for (Wheel wheel : mWheel) {
				wheel.setLinearVelocity(linearVelocity); // Max speed is set in
															// the wheel class
				wheel.setAngle(joystickAngle);
			}

			// mDriveTrain.enactMovement();

			if (mController.getAButtonReleased()) {
				isIntaking = !isIntaking;
			}
			mRight.set(ControlMode.PercentOutput, isIntaking ? -DriveConstants.RIGHT_INTAKE_SPEED : 0);
			mLeft.set(ControlMode.PercentOutput, isIntaking ? DriveConstants.LEFT_INTAKE_SPEED : 0);

			if (mController.getBumper(Hand.kLeft)) {
				for (Wheel wheel : mWheel) {
					wheel.setTurnSpeedToZero();
					wheel.setLinearVelocity(0);
				}
			}

			SmartDashboard.putNumber("right intake speed", DriveConstants.RIGHT_INTAKE_SPEED);
			SmartDashboard.putNumber("left intake speed", DriveConstants.LEFT_INTAKE_SPEED);

			SmartDashboard.putNumber("angle 0", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[0].getSelectedSensorPosition(0))));
			SmartDashboard.putNumber("angle 1", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[1].getSelectedSensorPosition(0))));
			SmartDashboard.putNumber("angle 2", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[2].getSelectedSensorPosition(0))));
			SmartDashboard.putNumber("angle 3", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[3].getSelectedSensorPosition(0))));

			Timer.delay(0.005); // wait for a motor update time
		}
	}

	public void SwerveDrive() {
		double intakeSpeed = 0;
		boolean isIntaking = false;
		while (isOperatorControl() && isEnabled()) {
			mDriveTrain.enactMovement();

			if (mController.getAButtonReleased()) {
				isIntaking = !isIntaking;
			}
			mRight.set(ControlMode.PercentOutput, isIntaking ? -DriveConstants.RIGHT_INTAKE_SPEED : 0);
			mLeft.set(ControlMode.PercentOutput, isIntaking ? DriveConstants.LEFT_INTAKE_SPEED : 0);

			if (mController.getBumper(Hand.kLeft)) {
				for (Wheel wheel : mWheel) {
					wheel.setTurnSpeedToZero();
					wheel.setLinearVelocity(0);
				}
			}

			SmartDashboard.putNumber("right intake speed", DriveConstants.RIGHT_INTAKE_SPEED);
			SmartDashboard.putNumber("left intake speed", DriveConstants.LEFT_INTAKE_SPEED);

			SmartDashboard.putNumber("angle 0", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[0].getSelectedSensorPosition(0))));
			SmartDashboard.putNumber("angle 1", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[1].getSelectedSensorPosition(0))));
			SmartDashboard.putNumber("angle 2", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[2].getSelectedSensorPosition(0))));
			SmartDashboard.putNumber("angle 3", ResourceFunctions
					.putAngleInRange(ResourceFunctions.tickToAngle(mTurn[3].getSelectedSensorPosition(0))));

			SmartDashboard.putNumber("robot angle", mNavX.getAngle());
			SmartDashboard.putNumber("NavX temp", mNavX.getTempC());

			Timer.delay(0.005); // wait for a motor update time
		}
	}

	public void TalonPIDTest() {
		//double speed = 0.85;
		double joystickAngle = 0;
		while (isOperatorControl() && isEnabled()) {
			for (int i = 0; i < 4; i++) {
				//mTurn[i].setSelectedSensorPosition(mTurn[i].getSelectedSensorPosition(0) % 4096, 0, 10);
				double x = mController.getX(Hand.kLeft);
				double y = -mController.getY(Hand.kLeft);
				SmartDashboard.putNumber("x-value", x);
				SmartDashboard.putNumber("y-value", y);
				if (Math.abs(y) >= DriveConstants.MIN_DIRECTION_MAG || Math.abs(x) >= DriveConstants.MIN_DIRECTION_MAG) {
					joystickAngle = -Math.toDegrees(Math.atan2(y, x)) + 90;
					joystickAngle = ResourceFunctions.putAngleInRange(joystickAngle); //puts angle between zero and 360
				}
				double triggerValue = mController.getTriggerAxis(Hand.kRight);
				double linearVelocity = Math.pow(triggerValue, 2);
				mWheel[i].setAngleTalon(joystickAngle);
				mWheel[i].setLinearVelocity(linearVelocity);
			}
			SmartDashboard.putNumber("angle 0", mTurn[0].getSelectedSensorPosition(0));
			SmartDashboard.putNumber("angle 1", mTurn[1].getSelectedSensorPosition(0));
			SmartDashboard.putNumber("angle 2", mTurn[2].getSelectedSensorPosition(0));
			SmartDashboard.putNumber("angle 3", mTurn[3].getSelectedSensorPosition(0));
		}

	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}

	public void disabled() {

	}
}
