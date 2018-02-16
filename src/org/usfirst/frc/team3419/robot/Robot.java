package org.usfirst.frc.team3419.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SampleRobot;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import constants.DriveConstants;
import constants.IntakeConstants;
import constants.Ports;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import robotcode.driving.Wheel;
import robotcode.systems.DoubleSolenoidReal;
import robotcode.systems.Intake;
import robotcode.systems.SolenoidInterface;
import robotcode.driving.DriveTrain;
import sensors.RobotAngle;
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

	private XboxController mController = new XboxController(Ports.XBOX);
	private Joystick mJoystick = new Joystick(Ports.JOYSTICK);

	private DriveTrain mDriveTrain;

	private Wheel[] mWheel = new Wheel[4];
	private WPI_TalonSRX[] mTurn = new WPI_TalonSRX[4];
	private WPI_TalonSRX[] mDrive = new WPI_TalonSRX[4];
	private TalonAbsoluteEncoder[] mEncoder = new TalonAbsoluteEncoder[4];

	private WPI_TalonSRX mIntakeTalon;

	private AHRS mNavX;
	private RobotAngle mRobotAngle;
	
	private PowerDistributionPanel mPDP;
	private Compressor mCompressor;
//	private boolean mShouldRunCompressor = false;
	private SolenoidInterface mLeft, mRight;

	private DigitalInput mLimitSwitch, mBreakbeam;
	
	private Thread mIntakeThread;
	private Intake mIntake;
	
	private boolean mInGame = false;

	public Robot() {
	}

	@Override
	public void robotInit() {
		mNavX = new AHRS(Ports.NAVX);
		mCompressor = new Compressor(Ports.COMPRESSOR);
		DriveInit();
		IntakeInit();
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

	/**
	 * Runs the motors with arcade steering.
	 */
	public void operatorControl() {
		startGame();
		while (isOperatorControl() && isEnabled()) {
			mIntake.setWheelSpeed();
			SwerveDrive();
			// TankDrive();	
			// CrabDrive();
			if (mPDP.getVoltage() < DriveConstants.EMERGENCY_VOLTAGE) {
				for (int i = 0; i < 4; i++) {
					mTurn[i].configPeakOutputForward(DriveConstants.MAX_EMERGENCY_VOLTAGE, 10);
					mTurn[i].configPeakOutputReverse(-DriveConstants.MAX_EMERGENCY_VOLTAGE, 10);

					mDrive[i].configPeakOutputForward(DriveConstants.MAX_EMERGENCY_VOLTAGE, 10);
					mDrive[i].configPeakOutputReverse(-DriveConstants.MAX_EMERGENCY_VOLTAGE, 10);
				}
			}
			Timer.delay(0.005); // wait for a motor update time
		}
	}

	public void startGame() {
		if (!mInGame) {
			if (RunConstants.RUNNING_INTAKE) {
				mIntake.enable();
				mIntakeThread = new Thread(mIntake);
				mIntakeThread.start();
			}
			if (RunConstants.RUNNING_PNEUMATICS) {
				mCompressor.start();
			} else {
				mCompressor.stop();
			}
			mInGame = true;
		}
	}
	
	public void endGame() {
		if(mInGame){
			mIntakeThread.interrupt();
			mCompressor.stop();
			mInGame = false;
		}
	}
	
	@Override
	public void disabled(){
		endGame();
	}
	
	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}

	public void TankDrive() {
		mDriveTrain.driveTank();
	}

	public void CrabDrive() {
		mDriveTrain.driveCrab();
	}

	public void SwerveDrive() {
		mDriveTrain.driveSwerve();
	}

	public void PneumaticsTest() {
		if (mController.getAButtonReleased()) {
			mLeft.set(IntakeConstants.CLOSED);
			mRight.set(IntakeConstants.CLOSED);
		} else if (mController.getYButtonReleased()) {
			mLeft.set(IntakeConstants.OPEN);
			mRight.set(IntakeConstants.OPEN);
		} else if (mController.getXButtonReleased()) {
			mLeft.set(mLeft.get().equals(IntakeConstants.CLOSED) ? IntakeConstants.OPEN
					: IntakeConstants.CLOSED);
		} else if (mController.getBButtonReleased()) {
			mRight.set(mRight.get().equals(IntakeConstants.CLOSED) ? IntakeConstants.OPEN
					: IntakeConstants.CLOSED);
		}
		SmartDashboard.putString("Right", mRight.get().toString());
		SmartDashboard.putString("Left", mLeft.get().toString());
		SmartDashboard.putBoolean("Limit Switch", mLimitSwitch.get());
		SmartDashboard.putBoolean("Break Beam", mBreakbeam.get());
	}

	public void DriveInit(){
		for (int i = 0; i < 4; i++) {
			mTurn[i] = new WPI_TalonSRX(Ports.TURN[i]);
			mTurn[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
			mTurn[i].setNeutralMode(NeutralMode.Brake);
			
			mTurn[i].setSensorPhase(DriveConstants.Modules.ENCODER_REVERSED[i]);
			mTurn[i].setInverted(DriveConstants.Modules.TURN_INVERTED[i]);
			
			mTurn[i].config_kP(0, DriveConstants.PID_Constants.ROTATION_P[i], 10);
			mTurn[i].config_kI(0, DriveConstants.PID_Constants.ROTATION_I[i], 10);
			mTurn[i].config_kD(0, DriveConstants.PID_Constants.ROTATION_D[i], 10);
			mTurn[i].config_IntegralZone(0, DriveConstants.PID_Constants.ROTATION_IZONE[i], 10);
			mTurn[i].configAllowableClosedloopError(0, DriveConstants.PID_Constants.ROTATION_TOLERANCE[i], 10);
			
			mDrive[i] = new WPI_TalonSRX(Ports.DRIVE[i]);
			mDrive[i].setInverted(DriveConstants.Modules.INVERTED[i]);
			mDrive[i].setNeutralMode(NeutralMode.Brake);
	
			mEncoder[i] = new TalonAbsoluteEncoder(mTurn[i], DriveConstants.Modules.ENCODER_REVERSED[i],
					ResourceFunctions.tickToAngle(DriveConstants.Modules.OFFSETS[i]));
			// Offset needs to be in degrees
			mWheel[i] = new Wheel(mTurn[i], mDrive[i], mEncoder[i], DriveConstants.Modules.TURN_INVERTED[i]);
		}
		mRobotAngle = new RobotAngle(mNavX, false, 0);
		mDriveTrain = new DriveTrain(mWheel, mController, mRobotAngle);
	}

	public void IntakeInit(){
		mLimitSwitch = new DigitalInput(Ports.LIMITSWITCH);
		mBreakbeam = new DigitalInput(Ports.BREAKBEAM);
		mLeft = new DoubleSolenoidReal(Ports.LEFT_INTAKE_IN, Ports.LEFT_INTAKE_OUT);
		mRight = new DoubleSolenoidReal(Ports.RIGHT_INTAKE_IN, Ports.RIGHT_INTAKE_OUT);
		mIntakeTalon = new WPI_TalonSRX(Ports.INTAKE);
		mIntake = new Intake(mIntakeTalon, mLeft, mRight, mLimitSwitch, mBreakbeam, mJoystick);
	}

}