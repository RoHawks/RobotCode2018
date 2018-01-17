package robotcode.driving;

import com.kauailabs.navx.frc.AHRS;

import constants.DriveConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import resource.ResourceFunctions;
import resource.Vector;

public class DriveTrain {
	private SwerveDrive mSwerveDrive;
	private Wheel[] mWheels;

	private XboxController mController;

	private Vector mDesiredRobotVel;
	private double mDesiredAngularVel;

	private double mJoystickAngle;
	private boolean mIsFieldRelative;

	private AHRS mNavX;

	public DriveTrain(Wheel[] pWheels, XboxController pController, AHRS pNavX) {
		mWheels = pWheels;
		mController = pController;
		mSwerveDrive = new SwerveDrive(mWheels);
		mNavX = pNavX;

		mDesiredRobotVel = new Vector();
		mDesiredAngularVel = 0;

		mJoystickAngle = 0;
		mIsFieldRelative = false;
	}

	/**
	 * are all wheels in tolerance range
	 * 
	 * @return
	 */
	// public boolean getAllWheelsInRange() {
	// boolean allInRange = true;
	// for (int i = 0; i < 4; i++) {
	// if (!mWheels[i].isInRangeNudge())
	// {
	// allInRange = false;
	// }
	// }
	//
	// return allInRange;
	// }

	public void enactMovement() {
		double joystickAngle = getStickAngle(Hand.kLeft);
		double robotDirectionAngle = joystickAngle;

		if (mController.getStartButtonReleased()) {
			mIsFieldRelative = !mIsFieldRelative;
		}

		if (mIsFieldRelative) {
			robotDirectionAngle = ResourceFunctions.putAngleInRange(joystickAngle - (mNavX.getAngle() % 360));
		}
		
		SmartDashboard.putNumber("NavX Angle", mNavX.getAngle() % 360);
		SmartDashboard.putNumber("Robot Angle", robotDirectionAngle);
		SmartDashboard.putBoolean("Field Relative", mIsFieldRelative);

		double linearSpeed = getStickLinearVel();
		Vector linearVel = Vector.createPolar(robotDirectionAngle, linearSpeed);
		mDesiredRobotVel = new Vector(linearVel);

		mDesiredAngularVel = angularVelStick();
		SmartDashboard.putNumber("Angular Velocity", mDesiredAngularVel);
		mSwerveDrive.calculate(getDesiredAngularVel(), getDesiredRobotVel());

		for (int i = 0; i < 4; i++) {
			if (Math.abs(mController.getX(Hand.kRight)) < DriveConstants.MIN_DIRECTION_MAG) {
				if (linearSpeed < DriveConstants.MIN_LINEAR_VEL) {
					mWheels[i].set(robotDirectionAngle, 0);
				} 
				else {
					mWheels[i].set(mDesiredRobotVel);
				}
			}
			else {
				mWheels[i].set(mSwerveDrive.getOutput(i));
			}
		}

	}

	public void driveTank() {
		double forwardVel = mController.getTriggerAxis(Hand.kRight) - mController.getTriggerAxis(Hand.kLeft);
		forwardVel *= 0.4;
		double angularVel = mController.getX(Hand.kRight);
		angularVel *= 0.1;

		double leftSpeed = forwardVel - angularVel;
		double rightSpeed = forwardVel + angularVel;

		mWheels[0].set(0, rightSpeed);
		mWheels[1].set(0, leftSpeed);
		mWheels[2].set(0, leftSpeed);
		mWheels[3].set(0, rightSpeed);
	}
	
	public void driveCrab(){
		double linearVelocity = mDriveTrain.getStickLinearVel();
		double joystickAngle = mDriveTrain.getStickAngle(Hand.kLeft);

		for (Wheel wheel : mWheel) {
			wheel.set(joystickAngle, linearVelocity); // Max speed is set in wheel class
		}
	}
	
	/**
	 * Gets the angle of Xbox controller joystick
	 * 
	 * @param h
	 *            Joystick to get the angle for
	 * @return Angle of the stick in degrees, with 0 degrees pointing directly
	 *         up on the controller
	 */
	public double getStickAngle(Hand h) { //add if statements to figure out which hand for logging
		double x = mController.getX(h);
		double y = -mController.getY(h);
		SmartDashboard.putNumber("x-value", x);
		SmartDashboard.putNumber("y-value", y);
		if (Math.abs(y) >= DriveConstants.MIN_DIRECTION_MAG || Math.abs(x) >= DriveConstants.MIN_DIRECTION_MAG) {
			mJoystickAngle = -Math.toDegrees(Math.atan2(y, x)) + 90;
			mJoystickAngle = ResourceFunctions.putAngleInRange(mJoystickAngle);
			//puts angle between zero and 360
		}
		SmartDashboard.putNumber("Joystick Angle", mJoystickAngle);
		return mJoystickAngle;
	}

	/**
	 * Gets magnitude of Xbox controller joystick
	 * 
	 * @param h
	 *            Joystick to get the magnitude for
	 * @return Magnitude of the depression; 0 is not used, 1 is fully depressed
	 */
	public double getStickMag(Hand h) {
		Vector v = new Vector(mController.getX(h), mController.getY(h));
		return v.getMagnitude();
	}

	// /**
	// * Get the direction vector for nudge driving using the letter buttons
	// * @return correct direction vector
	// */
	// private Vector nudgeMove()
	// {
	// Vector driveVec = new Vector();
	// if(mController.getYButton())
	// {
	// driveVec = Vector.createPolar(0,
	// DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
	// }
	// else if(mController.getXButton())
	// {
	// driveVec = Vector.createPolar(90,
	// DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
	// }
	// else if(mController.getAButton())
	// {
	// driveVec = Vector.createPolar(180,
	// DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
	// }
	// else if(mController.getBButton())
	// {
	// driveVec = Vector.createPolar(270,
	// DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
	// }
	//
	//
	// return driveVec;
	// }
	// /**
	// * Angular velocity using nudge bumpers
	// * @return correct angular velocity
	// */
	// private double nudgeTurn()
	// {
	// double speed = 0;
	// if(mController.getBumper(Hand.kLeft))
	// {
	// speed = DriveConstants.SwerveSpeeds.NUDGE_TURN_SPEED;
	// }
	// else if(mController.getBumper(Hand.kRight))
	// {
	// speed = -DriveConstants.SwerveSpeeds.NUDGE_TURN_SPEED;
	// }
	//
	// return speed;
	// }
	//
	public double getStickLinearVel() {
		double speed = mController.getTriggerAxis(Hand.kRight);
		SmartDashboard.putNumber("TriggerAxis", speed);
		speed = Math.pow(speed, 2) * DriveConstants.SwerveSpeeds.SPEED_MULT;
		//quadratic control, finer control of lower speeds
		return speed;
	}

	/**
	 * Angular velocity calculated with the right joystick
	 * 
	 * @return angular velocity for swerve drive
	 */
	private double angularVelStick() {
		double angularVel = mController.getX(Hand.kRight) * Math.abs(mController.getX(Hand.kRight));
		SmartDashboard.putNumber("Right Joystick X", mController.getX(Hand.kRight));
		angularVel *= DriveConstants.SwerveSpeeds.ANGULAR_SPEED_MULT;
		// angularVel = -angularVel; //correct the sign for clockwise/counter-clockwise
		return angularVel; // quadratic control for finer movements
	}

	public Vector getDesiredRobotVel() {
		return mDesiredRobotVel;
	}

	public double getDesiredAngularVel() {
		return mDesiredAngularVel;
	}
}
