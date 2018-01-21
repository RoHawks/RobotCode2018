package robotcode.driving;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

	private LinearVelocity mLinearVel;
	private LinearVelocity mPrevLinearVel;
	private RotationalVelocity mRotationalVel;

	public DriveTrain(Wheel[] pWheels, XboxController pController, AHRS pNavX) {
		mWheels = pWheels;
		mController = pController;
		mSwerveDrive = new SwerveDrive(mWheels);
		mNavX = pNavX;

		mDesiredRobotVel = new Vector();
		mDesiredAngularVel = 0;

		mJoystickAngle = 0;
		mIsFieldRelative = false;

		mLinearVel = LinearVelocity.NONE;
		mPrevLinearVel = LinearVelocity.NONE;
		mRotationalVel = RotationalVelocity.NONE;
	}

	public enum LinearVelocity {
		NORMAL, NUDGE, ANGLE_ONLY, NONE
	}

	public enum RotationalVelocity {
		NORMAL, NUDGE, NONE /* JOYSTICK, */
	}

	public void move() {

	}

	public void enactMovement() {
		double joystickAngle = getStickAngle(Hand.kLeft);
		double robotDirectionAngle = joystickAngle;
		
		mLinearVel = getLinearVelocity();
		mRotationalVel = getRotationalVelocity();

		if (mController.getStartButtonReleased()) {
			mIsFieldRelative = !mIsFieldRelative;
		}

		if (mIsFieldRelative) {
			robotDirectionAngle = ResourceFunctions
					.putAngleInRange(joystickAngle - ResourceFunctions.putAngleInRange(mNavX.getAngle()));
		}

		SmartDashboard.putBoolean("Field Relative", mIsFieldRelative);

		Vector linearVel = new Vector();
		switch (mLinearVel) {
		case NORMAL:
			SmartDashboard.putString("Linear Vel State", "NORMAL");
			linearVel = Vector.createPolar(robotDirectionAngle, getStickLinearVel());
			break;
		case NUDGE:
			SmartDashboard.putString("Linear Vel State", "NUDGE");
			linearVel = nudgeMove();
			break;
		case ANGLE_ONLY:
			SmartDashboard.putString("Linear Vel State", "ANGLE ONLY");
			// linearVel = Vector.createPolar(robotDirectionAngle, 0);
			break;
		case NONE:
			SmartDashboard.putString("Linear Vel State", "NONE");
			break;
		}
		SmartDashboard.putString("Previous Linear Velocity", linearVelocityToString(mPrevLinearVel));
		mDesiredRobotVel = new Vector(linearVel);

		switch (mRotationalVel) {
		case NORMAL:
			SmartDashboard.putString("Rotational Vel State", "NORMAL");
			mDesiredAngularVel = angularVelStick();
			break;
		case NUDGE:
			SmartDashboard.putString("Rotational Vel State", "NUDGE");
			mDesiredAngularVel = nudgeTurn();
			break;
		case NONE:
			SmartDashboard.putString("Rotational Vel State", "NONE");
			mDesiredAngularVel = 0;
			break;
		}
		
		for (int i = 0; i < 4; i++) {
			if (mLinearVel == LinearVelocity.ANGLE_ONLY && mRotationalVel == RotationalVelocity.NONE) {
				mWheels[i].set(robotDirectionAngle, 0);
			} else if (mLinearVel == LinearVelocity.NONE && mPrevLinearVel == LinearVelocity.NUDGE
					&& mRotationalVel == RotationalVelocity.NONE) {
				mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), 0);
			}
			else if (mRotationalVel == RotationalVelocity.NONE && mLinearVel == LinearVelocity.NONE) {
				mWheels[i].setDriveSpeed(0);
				mWheels[i].setTurnSpeed(0);
			} else {
				mSwerveDrive.calculate(getDesiredAngularVel(), getDesiredRobotVel());
				mWheels[i].set(mSwerveDrive.getOutput(i));
			}
		}
	}

	/**
	 * tank drive : right trigger = forward ; left trigger = turn ; right
	 * joystick x = turn
	 */
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

	/**
	 * i like crabs
	 */
	public void driveCrab() {
		double linearVelocity = getStickLinearVel();
		double joystickAngle = getStickAngle(Hand.kLeft);
		for (Wheel wheel : mWheels) {
			wheel.set(joystickAngle, linearVelocity);
			// Max speed is set in wheel class
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
	public double getStickAngle(Hand h) {
		// add if statements to figure out which hand for logging
		double x = mController.getX(h);
		double y = -mController.getY(h);
		SmartDashboard.putNumber("x-value", x);
		SmartDashboard.putNumber("y-value", y);
		if (Math.abs(y) >= DriveConstants.MIN_DIRECTION_MAG || Math.abs(x) >= DriveConstants.MIN_DIRECTION_MAG) {
			mJoystickAngle = -Math.toDegrees(Math.atan2(y, x)) + 90;
			mJoystickAngle = ResourceFunctions.putAngleInRange(mJoystickAngle);
			// puts angle between zero and 360
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

	/**
	 * Angular velocity using nudge bumpers
	 * 
	 * @return correct angular velocity
	 */
	private double nudgeTurn() {
		if (mController.getBumper(Hand.kLeft)) {
			return DriveConstants.SwerveSpeeds.NUDGE_TURN_SPEED;
		} else if (mController.getBumper(Hand.kRight)) {
			return -DriveConstants.SwerveSpeeds.NUDGE_TURN_SPEED;
		}

		return 0;
	}

	/**
	 * Get the direction vector for nudge driving using the letter buttons
	 * 
	 * @return correct direction vector
	 */
	private Vector nudgeMove() {
		Vector driveVec = new Vector();
		if (mController.getYButton()) {
			driveVec = Vector.createPolar(0, DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
		} else if (mController.getXButton()) {
			driveVec = Vector.createPolar(270, DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
		} else if (mController.getAButton()) {
			driveVec = Vector.createPolar(180, DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
		} else if (mController.getBButton()) {
			driveVec = Vector.createPolar(90, DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
		}

		return driveVec;
	}

	/**
	 * quadratic control of right trigger axis
	 * 
	 * @return trigger axis squared
	 */
	public double getStickLinearVel() {
		double speed = mController.getTriggerAxis(Hand.kRight);
		SmartDashboard.putNumber("TriggerAxis", speed);
		speed = Math.pow(speed, 2) * DriveConstants.SwerveSpeeds.SPEED_MULT;
		// quadratic control, finer control of lower speeds
		return speed;
	}

	/**
	 * Angular velocity calculated with the right joystick
	 * 
	 * @return angular velocity for swerve drive
	 */
	private double angularVelStick() {
		double joystickValue = mController.getX(Hand.kRight);
		SmartDashboard.putNumber("Right Joystick X", joystickValue);

		if (Math.abs(joystickValue) < DriveConstants.MIN_DIRECTION_MAG) {
			return 0;
		}
		double angularVel = joystickValue * Math.abs(joystickValue);
		angularVel *= DriveConstants.SwerveSpeeds.ANGULAR_SPEED_MULT;
		// angularVel = -angularVel; //correct the sign for
		// clockwise/counter-clockwise
		return angularVel; // quadratic control for finer movements
	}

	/**
	 * gets linear velocity state
	 * 
	 * @return bumper --> nudge; no move --> angle only; else --> normal
	 */
	private LinearVelocity getLinearVelocity() {
		LinearVelocity retLin = LinearVelocity.NONE;
		if (getLetterPressed()) {
			retLin = LinearVelocity.NUDGE;
		} else if (getStickLinearVel() < DriveConstants.MIN_LINEAR_VEL
				&& getStickMag(Hand.kLeft) > DriveConstants.MIN_DIRECTION_MAG) {
			retLin = LinearVelocity.ANGLE_ONLY;
		} else if (getStickLinearVel() > DriveConstants.MIN_LINEAR_VEL) {
			retLin = LinearVelocity.NORMAL;
		}
		if (retLin != mLinearVel) {
			mPrevLinearVel = mLinearVel;
		}
		return retLin;
	}

	/**
	 * gets rotational velocity state
	 * 
	 * @return bumper --> nudge; no move --> none; else --> normal
	 */
	// add joystick
	private RotationalVelocity getRotationalVelocity() {
		if (mController.getBumper(Hand.kRight) || mController.getBumper(Hand.kLeft)) {
			return RotationalVelocity.NUDGE;
		} else if (angularVelStick() != 0) {
			return RotationalVelocity.NORMAL;
		}
		return RotationalVelocity.NONE;
	}

	public boolean getLetterPressed() {
		return (mController.getAButton() || mController.getBButton() || mController.getXButton()
				|| mController.getYButton());
	}

	public Vector getDesiredRobotVel() {
		return mDesiredRobotVel;
	}

	public double getDesiredAngularVel() {
		return mDesiredAngularVel;
	}

	public String linearVelocityToString(LinearVelocity pLin) {
		switch (pLin) {
		case NORMAL:
			return "NORMAL";
		case NUDGE:
			return "NUDGE";
		case ANGLE_ONLY:
			return "ANGLE ONLY";
		case NONE:
			return "NONE";
		}
		return "";
	}

	public String rotationalVelocityToString(RotationalVelocity pRot) {
		switch (pRot) {
		case NORMAL:
			return "NORMAL";
		case NUDGE:
			return "NUDGE";
		case NONE:
			return "NONE";
		}
		return "";
	}
}

/// **
// * are all wheels in tolerance range
// *
// * @return
// */
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