package robotcode.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

	private WPI_TalonSRX mSquishyWheels;
	private SolenoidInterface mLeftPiston, mRightPiston;
	private DigitalInput mBreakbeam, mLimitSwitch;
	private IntakeState mIntakeState;
	private IntakeState mPrevIntakeState; // if we want to only go into flip
											// from open/closed from flip
	private Joystick mJoystick;

	private double mWheelSpeed;

	public Intake(WPI_TalonSRX pWheel, SolenoidInterface pLeft, SolenoidInterface pRight, DigitalInput pBreakbeam,
			DigitalInput pLimitSwitch, Joystick pJoystick) {
		mSquishyWheels = pWheel;
		mLeftPiston = pLeft;
		mRightPiston = pRight;
		mBreakbeam = pBreakbeam;
		mLimitSwitch = pLimitSwitch;
		mJoystick = pJoystick;
		mIntakeState = IntakeState.CLOSED;
		mPrevIntakeState = IntakeState.CLOSED;
	}

	public enum IntakeState {
		OPEN, FLIP_SYNCH, FLIP_ALT, CLOSED
	}

	public void enactMovement() {
		getState();
		
		SmartDashboard.putBoolean("Limit Switch", mLimitSwitch.get());
		SmartDashboard.putNumber("Joystick Y", mJoystick.getY());
		SmartDashboard.putString("Intake State", mIntakeState.toString());
		
		if (Math.abs(mJoystick.getY()) > 0.25) {
			mWheelSpeed = (mJoystick.getY() > 0) ? -1 * IntakeConstants.INTAKE_SPEED : IntakeConstants.INTAKE_SPEED;
		} else {
			mWheelSpeed = 0;
		}
		
		switch (mIntakeState) {
		case FLIP_SYNCH:
			synchMovePistons();
			break;
		case FLIP_ALT:
			altMovePistons();
			break;
		case CLOSED:
			closeMovePistons();
			break;
		case OPEN:
			openMovePistons();
			break;
		default:
			openMovePistons();
		}
		
		mSquishyWheels.set(mWheelSpeed);
	}

	public void openMovePistons() {
		mLeftPiston.set(IntakeConstants.OPEN);
		mRightPiston.set(IntakeConstants.OPEN);
	}

	public void synchMovePistons() {
		setLeftOpposite();
		setRightOpposite();
		Timer.delay(0.05);
	}

	public void altMovePistons() {
		if (!(mLeftPiston.get().equals(mRightPiston.get()))) {
			synchMovePistons();
		} else {
			setRightOpposite();
			//Timer.delay(0.1);
		}
	}

	public void closeMovePistons() {
		mLeftPiston.set(IntakeConstants.CLOSED);
		mRightPiston.set(IntakeConstants.CLOSED);
	}

	public void getState() {
		IntakeState state;
		if (mLimitSwitch.get()) {
			state = IntakeState.CLOSED;
		} else if (mJoystick.getRawButton(3)) {
			state = IntakeState.FLIP_ALT;
		} else if (mJoystick.getRawButton(1)) {
			state = IntakeState.CLOSED;
		} else if (mJoystick.getRawButton(2)) {
			state = IntakeState.OPEN;
		} else {
			state = mIntakeState.OPEN;
		}
		
		if (mIntakeState != state) {
			mPrevIntakeState = mIntakeState;
		}
		mIntakeState = state;
	}

	public void setState(IntakeState pState) {
		if (mIntakeState != pState) {
			mPrevIntakeState = mIntakeState;
		}
		mIntakeState = pState;
	}

	private void setLeftOpposite() {
		mLeftPiston.set(mLeftPiston.get().equals(IntakeConstants.OPEN) ? IntakeConstants.CLOSED : IntakeConstants.OPEN);
	}

	private void setRightOpposite() {
		mRightPiston
				.set(mRightPiston.get().equals(IntakeConstants.OPEN) ? IntakeConstants.CLOSED : IntakeConstants.OPEN);
	}
}