package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Intake {

	private WPI_TalonSRX mSquishyWheels;
	private SolenoidInterface mLeftPiston, mRightPiston;
	private DigitalInput mBreakbeam, mLimitSwitch;
	private IntakeState mIntakeState;
	private IntakeState mPrevIntakeState; //if we want to only go into flip from open/closed from flip
	private Joystick mJoystick;
	
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
		OPEN, FLIP_SYNCH, FLIP_ALT, CLOSED, RELEASE, IN, OUT
	}

	public void enactMovement() {
		double speed = 0;
		if (mJoystick.getRawButton(1)) {
			mLeftPiston.set(IntakeConstants.LEFT_CLOSED);
			mRightPiston.set(IntakeConstants.RIGHT_CLOSED);
			// Timer.delay(0.3);
		} else if (mJoystick.getRawButton(6)){
			mLeftPiston.set(IntakeConstants.LEFT_CLOSED);
		} else if(mJoystick.getRawButton(11)){
			mRightPiston.set(IntakeConstants.RIGHT_CLOSED);
		}
		else {
			mLeftPiston.set(IntakeConstants.LEFT_OPEN);
			mRightPiston.set(IntakeConstants.RIGHT_OPEN);
			// Timer.delay(0.3);
		}
		if (mLimitSwitch.get()) {
			speed = 0;
		} else {
			speed = IntakeConstants.INTAKE_SPEED;
		}
//		if(mLeftPiston.get().equals(IntakeConstants.LEFT_CLOSED) || mRightPiston.get().equals(IntakeConstants.RIGHT_CLOSED)){
//			speed = 0;
//		}
		mSquishyWheels.set(speed);
		//		getState();
//		switch (mIntakeState) {
//		case FLIP_SYNCH:
//			synchMove();
//			break;
//		case FLIP_ALT:
//			mLeftPiston.set(IntakeConstants.LEFT_CLOSED);
//			Timer.delay(0.3);
//			synchMove();
//			break;
//		case CLOSED:
//			closeMove();
//			break;
//		case RELEASE:
//			releaseMove();
//			break;
//		case OPEN:
//		default:
//			openMove();
//		}
	}

	public void openMove() {
		mSquishyWheels.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
		mLeftPiston.set(IntakeConstants.LEFT_OPEN);
		mRightPiston.set(IntakeConstants.RIGHT_OPEN);
	}

	public void synchMove() {
		mSquishyWheels.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
		if (mLeftPiston.get().equals(IntakeConstants.LEFT_OPEN)) {
			mLeftPiston.set(IntakeConstants.LEFT_CLOSED);
		} else {
			mLeftPiston.set(IntakeConstants.LEFT_OPEN);
		}
		if (mRightPiston.get().equals(IntakeConstants.RIGHT_OPEN)) {
			mRightPiston.set(IntakeConstants.RIGHT_CLOSED);
		} else {
			mRightPiston.set(IntakeConstants.RIGHT_OPEN);
		}
		Timer.delay(0.3);
		// mLeftPiston.set(IntakeConstants.LEFT_CLOSED);
		// mRightPiston.set(IntakeConstants.RIGHT_CLOSED);
		// Timer.delay(0.3);
		// mLeftPiston.set(IntakeConstants.LEFT_OPEN);
		// mRightPiston.set(IntakeConstants.RIGHT_OPEN);
		// Timer.delay(0.5);
	}

	public void altMove() {
		mSquishyWheels.set(IntakeConstants.INTAKE_SPEED);
		mLeftPiston.set(mRightPiston.get().equals(IntakeConstants.RIGHT_CLOSED) ? IntakeConstants.LEFT_OPEN
				: IntakeConstants.LEFT_CLOSED);
		mRightPiston.set(mLeftPiston.get().equals(IntakeConstants.LEFT_CLOSED) ? IntakeConstants.RIGHT_OPEN
				: IntakeConstants.RIGHT_CLOSED);
		Timer.delay(0.3); // ? ? ?
	}

	public void closeMove() {
		mSquishyWheels.set(ControlMode.PercentOutput, 0);
		mLeftPiston.set(IntakeConstants.LEFT_CLOSED);
		mRightPiston.set(IntakeConstants.RIGHT_CLOSED);
	}

	public void releaseMove() {
		mSquishyWheels.set(ControlMode.PercentOutput, -1 * IntakeConstants.INTAKE_SPEED);
		mLeftPiston.set(IntakeConstants.LEFT_OPEN);
		mRightPiston.set(IntakeConstants.RIGHT_OPEN);
		Timer.delay(0.3); // ?
	}

	public void getState() {
		IntakeState state;
		if (mLimitSwitch.get()) {
			state = IntakeState.CLOSED;
		} else if (mJoystick.getRawButton(1) && !mLimitSwitch.get()) {
			state = IntakeState.FLIP_SYNCH;
		} else {
			state = IntakeState.OPEN;
		}
		if (mIntakeState != state) {
			mPrevIntakeState = mIntakeState;
		}
		mIntakeState = state;
	}
	
	public void setState(IntakeState pState){
		if (mIntakeState != pState) {
			mPrevIntakeState = mIntakeState;
		}
		mIntakeState = pState;
	}
}