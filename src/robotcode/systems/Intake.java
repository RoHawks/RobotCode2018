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
	private DigitalInput mLimitSwitch, mBreakbeam;
	private IntakeState mIntakeState;
	private IntakeState mPrevIntakeState; // if we want to only go into flip
											// from open/closed from flip
	private Joystick mJoystick;

	private double mWheelSpeed;

	public Intake(WPI_TalonSRX pWheel, SolenoidInterface pLeft, SolenoidInterface pRight, DigitalInput pLimitSwitch,
			DigitalInput pBreakbeam, Joystick pJoystick) {
		mSquishyWheels = pWheel;
		mLeftPiston = pLeft;
		mRightPiston = pRight;
		mLimitSwitch = pLimitSwitch;
		mBreakbeam = pBreakbeam;
		mJoystick = pJoystick;
		mIntakeState = IntakeState.CLOSED;
		mPrevIntakeState = IntakeState.CLOSED;
	}

	public enum IntakeState {
		CLOSED, FLIP_ALT, FLIP_ALT_SLOW, FLIP_SYNCH, OPEN 
	}

	public void enactMovement() {
		getState();

		SmartDashboard.putBoolean("Limit Switch", mLimitSwitch.get());
		SmartDashboard.putBoolean("Breakbeam", mBreakbeam.get());
		SmartDashboard.putNumber("Joystick Y", mJoystick.getY());
		SmartDashboard.putString("Intake State", mIntakeState.toString());

		mWheelSpeed = (Math.abs(mJoystick.getY()) > 0.25)
				? -Math.signum(mJoystick.getY()) * IntakeConstants.INTAKE_SPEED : 0;

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
			case FLIP_ALT_SLOW:
				altMovePistonsSlow();
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
		setOpposite(mLeftPiston);
		setOpposite(mRightPiston);
		Timer.delay(0.05);
	}

	public void altMovePistons() {
		if (!(mLeftPiston.get().equals(mRightPiston.get()))) {
			synchMovePistons();
		} else {
			setOpposite(mRightPiston);
		}
	}

	public void closeMovePistons() {
		mLeftPiston.set(IntakeConstants.CLOSED);
		mRightPiston.set(IntakeConstants.CLOSED);
	}

	public void altMovePistonsSlow() {
		if (!(mLeftPiston.get().equals(mRightPiston.get()))) {
			synchMovePistons();
		} else {
			setOpposite(mRightPiston);
			Timer.delay(0.1); //unclear how long
		}
	}

	public void getState() {
		IntakeState state;
		if (mJoystick.getRawButton(2)) {
			state = IntakeState.OPEN;
		} else if (mLimitSwitch.get()/* || !mBreakbeam.get()*/) {
			state = IntakeState.CLOSED;
		} else if (mJoystick.getRawButton(3)) {
			state = IntakeState.FLIP_ALT;
		} else if (mJoystick.getRawButton(1)) {
			state = IntakeState.CLOSED;
		} else if (mJoystick.getRawButton(4)) {
			state = IntakeState.FLIP_ALT_SLOW;
		} /*else if (!mBreakbeam.get()) {
			state = IntakeState.FLIP_ALT_SLOW;
		}*/
		else {
			state = mIntakeState.OPEN;
		}

		if (mIntakeState != state) {
			mPrevIntakeState = mIntakeState;
		}
		mIntakeState = state;
	}

	private void setOpposite(SolenoidInterface pPiston) {
		pPiston.set(pPiston.get().equals(IntakeConstants.OPEN) ? IntakeConstants.CLOSED : IntakeConstants.OPEN);
	}

}