package robotcode.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Runnable{

	private WPI_TalonSRX mSquishyWheels;
	private SolenoidInterface mLeftPiston, mRightPiston;
	private DigitalInput mLimitSwitch, mBreakbeam;
	private IntakeState mIntakeState;
	private IntakeState mPrevIntakeState; // if we want to only go into flip
											// from open/closed from flip
	private Joystick mJoystick;

	private double mWheelSpeed;
	private boolean mEnabled = true;

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
		CLOSED, DISABLE, FLIP_ALT, FLIP_SYNCH, OPEN 
	}

	public void enactMovement() {
		getState();

		SmartDashboard.putBoolean("Limit Switch", mLimitSwitch.get());
		SmartDashboard.putBoolean("Breakbeam", mBreakbeam.get());
		SmartDashboard.putNumber("Joystick Y", mJoystick.getY());
		SmartDashboard.putString("Intake State", mIntakeState.toString());

		switch (mIntakeState) {
			case DISABLE:
				mWheelSpeed = 0;
				break;
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

	}

	public void openMovePistons() {
		mLeftPiston.set(IntakeConstants.OPEN);
		mRightPiston.set(IntakeConstants.OPEN);
		Timer.delay(0.15);
	}

	public void synchMovePistons() {
		setOpposite(mLeftPiston);
		setOpposite(mRightPiston);
		Timer.delay(0.15);
	}

	public void altMovePistons() {
		if (!(mLeftPiston.get().equals(mRightPiston.get()))) {
			synchMovePistons();
		} else {
			setOpposite(mRightPiston);
			Timer.delay(0.15);
		}
	}

	public void closeMovePistons() {
		mLeftPiston.set(IntakeConstants.CLOSED);
		mRightPiston.set(IntakeConstants.CLOSED);
		Timer.delay(0.15);
	}
	
	public void setWheelSpeed() {
		getSpeed();
		SmartDashboard.putNumber("Intake Talon Temp", mSquishyWheels.getTemperature());
		/*mWheelSpeed = (Math.abs(mJoystick.getY()) > 0.25)
				? Math.signum(mJoystick.getY()) * IntakeConstants.INTAKE_SPEED : 0;*/
		mSquishyWheels.set(mWheelSpeed);
	}

	public void getState() {
		IntakeState state;
		if (!mEnabled) {
			state = IntakeState.DISABLE;
		} else if (mJoystick.getRawButton(2)) {
			state = IntakeState.OPEN;
		} else if (mJoystick.getRawButton(3)) {
			state = IntakeState.FLIP_ALT;
		} else if (mJoystick.getRawButton(1)) {
			state = IntakeState.CLOSED;
		} else if (!mBreakbeam.get()) {
			state = IntakeState.CLOSED;
		} else {
			state = IntakeState.OPEN;
		}

		if (mIntakeState != state) {
			mPrevIntakeState = mIntakeState;
		}
		mIntakeState = state;
	}
	
	public void enable() {
		mEnabled = true;
	}
	
	public void disable() {
		mEnabled = false;
	}
	
	private void setOpposite(SolenoidInterface pPiston) {
		pPiston.set(pPiston.get().equals(IntakeConstants.OPEN) ? IntakeConstants.CLOSED : IntakeConstants.OPEN);
	}
	
	private void getSpeed() {
		double yPos = mJoystick.getY();
		mWheelSpeed = (Math.abs(yPos) > 0.25) ? (-1 * Math.signum(yPos) * yPos * yPos) : 0;
	}

	@Override
	public void run() {
		while(mEnabled && !Thread.interrupted()){
			enactMovement();
		}
		mLeftPiston.set(IntakeConstants.OPEN);
		mRightPiston.set(IntakeConstants.OPEN);
		mSquishyWheels.set(0);
	}

}