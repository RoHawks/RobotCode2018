package robotcode.driving;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import constants.DriveConstants;
import edu.wpi.first.wpilibj.LocalPositionPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import sensors.TalonAbsoluteEncoder;

public class Wheel {
	
	private WPI_TalonSRX mTurn;
	private WPI_TalonSRX mDrive;
	private TalonAbsoluteEncoder mEncoder;
	private boolean mTurnInverted;
	private int counter = 0;
	
	private static final double NONTALON_P = 0.1;
	
	
	//public LocalPositionPIDController mPID;
	
	public Wheel(WPI_TalonSRX pTurn, WPI_TalonSRX pDrive, TalonAbsoluteEncoder pEncoder, boolean pTurnInverted){
		mTurn = pTurn;
		mDrive = pDrive;
		mEncoder = pEncoder;
		mTurnInverted = pTurnInverted;
		
//		mPID = new LocalPositionPIDController(PCONSTANT, ICONSTANT, 0.0, mTurn);
//		mPID.setOutputRange(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
//		mPID.setInputRange(0, 360);
//		mPID.setSetpoint(0.0);
//		mPID.setContinuous(true);
//		mPID.enable();
	}
	
	/**
	 * Set wheel angle & speed
	 * 
	 * @param pWheelVelocity
	 *            Vector of wheel velocity
	 */
	public void set(Vector pWheelVelocity) {
		set(pWheelVelocity.getAngle(), pWheelVelocity.getMagnitude());
	}

	/**
	 * Set wheel angle & speed
	 * 
	 * @param angle
	 *            direction to point the wheel
	 * @param speed
	 *            magnitude to drive the wheel
	 */
	public void set(double angle, double speed) {
		setAngle(angle);
		setLinearVelocity(speed);
	}
	
	public void setLinearVelocity(double pSpeed) {
		pSpeed = Math.signum(pSpeed) * Math.min(Math.abs(pSpeed), DriveConstants.MAX_LINEAR_VEL);
		mDrive.set(ControlMode.PercentOutput, pSpeed);
		SmartDashboard.putNumber("Linear Speed", pSpeed);
	}
	
	public void setAngle(double pAngle) {
		double speed = proportional(pAngle);
		mTurn.set(ControlMode.PercentOutput, speed);
	}
	
	public void setAngleTalon(double pAngle){
		TalonPID(pAngle);
	}
	
	private double proportional(double pTarget) {
		double current = mEncoder.getAngleDegrees();
		double error = ResourceFunctions.continuousAngleDif(current, 360 - pTarget);//360 - pTarget was a quick fix, will think more later
		if (Math.abs(error) > 90) {
			mEncoder.setAdd180(!mEncoder.getAdd180());
			setInverted(!mDrive.getInverted());
			error = ResourceFunctions.continuousAngleDif(pTarget, mEncoder.getAngleDegrees());
		}
		if (Math.abs(error) < 5) {
			return 0;
		}
		SmartDashboard.putNumber("error", error); //Instance variable?
		double speed = error * NONTALON_P;//Temporary for onboard PID
		speed = Math.signum(speed) * Math.min(Math.abs(speed), DriveConstants.MAX_TURN_VEL) * (mTurnInverted ? -1 : 1);
		return speed;
	}
	
	
	private void TalonPID(double pTarget){
		/*mTurn.setSelectedSensorPosition(mTurn.getSelectedSensorPosition(0) % 4096, 0, 10);
		double current = mEncoder.getAngleDegrees(); //takes offset into account
		double error = ResourceFunctions.continuousAngleDif(pTarget, current);
		SmartDashboard.putNumber("error" + counter%4, error);
		counter++;
		double travel = error + current + mEncoder.getOffset();
		if (Math.abs(error) > 90){
			mEncoder.setAdd180(!mEncoder.getAdd180());
			setInverted(!mDrive.getInverted());
			travel = ResourceFunctions.putAngleInRange(error + 180) + current + mEncoder.getOffset();
		}
		travel %= 4096;
		mTurn.set(ControlMode.Position, ResourceFunctions.angleToTick(travel));*/
		double current = ResourceFunctions.tickToAngle(mTurn.getSelectedSensorPosition(0)); 
		double error = ResourceFunctions.continuousAngleDif(pTarget, 
				ResourceFunctions.putAngleInRange(current - mEncoder.getOffset()));
		if (Math.abs(error) > 90){
			mEncoder.setAdd180(!mEncoder.getAdd180());
			setInverted(!mDrive.getInverted());
			error = ResourceFunctions.continuousAngleDif(pTarget, ResourceFunctions.putAngleInRange(current + 180));
		}
		if (Math.abs(error) < 5) {
			error = 0;
		}
		SmartDashboard.putNumber("error" + counter%4, error);
		counter++;
		mTurn.set(ControlMode.Position, ResourceFunctions.angleToTick(current + error));
	}

	public void setTurnSpeedToZero() {
		mTurn.set(0);
	}
	
	public void forceTurnToZero() {
		this.setAngle(0);
	}
	
	public void setInverted (boolean pInverted) {
		mDrive.setInverted(pInverted);
	}
	
	public WPI_TalonSRX getTurn () {
		return mTurn;
	}
	
}