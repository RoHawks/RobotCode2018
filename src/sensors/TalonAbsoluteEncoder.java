package sensors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import resource.ResourceFunctions;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class TalonAbsoluteEncoder extends RotationInputter {
	TalonSRX mTalon;
	
	public TalonAbsoluteEncoder (TalonSRX pTalon, boolean pReverse, double pOffset) {
		super(pReverse, pOffset);
		mTalon = pTalon;
		
		mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
	}
	
	public double getRawAngleDegrees() {
		return ResourceFunctions.putAngleInRange(mTalon.getSelectedSensorPosition(0) * 360 / 4096 - this.getOffset());
	}
	public double getAngleDegrees()
    {
		double angle = getRawAngleDegrees();// * (mReversed ? -1 : 1); //Removed this due to multiple things reversing the direction
		angle = this.getAdd180() ? angle + 180 : angle;
		angle = ResourceFunctions.putAngleInRange(angle);
   	 
   	 	return angle;
    }

}
