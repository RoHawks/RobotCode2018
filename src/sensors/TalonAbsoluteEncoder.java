package sensors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class TalonAbsoluteEncoder extends RotationInputter {
	TalonSRX mTalon;
	
	public TalonAbsoluteEncoder (TalonSRX pTalon, boolean pReverse, double pOffset) {
		super(pReverse, pOffset);
		mTalon = pTalon;
		
		mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
	}
	
	@Override
	protected double getRawAngleDegrees() {
		return mTalon.getSelectedSensorPosition(0) * 360 / 4096;
	}

}
