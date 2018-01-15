package sensors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class TalonRelativeEncoder extends RelativeEncoder {
	private TalonSRX mTalon;
	private boolean mReversed;
	
	public TalonRelativeEncoder (TalonSRX pTalon, boolean pReversed, double pTicksToRPM) {
		super(pTicksToRPM);
		mTalon = pTalon;
		mReversed = pReversed;
		
		mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		mTalon.setInverted(mReversed);
	}

	public double getRawTicksPerSecond() {
		return mTalon.getSelectedSensorVelocity(0);
	}
}
