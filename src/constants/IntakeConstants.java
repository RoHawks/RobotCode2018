package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeConstants {
	//pneumatic states
	public static final Value
		OPEN = Value.kReverse,
		CLOSED = Value.kForward;
	
	
	//break beam normally open or closed
	public static final boolean BOX_PRESENT_BREAK_BEAM = false, BOX_PRESENT_LIMIT_SWITCH = true;

	//time to activate pistons
	public static final double 
		SNATCHER_TIME_RELEASE_PRESSURE_OPEN = 0.04,
		SNATCHER_TIME_RELEASE_PRESSURE_CLOSE = 0.12,
		SNATCHER_TIME_OPEN = 0.3,
		SNATCHER_TIME_CLOSE = 0.3,
		WAIT_AFTER_ALIGNED = 0.2,
		WAIT_AFTER_BUMPSWITCH = 0.2;

	public static final double  //Intake wheel speed
		INTAKE_SPEED = -1;
}
