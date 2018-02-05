package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeConstants {
	//pneumatic states
	public static final Value
		OPEN = Value.kReverse,
		CLOSED = Value.kForward;

	//break beam normally open or closed
	public static final boolean 
		BOX_PRESENT_BREAK_BEAM = false, 
		BOX_PRESENT_LIMIT_SWITCH = true;

	//time to activate pistons, haven't calculated yet
	/*public static final double 
		TIME_RELEASE_PRESSURE_OPEN = 0.04,
		TIME_RELEASE_PRESSURE_CLOSE = 0.12,
		TIME_OPEN = 0.3,
		TIME_CLOSE = 0.3;*/

	public static final double INTAKE_SPEED = -1; //Intake wheel speed
		
}
