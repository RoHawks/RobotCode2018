package constants;

public class DriveConstants {
	public static final double RAW_TO_RPS = (1.0 / 60.0) * (34.0 / 70.0);

	// speed mins, when lower than these don't do anything
	public static final double 
			MIN_ANGULAR_VEL = 0.01,
			MIN_ANGULAR_VEL_STICK = 0.2, 
			MIN_LINEAR_VEL = 0.02,
			MIN_DIRECTION_MAG = 0.75, // refers to left joystick magnitude for choosing swerve direction
			MAX_INDIVIDUAL_VELOCITY = 1.0;
	
	public static final double  //Intake wheel speed
			RIGHT_INTAKE_SPEED = -0.5,
			LEFT_INTAKE_SPEED = 0.5;

	public static final double 
			MAX_ANGULAR_VEL = 0.4,
			MAX_LINEAR_VEL = 0.3,
			MAX_TURN_VEL = 0.4;
/*	public static final int MIN_TIME_IN_RANGE_NUDGE_MILLIS = 300;
	public static final double NUDGE_ANGLE_TOLERANCE = 6;

	public static final double MAX_RPS = 9;

	public static final boolean USING_DRIFT_COMP_PID = true;

	public static final double MAX_VOLTAGE = 12.0, VOLTAGE_RAMP_MILLIS = 25;

	public static final double EMERGENCY_VOLTAGE = -1, MAX_VOLTAGE_EMERGENCY = 12.0;*/

	public static class SwerveSpeeds {
		public static final double SPEED_MULT = 1.0, ANGULAR_SPEED_MULT = 1.0,
				NUDGE_MOVE_SPEED = 0.5, NUDGE_TURN_SPEED = 0.5;
	}

	public static class Modules{
		public static final boolean[]
				ENCODER_REVERSED = new boolean[] {true, true, true, false},
				TALON_TURN_INVERTED = new boolean[] /*{true, true, true, false},*/{false, false, false, true},
				INVERTED = new boolean[] { true, true, true, true },
				TURN_INVERTED = new boolean[] { false, false, false, true },
				TALON_ENCODER_REVERSED = new boolean[] { false, false, false, true } ; //last one true
		public static final double[] 
				/*X_OFF = new double[] { 19.0 / 2.0, 19.0 / 2.0, -19.0 / 2.0, -19.0 / 2.0 },
				Y_OFF = new double[] { -22.0 / 2.0, 22.0 / 2.0, 22.0 / 2.0, -22.0 / 2.0 };*/
				X_OFF = new double[] { -19.0 / 2.0, -19.0 / 2.0, 19.0 / 2.0, 19.0 / 2.0 },
				Y_OFF = new double[] { -22.0 / 2.0, 22.0 / 2.0, 22.0 / 2.0, -22.0 / 2.0 };

		public static final int[] OFFSET = new int[] { -1663, 1237, -1448, 3648 };
	}


	public static class PID_Constants {
		public static final double 
			MAX_ANGULAR_VELOCITY_COMPENSATE = 2,
			TIME_AFTER_TURNING_ACTIVATE_MILLIS = 500;
		public static final double[] 
				ROTATION_P = new double[] { 0.02, 0.02, 0.02, 0.02 },
				ROTATION_I = new double[] { 0.00025, 0.0003, 0.00025, 0.00025 },
				ROTATION_D = new double[] { 0, 0, 0, 0 };
		
		public static final int[]
				ROTATION_IZONE = new int[] { 15, 15, 15, 15 };

		// for turning robot to an angle
		public static final double 
				ANGLE_P = 0.025,
				ANGLE_I = 0.005, 
				ANGLE_D = 0.04, 
				ANGLE_IZONE = 5, 
				ANGLE_MAX = 0.4,
				ANGLE_MIN = 0.1, 
				ANGLE_DEADBAND = 2;
		}
}
