package constants;

public class DriveConstants {

	// speed mins, when lower than these don't do anything
	public static final double 
			MIN_ANGULAR_VEL = 0.01,
			MIN_ANGULAR_VEL_STICK = 0.2, 
			MIN_LINEAR_VEL = 0.02,
			MIN_DIRECTION_MAG = 0.75, // refers to left joystick magnitude for choosing swerve direction
			MAX_INDIVIDUAL_VELOCITY = 1.0;
	
	public static final double
			EMERGENCY_VOLTAGE = 10000,
			MAX_EMERGENCY_VOLTAGE = 0.5;
	
	public static final double 
			MAX_ANGULAR_VEL = 0.4,
			MAX_LINEAR_VEL = 0.3;

	public static class SwerveSpeeds {
		public static final double 
				SPEED_MULT = 1.0,
				ANGULAR_SPEED_MULT = 1.0,
				NUDGE_MOVE_SPEED = 0.2,
				NUDGE_TURN_SPEED = 0.2;
	}

	public static class Modules{
		public static final boolean[]
				TURN_INVERTED = new boolean[] {false, false, false, true},
				INVERTED = new boolean[] { false, false, false, false },
				ENCODER_REVERSED = new boolean[] {true, true, true, false};
		
		public static final double[] 
				X_OFF = new double[] { -19.0 / 2.0, -19.0 / 2.0, 19.0 / 2.0, 19.0 / 2.0 },
				Y_OFF = new double[] { -22.0 / 2.0, 22.0 / 2.0, 22.0 / 2.0, -22.0 / 2.0 };

		public static final int[] OFFSETS = new int[] { 3721 , 767, 3402, 2527 };
		
	}


	public static class PID_Constants {

		public static final double[] 
				ROTATION_P = new double[] { 0.7, 0.7, 0.7, 0.7 },
				ROTATION_I = new double[] { 0.007, 0.007, 0.007, 0.007 },
				ROTATION_D = new double[] { 0, 0, 0, 0 };
		
		public static final int[]
				ROTATION_IZONE = new int[] { 15, 15, 15, 15 };

		// for turning robot to an angle
		public static final double
				GYRO_P = 0.03,
				GYRO_I = 0,
				GYRO_D = 0,
				GYRO_TOLERANCE = 5;
		}
}
