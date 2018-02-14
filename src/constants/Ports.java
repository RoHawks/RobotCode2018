package constants;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Ports {
	
	public static final int[] 
		TURN = new int[] { 0, 2, 4, 6 },
		DRIVE = new int[] { 1, 3, 5, 7 }; //SW, SE, NE, NW
	
	public static final int INTAKE = 8;		
	
	public static final int 
		LIMITSWITCH = 1,
		BREAKBEAM = 0;
	
	//Intake Pistons
	public static final int
		LEFT_INTAKE_IN = 6,
		LEFT_INTAKE_OUT = 7, 
		
		RIGHT_INTAKE_IN = 0,
		RIGHT_INTAKE_OUT = 1;
	
	public static final SerialPort.Port NAVX = Port.kUSB;
	
	//controllers
	public static final int
		XBOX = 0,
		JOYSTICK = 1;
	
	public static final int COMPRESSOR = 0;
}
