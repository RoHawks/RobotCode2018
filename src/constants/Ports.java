package constants;

public class Ports {
	
	public static final int[] 
		TURN = new int[] { 0, 2, 4, 6 },
		DRIVE = new int[] { 1, 3, 5, 7 }; //sw, SE, NE, NW
	
	public static final int INTAKE = 8;		
	
	public static final int LIMITSWITCH = 0, BREAKBEAM = 1;
	
	//pistons
	public static final int
		LEFT_INTAKE_IN = 6,
		LEFT_INTAKE_OUT = 7, 
		RIGHT_INTAKE_IN = 0,
		RIGHT_INTAKE_OUT = 1;
}
