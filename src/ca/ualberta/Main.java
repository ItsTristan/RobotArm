package ca.ualberta;

import java.io.FileNotFoundException;
import java.io.PrintWriter;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.utility.Delay;

public class Main {
	// A is the inner-most, B is the outer
		private static EV3TouchSensor touchSensor;
		public static int num_modes = 12;
			
	/**
	 * Displays the main menu and allows the user to select
	 * options.
	 * Up/Down selects the previous/next mode, respectively
	 * Enter runs the currently selected option
	 * Back quits the program.
	 */
	public static void main(String[] args) {
		RobotController.initializeMotorZero();
		int mode = 0;
		System.out.println("\n\nMainMenu:");
		
		while (true){
			printModeInfo(mode);
			
			int button = Button.waitForAnyPress();
			if (button == Button.ID_ENTER) {
				selectMode(mode);
				Button.waitForAnyPress();
				RobotController.resetMotors();
				System.out.println("\n\nMainMenu:");
				
			} else if (button == Button.ID_UP) {
				mode = (mode+num_modes-1) % num_modes;
			} else if (button == Button.ID_DOWN) {
				mode = (mode+1) % num_modes;
			} else if (button == Button.ID_ESCAPE) {
				break;
			}
		}
		
		return;
	}
	
	/**
	 * Selects a menu option given some integer choice
	 * @param choice the option to select
	 */
	public static void selectMode(int choice) {
		// Clear screen and print choice
		System.out.println("\n\n\n\n\n\n\n");
		printModeInfo(choice);
		
		switch(choice) {
		case 0:
			// Part 1 - Moves to the given angles and reports
			// XXX
			doForward2DbyAngle(90,0);
			break;
		case 1:
			// Part 2 - Select point and get distance between them
			getDistance();
			break;
		case 2:
			// Part 3 - First is intersecting, second and third are arms
			// Reports the angle between the two lines at the intersection
			doAngleBWlines();
			break;
		// XXX To change to analytic solution, change the default
		// 		motion type in RobotController.moveTo
		case 3:
			// Part 4 - Do inverse kinematics to move to a point
			// XXX
			testInverse2D(new Point3D(200,100));
			break;
		case 4:
			// Part 5 - Moves to the midpoint between two selected points
			moveToMidpoint();
			break;
		case 5:
			// Part 6 - Given two points, draws a straight line
			// XXX
			RobotController.drawLineBW(new Point3D(230, 70), new Point3D(30, 200));
			break;
		case 6:
			// Part 7 - Given a point, an angle, and a distance, draws
			// a straight line from point 
			// XXX
			drawLineFromPointAngleDist(new Point3D(0,220), -20, 200);
			break;
		case 7:
			// Part 8 - Draw an arc between 3 points
			// XXX
			RobotController.drawArcLine(new Point3D(180,-150), new Point3D(200,0), new Point3D(100,150));
			break;
		case 8:
			// Part 9 - Inverse 3D kinematics
			// XXX
			RobotController.moveTo3D(new Point3D(150,0,50));
			System.out.println("Point: " + RobotController.getLocation3());
			break;
		case 9:
			// Part ? - Trace Labyrinth
			traceMaze();
			break;
		case 10:
			// Part ? - Forward by Hand
			testFWDByHand();
			break;
		case 11:
			// Part ? - Test Trace Maze
			testTraceMaze();
			break;
		default:
			System.out.println("Unknown mode?");
			Sound.buzz();
			break;
		}
	}
	
	/**
	 * Displays the name of the mode given by choice.
	 * @param choice the menu option to display the name of.
	 */
	public static void printModeInfo(int choice) {
		System.out.print(choice + ": ");
		switch(choice) {
		case 0:
			// Part 1 - Moves to the given angles and reports
			System.out.println("Fwd by Angle");
			break;
		case 1:
			// Part 2 - Select point and get distance between them
			System.out.println("dist2Point");
			break;
		case 2:
			// Part 3 - First is intersecting, second and third are arms
			// Reports the angle between the two lines at the intersection
			System.out.println("angle BW Lines");
			break;
		case 3:
			// Part 4 - Do inverse kinematics to move to a point
			System.out.println("Inv 2D Kin");
			break;
		case 4:
			// Part 5 - Moves to the midpoint between two selected points
			System.out.println("Midpoint");
			break;
		case 5:
			// Part 6 - Given two points, draws a straight line
			System.out.println("2pt Line");
			break;
		case 6:
			// Part 7 - Given a point, an angle, and a distance, draws
			// a straight line from point 
			System.out.println("Pt/Angle Line");
			break;
		case 7:
			// Part 8 - Draw an arc between 3 points
			System.out.println("Draw Arc");
			break;
		case 8:
			// Part 9 - Inverse 3D kinematics
			System.out.println("Inv 3D Kin");
			break;
		case 9:
			// Part ? - Trace Labyrinth
			System.out.println("Labyrinth");
			break;
		case 10:
			// Part ? - Forward by Hand
			System.out.println("Test Fwd man.");
			break;
		case 11:
			// Part ? - Test Trace Maze
			System.out.println("DBG: Labyrinth");
			break;
		default:
			System.out.println("Unknown mode?");
			Sound.buzz();
			break;
		}
	}
	/** tests tracing a maze and records all the points in a file so 
	 * we may repeat it later. 
	 */
	private static void testTraceMaze() {
		String filename = "maze.txt";
		Point3D[] maze_points = traceMaze();	
		PrintWriter out = null;
		try {
			out = new PrintWriter(filename);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		for (int p = 0; p < maze_points.length; p++){
			if (maze_points[p] != null){
				out.printf("(%f,%f)\n", maze_points[p].x, maze_points[p].y);
			}
		}
		out.close();
	}	
		
	private static Point3D[] traceMaze(){
		int max_nodes = 30;
		Point3D[] path = getSensorPoints(max_nodes);
		for ( int i = 1; i < path.length-1; i++ ) {
			if (path[i] == null){
				return path;
			}
			RobotController.drawLineBW(path[i-1], path[i]);
		}
		return path;
	}
	/**
	 * Takes a point, an angles in degrees and a distance and draws a straight line 
	 * from the point in the direction of the angle the length of the distance.
	 * @param start a Point3D
	 * @param angle in degrees
	 * @param distance float in millimeters
	 */
	private static void drawLineFromPointAngleDist(Point3D start, int angle, double distance){
		double delta_x = distance*Math.cos(Math.toRadians(angle));
		double delta_y = distance*Math.sin(Math.toRadians(angle));
		Point3D end_point = new Point3D(start.x + delta_x, start.y + delta_y);
		RobotController.drawLineBW(start, end_point);
	}
	
	/** Waits for two target points selected and calculates midpoint
	 * then moves robot end effector to the midpoint
	 * @return 
	 */
	private static void moveToMidpoint(){
		Point3D[] points = getSensorPoints(2);
		double x_avg = (points[0].x + points[1].x)/2;
		double y_avg = (points[0].y + points[1].y)/2;
		Point3D midpoint = new Point3D(x_avg, y_avg);
		testInverse2D(midpoint);
	}
	
	/** given target point moves robot to that location
	 * @param target point to move to
	 * @return 
	 */
	private static void testInverse2D(Point3D target) {
		RobotController.moveTo(target.x, target.y);
		
		Point3D end = RobotController.getLocation();
		System.out.format("target= (" + target + ")\n"
						+ "real= (" + end + ")\n");
	}
	
	/**waits for two points selected by pressing sensor button and 
	 * calculates distance between them
	 * @return distance between the points
	 */
	private static double getDistance(){
		Point3D[] points = getSensorPoints(2);
		double distance = (double) points[0].distance(points[1]);
		System.out.format("point1= (%f,%f) \npoint2= (%f,%f) \ndistance: %f", 
						points[0].x, points[0].y, points[1].x, points[1].y, distance);
		Button.waitForAnyPress();
		return distance;
	}
	
	/**
	 * Waits for sensor presses and gets the point where it was pressed
	 * @param num_points is number of points/presses to wait for
	 * @return points
	 */
	private static Point3D[] getSensorPoints(int num_points) {
		Point3D[] points = new Point3D[num_points];
		int pnum = 0;
		int is_pressed = 1;
		if (touchSensor == null) {
			touchSensor = new EV3TouchSensor(SensorPort.S1);
		}
		float[] sample = new float[touchSensor.sampleSize()];
		System.out.println("select 1st point \n");
		while (pnum < num_points && Button.getButtons() != Button.ID_ENTER){
			touchSensor.getTouchMode().fetchSample(sample, 0);  
			if (sample[0] == is_pressed){
				points[pnum] = RobotController.getLocation();
				System.out.format("point%d= %f,%f\n", pnum, points[pnum].x, points[pnum].y);
				pnum++;
			}
			Delay.msDelay(300);
		}
		touchSensor.close();
		touchSensor = null;
		return points;
	}
	
	private static void doAngleBWlines() {
		Point3D[] cba = getSensorPoints(3);
		int angle = Kinematics.getAngleBWlines(cba[0], cba[1], cba[2]);
		System.out.format("inter ang= %d \n", angle);
		Button.waitForAnyPress();
	}

	private static void doForward2DbyAngle(int angleA, int angleB) {
		//print starting location
		Point3D p = RobotController.getLocation();
		System.out.format("x = %f \ny= %f\n", p.x,p.y);
		
		RobotController.rotateTo(new int[] {angleA, angleB});
		Delay.msDelay(200);
		
		p = RobotController.getLocation();
		System.out.format("x = %f \ny= %f\n", p.x,p.y);
	}

	private static void testFWDByHand() {
		while (true) {
			Point3D[] p = getSensorPoints(1);	
			Button.waitForAnyPress();	
		}
	}

	/**
	 * Converts two bitmasks into a boolean value.
	 * @param source mask containing the bit information.
	 * @param flag mask to compare against.
	 * @return true if the two inputs share some flags.
	 */
	public static boolean check_fields(int source, int flag) {
		return (source & flag) != 0;
	}
}
