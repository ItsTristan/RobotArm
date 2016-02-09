package ca.ualberta;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Collections;

import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.utility.Delay;

public class Main {
	// A is the inner-most, B is the outer
		private static EV3TouchSensor touchSensor;
			
	public static void main(String[] args) {
		RobotController.initializeMotorZero();
		while (true){
		// maybe make menu on brick??
		//doAngleBWlines(); 
		//getDistance(); 
		//testFWDByHand(); 
		//doForward2DbyAngle(90, 90);
			//testTraceMaze();
			//Kinematics.getCenterArc(new Point3D(2,-4), new Point3D(6,-2), new Point3D(5,5));
			Point3D[] arcTest = getSensorPoints(3);
			Button.waitForAnyPress();
			RobotController.drawArcLine(arcTest[0] , arcTest[1], arcTest[2]);
//			System.out.println("Move far");
//			RobotController.moveTo(new Point3D(0,240));
			
//			System.out.println("Draw Line");
//			RobotController.drawLineBW(new Point3D(230, -70), new Point3D(30,200));
//			drawLineFromPointAngleDist(new Point3D(0,220), -20, 100.0);
			
//			System.out.println("Move to midpoint");
//			moveToMidpoint();
			Button.waitForAnyPress();
			RobotController.relaxMotors();
			//System.out.println("RIGHTBEFORELURGE!!!");
			//Button.waitForAnyPress();
			
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
		RobotController.moveTo(target);
		
		Point3D end = RobotController.getLocation();
		int[] theta = RobotController.getJointAngles();
		System.out.format("target= (%f,%f) \nreal= (%f,%f) \n th = [%d, %d]", 
				target.x, target.y, end.x, end.y, theta[0], theta[1]);
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
		
		int[] theta = RobotController.getJointAngles();
		p = Kinematics.forwardKinematics(new int[]{theta[0], theta[1]});
		System.out.format("x = %f \ny= %f \nTachoA: %d \nTachoB: %d\n", p.x,p.y,theta[0], theta[1] );
		Button.waitForAnyPress();
	}

	private static void testFWDByHand() {
		while (true) {
			Point3D[] p = getSensorPoints(1);	
			Button.waitForAnyPress();	
		}
	}

	public static boolean check_fields(int source, int flag) {
		return (source & flag) != 0;
	}
}
