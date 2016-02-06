package ca.ualberta;

import lejos.hardware.Button;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.utility.Delay;

public class Main {
	// A is the inner-most, B is the outer
		private static EV3TouchSensor touchSensor;
		
	public static void main(String[] args) {
		while (true){
		// maybe make menu on brick??
		//doAngleBWlines(); 
		//getDistance(); 
		//testFWDByHand(); 
		//doForward2DbyAngle(90, 90);
		Point3D test_point = new Point3D(200,80);
		testInverse2D(test_point);
		
		Button.waitForAnyPress();
		}
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
		
		//somewhere x and y are switched
		Point3D end = RobotController.getLocation();
		int[] theta = RobotController.getJointAngles();
		System.out.format("target= (%f,%f) \nreal= (%f,%f) \n th = [%f, %f]", 
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
		if (touchSensor == null) {
			touchSensor = new EV3TouchSensor(SensorPort.S1);
		}
		float[] sample = new float[touchSensor.sampleSize()];
		while (pnum < num_points){
			touchSensor.getTouchMode().fetchSample(sample, 0);  
			if (sample[0] == 1){
				points[pnum] = RobotController.getLocation();
				System.out.format("point%f= %f,%f\n", pnum, points[pnum].x, points[pnum].y);
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
