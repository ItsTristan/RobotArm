package ca.ualberta;

import java.awt.Point;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

public class Main {
	// A is the inner-most, B is the outer
		static RegulatedMotor A = new EV3LargeRegulatedMotor(MotorPort.A);
		static RegulatedMotor B = new EV3LargeRegulatedMotor(MotorPort.B);
		private static EV3TouchSensor touchSensor;
		
	public static void main(String[] args) {
		// maybe make menu on brick??
		//doAngleBWlines(); 
		//getDistance(); 
		//testFWDByHand(); 
		//doForward2DbyAngle(90, 90);
		Point test_point = new Point(100,100);
		testInverse2D(test_point);
	}
	
	/** Waits for two target points selected and calculates midpoint
	 * then moves robot end effector to the midpoint
	 * @return 
	 */
	private static void moveToMidpoint(){
		Point[] points = getSensorPoints(2);
		int x_avg = (points[0].x + points[1].x)/2;
		int y_avg = (points[0].y + points[1].y)/2;
		Point midpoint = new Point(x_avg, y_avg);
		testInverse2D(midpoint);
	}
	
	/** given target point moves robot to that location
	 * @param target point to move to
	 * @return 
	 */
	private static void testInverse2D(Point target) {
		int[] angles = Kinematics.inverseAnalyticKinematics(target);
		RobotController.rotateTo(angles);
	}
	
	/**waits for two points selected by pressing sensor button and 
	 * calculates distance between them
	 * @return distance between the points
	 */
	private static int getDistance(){
		Point[] points = getSensorPoints(2);
		int distance = (int) points[0].distance(points[1]);
		System.out.format("point1= (%d,%d) \npoint2= (%d,%d) \ndistance: %d", 
						points[0].x, points[0].y, points[1].x, points[1].y, distance);
		Button.waitForAnyPress();
		return distance;
	}
	
	/**
	 * Waits for sensor presses and gets the point where it was pressed
	 * @param num_points is number of points/presses to wait for
	 * @return points
	 */
	private static Point[] getSensorPoints(int num_points) {
		Point[] points = new Point[num_points];
		int pnum = 0;
		if (touchSensor == null) {
			touchSensor = new EV3TouchSensor(SensorPort.S1);
		}
		float[] sample = new float[touchSensor.sampleSize()];
		while (pnum < num_points){
			touchSensor.getTouchMode().fetchSample(sample, 0);  
			if (sample[0] == 1){
				points[pnum] = Kinematics.forwardKinematics(new int[]{A.getTachoCount(), B.getTachoCount()});
				System.out.format("point%d= %d,%d\n", pnum, points[pnum].x, points[pnum].y);
				pnum++;
			}
			Delay.msDelay(300);
		}
		return points;
	}
	
	private static void doAngleBWlines() {
		Point[] cba = getSensorPoints(3);
		int angle = Kinematics.getAngleBWlines(cba[0], cba[1], cba[2]);
		System.out.format("inter ang= %d \n", angle);
		Button.waitForAnyPress();
	}

	private static void doForward2DbyAngle(int angleA, int angleB) {
		//print starting location
		Point p = Kinematics.forwardKinematics(new int[]{A.getTachoCount(), B.getTachoCount()});
		System.out.format("x = %d \ny= %d\n", p.x,p.y);
		angleA = 2*angleA; // for use of gears
		A.rotateTo(angleA);
		B.rotateTo(angleB);
		Delay.msDelay(200);
		
		int At = A.getTachoCount();
		int Bt = B.getTachoCount();
		p = Kinematics.forwardKinematics(new int[]{At, Bt});
		System.out.format("x = %d \ny= %d \nTachoA: %d \nTachoB: %d\n", p.x,p.y, At,Bt );
		Button.waitForAnyPress();
	}

	private static void testFWDByHand() {
		while (true) {
			Point[] p = getSensorPoints(1);	
			Button.waitForAnyPress();	
		}
	}

	public static boolean check_fields(int source, int flag) {
		return (source & flag) != 0;
	}
}
